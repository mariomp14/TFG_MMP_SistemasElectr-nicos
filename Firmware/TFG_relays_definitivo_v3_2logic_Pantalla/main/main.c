#include "LoRa.h"
#include "esp_netif.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/portmacro.h"
#include "sht40.h"
#include "driver/gpio.h"
#include "esp_err.h"
//#include "freertos/projdefs.h"
#include "hal/gpio_types.h"
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "string.h"
#include "freertos/timers.h"
#include "esp_sleep.h"

#include "nvs_flash.h"
#include "mqtt_client.h"
#include "my_mqtt/include/my_mqtt.h"
#include "my_wifi/include/my_wifi.h"

#include "screen.h"
#include "ui.h"
#include <math.h>  // Para la función exp()

static const char *TAG = "LoRa_App";
static void safe_update_labels(void *param);
 
#define RELAY_1 GPIO_NUM_6
#define RELAY_2 GPIO_NUM_7
#define RELAY_3 GPIO_NUM_8
#define RELAY_4 GPIO_NUM_9
//Mecanismos de freeRTOS
static TaskHandle_t manejador_tarea_relays = NULL; 
static TaskHandle_t manejador_tarea_receiver = NULL; 
static QueueHandle_t lora_queue;  // Cola para almacenar los mensajes LoRa
static QueueHandle_t myQueueSet;
static SemaphoreHandle_t lora_semaphore;  // Semáforo para manejar la interrupción
static SemaphoreHandle_t security_semaphore_Sector1; 
static SemaphoreHandle_t security_semaphore_Sector2; 
SemaphoreHandle_t systemReadySemaphore;
TimerHandle_t timer_security_Sector1 = NULL; 
TimerHandle_t timer_security_Sector2 = NULL; 

QueueHandle_t cola_consignas;


RTC_DATA_ATTR volatile uint16_t light_sleep_time = 210;// 3 minutos y medio																																																
RTC_DATA_ATTR volatile uint32_t time_security_s = 180;
RTC_DATA_ATTR volatile uint16_t	convert_ms = 1000;

uint8_t MasterNode = 0xFF;//Nodo que controla la nebulización
uint8_t Node1 = 0xBB;
uint8_t Node2 = 0xAA;

#define BOTON_GPIO GPIO_NUM_41


//Funciones y tareas
static portTASK_FUNCTION(RELAYS, pvParameters);
static portTASK_FUNCTION(RECEIVER, pvParameters);
void configure_send_ack(uint8_t Node);
void setup_interrupt(void);
void procesar_buffer(uint8_t *buf);
static void  gpio_isr_handler(void *arg);
void CONSIGNMENTS(void *param);
uint8_t calcular_dpv(float temperatura, float humedad_relativa, uint8_t sector);
int automatismo_desactivado(void);
void actualizar_label_async(lv_obj_t *label, const char *nuevo_texto);
void inicializar_nvs_por_defecto();

 DatoLVGLtemporal temp_S1 = {0};  
 DatoLVGLtemporal hum_S1 = {0}; 
 DatoLVGLtemporal temp_S2 = {0}; 
 DatoLVGLtemporal hum_S2 = {0}; 
 DatoLVGLtemporal temp_suelo = {0}; 
 DatoLVGLtemporal hum_suelo = {0}; 
 DatoLVGLtemporal cond_suelo = {0}; 
 DatoLVGLtemporal ph_suelo = {0}; 
 DatoLVGLtemporal DPVS1 = {0};
 DatoLVGLtemporal DPVS2 = {0};
 DatoLVGLtemporal DPVminS1 = {0};
 DatoLVGLtemporal DPVminS2 = {0};
 DatoLVGLtemporal DPVmaxS1 = {0};
 DatoLVGLtemporal DPVmaxS2 = {0};
 DatoLVGLbooltemporal Rele1 = {0}; 
 DatoLVGLbooltemporal Rele2= {0}; 
 DatoLVGLbooltemporal estado_automatismo= {0}; 

void IRAM_ATTR boton_isr_handler(void* arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(semaforo_activar_timer_screen, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}



void app_main(void)
{
	
	systemReadySemaphore = xSemaphoreCreateBinary();  // Crear el semáforo global
	
	//WIFI e MQTT//
	esp_log_level_set("wifi", ESP_LOG_VERBOSE);//Habilita los logs detallados de Wi-Fi para ver mensajes de error:
	 									//Poner un comando desde mqtt para eliminar la flash?
	//nvs_flash_erase(); 					//Solo para pruebas, eliminar?!->borra memoria NVS. Ojo si pongo esto no tendria sentido guardarlo en la memoria volátil no?
    ESP_ERROR_CHECK(nvs_flash_init());  //Se usa para guardar datos en memoria no volátil, como credenciales WiFi, configuraciones o tokens de acceso.
    inicializar_nvs_por_defecto();  // para guardar los valores de dpv max y min y si esta o no desactivado el automatismo
    esp_netif_init();					//inicializa la pila de red, para poder usar wifi
    esp_event_loop_create_default();	//crea e inicializa el bucle de eventos predeterminado en esp-idf (wifi,bluetooth...)
    wifi_connect_sta();
	

    ESP_LOGI("MAIN", "Esperando WiFi y MQTT...");
    xSemaphoreTake(systemReadySemaphore, portMAX_DELAY);
    ESP_LOGI("MAIN", "WiFi y MQTT conectados. Continuando ejecución...");
    
	/////////////////////////
	
    // Inicialización de LoRa
    ESP_LOGI(TAG, "Inicializando LoRa...");
    if (!lora_init()) {
        ESP_LOGE(TAG, "Error al inicializar LoRa");
        return;
    }
    
    // Configurar el pin de interrupción DIO0
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_POSEDGE;  // Interrupción en flanco de subida
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << LORA_DIO0_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    // Instalar ISR y registrar el manejador de interrupción
    gpio_install_isr_service(0);  
    gpio_isr_handler_add(LORA_DIO0_PIN, gpio_isr_handler, NULL);
    
    /////////////////////////////////////////////////
    //Timer de seguridad de 3.5mins
    timer_security_Sector1 = xTimerCreate("TimerWakeup1", pdMS_TO_TICKS(210000), pdFALSE, NULL, parpadeoAsimCallbackRelay1);
	timer_security_Sector2 = xTimerCreate("TimerWakeup2", pdMS_TO_TICKS(210000), pdFALSE, NULL, parpadeoAsimCallbackRelay2);
    // Configuración básica
    ESP_LOGI(TAG, "Configurando LoRa...");
    lora_set_frequency(868E6);       // Configurar frecuencia (por ejemplo, 868 MHz)
    ESP_LOGI(TAG, "Frecuencia configurada a 868 MHz");
    
    lora_set_tx_power(10);           // Configurar potencia de transmisión
    ESP_LOGI(TAG, "Potencia de transmision configurada a 10 dBm");
    

    ESP_LOGI(TAG, "CRC habilitado para asegurar la integridad de los datos");
	
	//MIRAR que otros parámetros de la comunicación LoRa y de la trama se pueden modificar*
	
    
    lora_receive();//Pongo LoRa en modo recepción
	gpio_set_direction(RELAY_1, GPIO_MODE_OUTPUT); // Configurar pin  como salida para el relé 1
	gpio_set_direction(RELAY_2, GPIO_MODE_OUTPUT); // Configurar pin  como salida para el relé 2
    gpio_set_direction(RELAY_3, GPIO_MODE_OUTPUT); // Configurar pin  como salida para el relé 2
  	lora_enable_crc();

	// Crear la cola (10 mensajes de 6 bytes cada uno)
	//La FIFO del módulo LoRa tiene 256 bytes, así que en tamaño de la cola máximo
    //podría ser 256bytes/tamaño del paquete, pero lo pongo de tamaño 10 para empezar
    lora_queue = xQueueCreate(10, sizeof(char) * 14);// ya que recibo 14 bytes comomaximo del Nodo2
    if (lora_queue == NULL) 
    {
        ESP_LOGE(TAG, "Error al crear la cola");
        while (1);
    }
    mqtt_queue = xQueueCreate(10, sizeof(mqtt_message_t));  // Crear la cola con 10 mensajes
    
    if (mqtt_queue == NULL) 
    {
        ESP_LOGE(TAG, "Error al crear la cola");
        while (1);
    }

    // Crear semáforo binario
    lora_semaphore = xSemaphoreCreateBinary();
    security_semaphore_Sector1 = xSemaphoreCreateBinary();
    security_semaphore_Sector2 = xSemaphoreCreateBinary();
    mutexUmbrales = xSemaphoreCreateMutex(); // Crear el mutex
    // Crear un QueueSet con espacio para ambos
    myQueueSet = xQueueCreateSet(10 + 2);  // Máximo de elementos en la cola + semáforos
    
    // Agregar la cola y el semáforo al QueueSet
    xQueueAddToSet(lora_queue, myQueueSet);
    xQueueAddToSet(security_semaphore_Sector1, myQueueSet);
    xQueueAddToSet(security_semaphore_Sector2, myQueueSet);
    
    //PANTALLA
    screen_mutex = xSemaphoreCreateMutex();
	semaforo_activar_timer_screen= xSemaphoreCreateBinary();
	
	gpio_set_direction(TFT_BL, GPIO_MODE_OUTPUT);
	gpio_set_level(TFT_BL, 0);  // Inicia apagado
    gpio_config_t io_conf2 = {
        .pin_bit_mask = 1ULL << BOTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&io_conf2);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BOTON_GPIO, boton_isr_handler, NULL);
	
	apagar_screen_timer = xTimerCreate(
    "apagar_timer",
    pdMS_TO_TICKS(2 * 60 * 1000),  // 2 minutos
    pdFALSE,
    NULL,
    apagar_pantalla  // nueva función callback
	);
	 //Tarea y cola que gestionan las consignas recibidas por pantalla (y por mqtt???)
	cola_consignas = xQueueCreate(10, sizeof(ConsignasLVGL));
	
	//
	timerSector1 = xTimerCreate("ParpadeoAsim1", tiempo_off, pdFALSE, NULL, parpadeoAsimCallbackRelay1);
	timerSector2 = xTimerCreate("ParpadeoAsim2", tiempo_off, pdFALSE, NULL, parpadeoAsimCallbackRelay2);
	estado_Rele_Sector1 = 0;//se inicializan apagados!
	estado_Rele_Sector1 = 0;
	// 
	xReleMutex = xSemaphoreCreateMutex();
	xTaskCreate(CONSIGNMENTS, "TareaConsignas", 4096, NULL, 1, NULL);
 	//Tarea que gestiona los relés y las recepciones LoRa
 	//tambien puedo utilizar x TaskCreatePinnedToCore...
  	xTaskCreate(RELAYS, "SendMsgTask", 4096, NULL, 1, &manejador_tarea_relays);
	xTaskCreate(RECEIVER, "SendMsgTask", 4096, NULL, 1, &manejador_tarea_receiver);
   	xTaskCreate(MQTT, "MQTT", 4096, NULL, 5, NULL);
   	//Tareas pantalla
	xTaskCreatePinnedToCore(
                    SCREEN,
                    "screen_task",
                    12288,//8192
                    NULL,
                    5,
                    &screen_task_handle,
                    1
                );
    xTaskCreate(SCREEN_ON_OFF, "tarea_control_pantalla", 4096, NULL, 2, NULL);
   	//vTaskStartScheduler();	//No es necesario llamarlo explicitamente en el esp32

}

void vTimerAckCallback_Sector1(TimerHandle_t xTimer) {
    ESP_LOGI(TAG, "Temporizador Seguridad Sector 1 activado!");
    xSemaphoreGive(security_semaphore_Sector1);
}

void vTimerAckCallback_Sector2(TimerHandle_t xTimer) {
    ESP_LOGI(TAG, "Temporizador Seguridad Sector 2 activado!");
    xSemaphoreGive(security_semaphore_Sector2);
}

/* Light Sleep suspende la CPU y puede afectar la conexión TCP
Aunque WiFi se mantiene activo en "Modem Sleep", la conexión TCP/MQTT podría cerrarse si no responde a tiempo.
Esto sucede porque MQTT usa una conexión TCP persistente, y si la conexión se interrumpe por Light Sleep,
 el ESP32 intentará reconectar automáticamente.
 void sleep_now(void) -> Por tanto he eliminado esta función
*/

void CONSIGNMENTS(void *param) {//tarea consginas desde pantalla?
    ConsignasLVGL evento;
    nvs_handle_t handle;
    esp_err_t err;

    while (1) {
        if (xQueueReceive(cola_consignas, &evento, portMAX_DELAY)) {
            // Abrimos el espacio "consignas" en la NVS
            err = nvs_open("storage", NVS_READWRITE, &handle);
            if (err != ESP_OK) {
                printf("Error abriendo NVS: %s\n", esp_err_to_name(err));
                continue;
            }

            switch (evento.tipo) {
                case EVENTO_DPV_MINS1:
                    err = nvs_set_i32(handle, "DPVMinS1", evento.valor);
                    break;

                case EVENTO_DPV_MAXS1:
                    err = nvs_set_i32(handle, "DPVMaxS1", evento.valor);
                    break;

                case EVENTO_DPV_MINS2:
                    err = nvs_set_i32(handle, "DPVMinS2", evento.valor);
                    break;

                case EVENTO_DPV_MAXS2:
                    err = nvs_set_i32(handle, "DPVMaxS2", evento.valor);
                    break;

                case EVENTO_DESACTIVAR_AUTOMATISMO:
                    err = nvs_set_i32(handle, "AutoOFF", evento.valor);  // 0 o 1
                    break;

                case EVENTO_NEBULIZARS1:
                   // err = nvs_set_i32(handle, "NebS1", evento.valor);
                    //gpio_set_level(RELAY_1, (evento.valor == 1) ? 0 : 1);
                    gpio_set_level(RELAY_3, evento.valor);
                    
                    if(evento.valor == 0)
                    { 
						xTimerStop(timerSector1, portMAX_DELAY);
						gpio_set_level(RELAY_1, 0);  // apagar relé,por si justo en el momento de parar el timer estaba encendido!-> (por el blink)
                  		if (xSemaphoreTake(xReleMutex, portMAX_DELAY)) {
                  		estado_Rele_Sector1 = 0;
                  		if(!estado_Rele_Sector2)
                  		{
							  gpio_set_level(RELAY_3, 0); //se apaga rele que controla flujo general del agua
						 }
                  		xSemaphoreGive(xReleMutex);
                  		}
                  		esp_mqtt_client_publish(client, "mariomp/f/activacionRele1", "OFF", 0, 1, 0);
                   	}
                   	else 
                   	{
						xTimerStart(timerSector1, portMAX_DELAY);
						if (xSemaphoreTake(xReleMutex, portMAX_DELAY)) {
                  		estado_Rele_Sector1 = 1;
                  		xSemaphoreGive(xReleMutex);
                  		}
                  		gpio_set_level(RELAY_3, 1);//general
						esp_mqtt_client_publish(client, "mariomp/f/activacionRele1", "ON", 0, 1, 0);
					}
                    break;

                case EVENTO_NEBULIZARS2:
                    //err = nvs_set_i32(handle, "NebS2", evento.valor);
                    //gpio_set_level(RELAY_2, (evento.valor == 1) ? 0 : 1);
                    gpio_set_level(RELAY_3, evento.valor);
                    
                     if(evento.valor == 0)
                    { 
						xTimerStop(timerSector2, portMAX_DELAY);
						gpio_set_level(RELAY_2, 0);  // apagar relé,por si justo en el momento de parar el timer estaba encendido!
                  		if (xSemaphoreTake(xReleMutex, portMAX_DELAY)) {
                  		estado_Rele_Sector2 = 0;
                  		if(!estado_Rele_Sector1)
                  		{
							  gpio_set_level(RELAY_3, 0); //se apaga rele que controla flujo general del agua
						 }
                  		xSemaphoreGive(xReleMutex);
                  		}
                  		esp_mqtt_client_publish(client, "mariomp/f/activacionRele2", "OFF", 0, 1, 0);
                   	}
                   	 else 
                   	{
						xTimerStart(timerSector2, portMAX_DELAY);
						if (xSemaphoreTake(xReleMutex, portMAX_DELAY)) {
                  		estado_Rele_Sector2 = 1;
                  		xSemaphoreGive(xReleMutex);
                  		}
                  		gpio_set_level(RELAY_3, 1);//general
						esp_mqtt_client_publish(client, "mariomp/f/activacionRele2", "ON", 0, 1, 0);
					}
                    break;

                default:
                    printf("Evento no reconocido: %d\n", evento.tipo);
                    err = ESP_ERR_INVALID_ARG;
                    break;
            }

            if (err == ESP_OK) {
                err = nvs_commit(handle);  // Importante: guarda los cambios en la Flash
                if (err != ESP_OK) {
                    printf("Error al hacer commit en NVS: %s\n", esp_err_to_name(err));
                }
            } else {
                printf("Error al guardar en NVS: %s\n", esp_err_to_name(err));
            }
			nvs_commit(handle);
            nvs_close(handle);  // Cierro el handle
        }
    }
}

//sirve ante posibles fallos de alimentacion o reinicios para guardar las consignas
//para realizar el automatismo (por ejemplo valores como la lectura de los sensores no 
//los guardo aqui ya que no son tan criticos)))
void inicializar_nvs_por_defecto() {
    nvs_handle_t handle;
    esp_err_t err;
//particion NSV definida en el archivo partitions.csv
    err = nvs_open("storage", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Error al abrir NVS: %s", esp_err_to_name(err));
        return;
    }

    int32_t  valor ;
	float valor_float;
    // Verifica y establece valor por defecto si no existe
    if (nvs_get_i32(handle, "DPVMinS1", &valor) == ESP_ERR_NVS_NOT_FOUND) {
        nvs_set_i32(handle, "DPVMinS1", 10);  // 1.0 kPa
        valor = 10;
    }
	ESP_LOGW("DEBUG", "valorvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv = %ld", (long)valor);
	//Hago esto para que se muestr por la secreen física desde el arranque del sistema
	// y que concuerde las consginas que se ven por pantalla con las que utiliza el programa 
	//por dentro
	// Ahora convierto a float dividiendo
	valor_float = (float)valor / 10.0f;
	snprintf(DPVminS1.valor, SENSOR_STR_SIZE, "%.1f", valor_float);
    DPVminS1.valor[SENSOR_STR_SIZE - 1] = '\0';
    DPVminS1.actualizado = true;
    
    if (nvs_get_i32(handle, "DPVMaxS1", &valor) == ESP_ERR_NVS_NOT_FOUND) {
        nvs_set_i32(handle, "DPVMaxS1", 15);  // 1.5 kPa
        valor = 15;
    }
    
    valor_float = (float)valor / 10.0f;
	snprintf(DPVmaxS1.valor, SENSOR_STR_SIZE, "%.1f", valor_float);
    DPVmaxS1.valor[SENSOR_STR_SIZE - 1] = '\0';
    DPVmaxS1.actualizado = true;

    if (nvs_get_i32(handle, "DPVMinS2", &valor) == ESP_ERR_NVS_NOT_FOUND) {
        nvs_set_i32(handle, "DPVMinS2", 10);
        valor = 10;
    }
	
	valor_float = (float)valor / 10.0f;
	snprintf(DPVminS2.valor, SENSOR_STR_SIZE, "%.1f", valor_float);
    DPVminS2.valor[SENSOR_STR_SIZE - 1] = '\0';
    DPVminS2.actualizado = true;
    
    if (nvs_get_i32(handle, "DPVMaxS2", &valor) == ESP_ERR_NVS_NOT_FOUND) {
        nvs_set_i32(handle, "DPVMaxS2", 15);
        valor = 15;
    }
	
	valor_float = (float)valor / 10.0f;
	snprintf(DPVmaxS2.valor, SENSOR_STR_SIZE, "%.1f", valor_float);
    DPVmaxS2.valor[SENSOR_STR_SIZE - 1] = '\0';
    DPVmaxS2.actualizado = true;
    
    if (nvs_get_i32(handle, "AutoOFF", &valor) == ESP_ERR_NVS_NOT_FOUND) {
        nvs_set_i32(handle, "AutoOFF", 0);  // Automatismo activo por defecto
        valor = 0;
    }
	if(valor == 0)
    {
		estado_automatismo.estadoReles = false;
		estado_automatismo.actualizado = true;
	}
	else if(valor == 1)
	{
		estado_automatismo.estadoReles = true;
		estado_automatismo.actualizado = true;
	}

    nvs_commit(handle);
    nvs_close(handle);
}

esp_err_t comprobar_direcciones(uint8_t *buf)
{
	if(buf[0] == MasterNode && (buf[1] == Node1 || buf[1] == Node2))
		return ESP_OK;
		
	return ESP_FAIL;	
}

static portTASK_FUNCTION(RELAYS, pvParameters)
{
    int received_bytes = 0;//del nodo1 vendrian 6 y del nodo2 14
    uint8_t *received_msg;
    uint8_t node_id;
    QueueSetMemberHandle_t activatedHandle;
    while (1)
    {
        // Esperar un mensaje en la cola o que el semáforo sea liberado
        activatedHandle = xQueueSelectFromSet(myQueueSet, portMAX_DELAY);
        // if (eventosRELES & ACTUALIZAR_RELES)
        if (activatedHandle == lora_queue)
        {
            // leo mensaje
            ESP_LOGI(TAG, "Mensajes antes de recibir: %d", uxQueueMessagesWaiting(lora_queue));
            if (xQueueReceive(lora_queue, &received_msg, portMAX_DELAY) == pdPASS)
            {
                ESP_LOGI(TAG, "Cola recibida:");
            }
            
                // Detectar el nodo emisor por el segundo byte
             node_id = received_msg[1];
             received_bytes = 0;
             
            //ESP_LOGI(TAG, "Mensajes despues de recibir: %d", uxQueueMessagesWaiting(lora_queue));
            if (node_id == Node1) {
                    received_bytes = 6;   // Nodo1 (ambiente)
            } else if (node_id == Node2) {
                    received_bytes = 14;  // Nodo2 (suelo + ambiente)
             } 
        
             
            for (int i = 0; i < received_bytes; i++)
            {
                ESP_LOGI(TAG, "Byte %d: 0x%02X", i, received_msg[i]); // Mostrar en formato hexadecimal
            }
			
			// Crear una copia local del mensaje antes de procesarlo
             //uint8_t local_msg[received_bytes];
             uint8_t local_msg[14];
             memcpy(local_msg, received_msg, received_bytes);
                    
            procesar_buffer(local_msg);
            
            // Solo liberar el mensaje si fue asignado dinámicamente
             if (received_msg != NULL)
              {
				  
                     free(received_msg);
                      received_msg = NULL;
              }
            //free(received_msg); // Libera la memoria después de procesar el mensaje
            // mando ack-> Consta de direcciones y variable preservada?
            // ojo solo puedo mandar ack si la otra tarea está inactiva?->Pensar
            //esto ya lo hago dentro de procesar_buffer
           // configure_send_ack(); // inecitablemente si haymuchas peticiones se perderá alguna al tener que enviar el ack_correspondiente
        }
        else if (activatedHandle == security_semaphore_Sector1)
        {
            // Apago relés por si estaban encendidos
            //gpio_set_level(RELAY_1, portMAX_DELAY); //
            xTimerStop(timerSector1, portMAX_DELAY);
			gpio_set_level(RELAY_1, 0);  // apagar relé,por si justo en el momento de parar el timer estaba encendido!
			if(xSemaphoreTake(xReleMutex, portMAX_DELAY))
			{
				estado_Rele_Sector1 = 0;
				if(!estado_Rele_Sector2)
				{
					gpio_set_level(RELAY_3, 0); // se apaga el general
				}
				xSemaphoreGive(xReleMutex);
			}

            esp_mqtt_client_publish(client, "mariomp/f/activacionRele1", "OFF", 0, 1, 0);
            ESP_LOGI(TAG, "Rele Sector1 desactivados:");
        }
        else // (activatedHandle == security_semaphore_Sector1)
        {
            xTimerStop(timerSector2, portMAX_DELAY);
			gpio_set_level(RELAY_2, 0);  // apagar relé,por si justo en el momento de parar el timer estaba encendido!(blink)
			if(xSemaphoreTake(xReleMutex, portMAX_DELAY))
			{
				estado_Rele_Sector2 = 0;
				if(!estado_Rele_Sector1)
				{
					gpio_set_level(RELAY_3, 0); // se apaga el general
				}
				xSemaphoreGive(xReleMutex);
			}
            esp_mqtt_client_publish(client, "mariomp/f/activacionRele2", "OFF", 0, 1, 0);
            ESP_LOGI(TAG, "Rele Sector2 desactivados:");
		}
    }
}


void procesar_buffer(uint8_t *buf)
{
    float temperature=0, humidity=0;
    float temp_soil = 0, hum_soil = 0, cond_soil = 0, ph_soil = 0;
    uint8_t NodeACK;

    
    for (int i = 0; i < sizeof(buf); i++) {
    	printf("%02X ", buf[i]);  // Imprime en hexadecimal
	}
	printf("\n");

    // Compruebo que el mensaje es para este módulo
    if (comprobar_direcciones(buf) != ESP_OK)
    {
        ESP_LOGI(TAG, "Direccion incorrecta:");// Pensar que pasaría si no?!!!!!
        // Volver a Poner en modo recepcion!
    	lora_receive();
        return;
    }
 
        ESP_LOGI(TAG, "Direccion correcta:"); 
           

    	// Convierto los 4 bytes de temperatura y humedad a valores reales
		if(buf[1] == Node1)
    	{
			    
    	//Paro el posible timer_security que puede que este activo:
     	if (timer_security_Sector1 != NULL) 
     	{
        	if (xTimerIsTimerActive(timer_security_Sector1) == pdTRUE) 
        	{
            	xTimerStop(timer_security_Sector1, pdMS_TO_TICKS(500));
        	}
     	}
			NodeACK = Node1;
			convert4bytes_to_data(buf, &temperature, &humidity);
			printf("NODO1\n");
    		ESP_LOGI(TAG, "Temperatura: %.2f °C", temperature);
    		ESP_LOGI(TAG, "Humedad: %.2f %%", humidity);
		}
		else //viene del Nodo2 (con sensor de suelo)
		{
				//Paro el posible timer_security que puede que este activo:
     	if (timer_security_Sector2 != NULL) 
     	{
        	if (xTimerIsTimerActive(timer_security_Sector2) == pdTRUE) 
        	{
            	xTimerStop(timer_security_Sector2, pdMS_TO_TICKS(500));
        	}
     	}
			NodeACK = Node2;
			convert12bytes_to_data(buf, &temperature, &humidity, &temp_soil,&hum_soil,&cond_soil,&ph_soil);
			printf("NODO2\n");
			ESP_LOGI(TAG, "Temperatura ambiente: %.2f °C", temperature);
			ESP_LOGI(TAG, "Humedad ambiente: %.2f %%", humidity);
			ESP_LOGI(TAG, "Temperatura suelo: %.2f °C", temp_soil);
			ESP_LOGI(TAG, "Humedad suelo: %.2f %%", hum_soil);
			ESP_LOGI(TAG, "Conductividad suelo: %.2f uS/cm", cond_soil);
			ESP_LOGI(TAG, "pH suelo: %.2f", ph_soil);
		}
		
    	
    //////////////////aqui iria DPV
    	// compruebo condiciones temperatura y humedad, actualizo variable preservada
    	//if (humidity < 60.0 || (humidity < 70.0 && temperature > 30.0))//Aqui iría el DPV
    	// if(humidity > 60.0)
    	uint8_t resultDPV;
    	resultDPV =  calcular_dpv( temperature,  humidity, NodeACK);
    	
		if(automatismo_desactivado() == 0)
    	{
		printf(" Automatismo activo\n");
    	if (resultDPV == 1 || resultDPV == 0) 
    	{
    	uint8_t estado = resultDPV;                // 1 = encender, 0 = apagar
    	//uint8_t nivel_gpio = (estado == 1) ? 0 : 1;

		if (NodeACK == Node1) {
			if (xSemaphoreTake(xReleMutex, portMAX_DELAY))
			{
        		estado_Rele_Sector1 = estado;
        		//gpio_set_level(RELAY_1, nivel_gpio);
        		gpio_set_level(RELAY_3, estado_Rele_Sector1);
        		ESP_LOGI(TAG, "Nebulizacion Sector1 %s", estado ? "activada" : "desactivada");
				if(estado_Rele_Sector1 == 1)
        		{
					xTimerStart(timerSector1, portMAX_DELAY);//empieza la nebulizacion
				}
				else 
				{
					xTimerStop(timerSector1, portMAX_DELAY);
					gpio_set_level(RELAY_1, 0);  // apagar relé,por si justo en el momento de parar el timer estaba encendido!
					if(!estado_Rele_Sector2)
					{
						gpio_set_level(RELAY_3, 0); //apago el general
					}
				}
				xSemaphoreGive(xReleMutex);
        	}
        	
    		} else {
				
				if (xSemaphoreTake(xReleMutex, portMAX_DELAY)){
        		estado_Rele_Sector2 = estado;
        		//gpio_set_level(RELAY_2, nivel_gpio);
        		gpio_set_level(RELAY_3, estado_Rele_Sector2);
        		ESP_LOGI(TAG, "Nebulizacion Sector2 %s", estado ? "activada" : "desactivada");
				if(estado_Rele_Sector2 == 1)
        		{
					xTimerStart(timerSector2, portMAX_DELAY);//empieza la nebulizacion
				}
				else 
				{
					xTimerStop(timerSector2, portMAX_DELAY);
					gpio_set_level(RELAY_2, 0);  // apagar relé,por si justo en el momento de parar el timer estaba encendido!
					if(!estado_Rele_Sector1)
					{
						gpio_set_level(RELAY_3, 0); //apago el general
					}
				}
					xSemaphoreGive(xReleMutex);
        		}
    		}
		}
		}
		else {
		printf(" Automatismo Desactivado\n");
		}
    	
	//////////////////////////////////
		configure_send_ack(NodeACK);/////////envio ACK

    	char payload[10];
    	if(buf[1] == Node1 )
    	{
    		sprintf(payload, "%.2f", temperature); // Convertir float a string
    		esp_mqtt_client_publish(client, "mariomp/f/Temperatura_Ambiente_Sector1", payload, 0, 1, 0);

		strncpy(temp_S1.valor, payload, SENSOR_STR_SIZE);
    	temp_S1.valor[SENSOR_STR_SIZE - 1] = '\0';
    	temp_S1.actualizado = true;

    		//update_temp_label_async(ui_VALORtempS1);
    		sprintf(payload, "%.2f", humidity); // Convertir float a string
    		esp_mqtt_client_publish(client, "mariomp/f/Humedad_Ambiente_Sector1", payload, 0, 1, 0);
			strncpy(hum_S1.valor, payload, SENSOR_STR_SIZE);
    		hum_S1.valor[SENSOR_STR_SIZE - 1] = '\0';
    		hum_S1.actualizado = true;
    		
    		if (estado_Rele_Sector1 == 1 && (automatismo_desactivado() == 0))
    		{
				Rele1.estadoReles = true;
				Rele1.actualizado = true;
        		esp_mqtt_client_publish(client, "mariomp/f/activacionRele1", "ON", 0, 1, 0);
    		}
    		else if (estado_Rele_Sector1 == 0 && (automatismo_desactivado() == 0))
    		{
				Rele1.estadoReles = false;
				Rele1.actualizado = true;
        		esp_mqtt_client_publish(client, "mariomp/f/activacionRele1", "OFF", 0, 1, 0);
    		}
    	}
    	else //Se trata de una informacion procedende del nodo 2
    	{
			sprintf(payload, "%.2f", temperature); // Convertir float a string
    		esp_mqtt_client_publish(client, "mariomp/f/Temperatura_Ambiente_Sector2", payload, 0, 1, 0);
			strncpy(temp_S2.valor, payload, SENSOR_STR_SIZE);
    		temp_S2.valor[SENSOR_STR_SIZE - 1] = '\0';
    		temp_S2.actualizado = true;
    		
    		sprintf(payload, "%.2f", humidity); // Convertir float a string
    		esp_mqtt_client_publish(client, "mariomp/f/Humedad_Ambiente_Sector2", payload, 0, 1, 0);
    		strncpy(hum_S2.valor, payload, SENSOR_STR_SIZE);
    		hum_S2.valor[SENSOR_STR_SIZE - 1] = '\0';
    		hum_S2.actualizado = true;
    		
    		sprintf(payload, "%.2f", temp_soil); // Convertir float a string
    		esp_mqtt_client_publish(client, "mariomp/f/Temperatura_Suelo_Sector2", payload, 0, 1, 0);
			strncpy(temp_suelo.valor, payload, SENSOR_STR_SIZE);
    		temp_suelo.valor[SENSOR_STR_SIZE - 1] = '\0';
    		temp_suelo.actualizado = true;
    		
    		sprintf(payload, "%.2f", hum_soil); // Convertir float a string
    		esp_mqtt_client_publish(client, "mariomp/f/Humedad_Suelo_Sector2", payload, 0, 1, 0);
    		strncpy(hum_suelo.valor, payload, SENSOR_STR_SIZE);
    		hum_suelo.valor[SENSOR_STR_SIZE - 1] = '\0';
    		hum_suelo.actualizado = true;
    		
    		sprintf(payload, "%.2f", cond_soil); // Convertir float a string
    		esp_mqtt_client_publish(client, "mariomp/f/Conductividad_Suelo_Sector2", payload, 0, 1, 0);
    		strncpy(cond_suelo.valor, payload, SENSOR_STR_SIZE);
    		cond_suelo.valor[SENSOR_STR_SIZE - 1] = '\0';
    		cond_suelo.actualizado = true;
    		
    		sprintf(payload, "%.2f", ph_soil); // Convertir float a string
    		esp_mqtt_client_publish(client, "mariomp/f/PH_Suelo_Sector2", payload, 0, 1, 0);
    		strncpy(ph_suelo.valor, payload, SENSOR_STR_SIZE);
    		ph_suelo.valor[SENSOR_STR_SIZE - 1] = '\0';
    		ph_suelo.actualizado = true;

		if ((estado_Rele_Sector2 == 1 && (automatismo_desactivado() == 0))) // y si no está desactivado el modo Desactivar Automatismo
    		{
				Rele2.estadoReles = true;
				Rele2.actualizado = true;
        		esp_mqtt_client_publish(client, "mariomp/f/activacionRele2", "ON", 0, 1, 0);
    		}
		else if ((estado_Rele_Sector2 == 0 && (automatismo_desactivado() == 0)))
    		{
				Rele2.estadoReles = false;
				Rele2.actualizado = true;
        		esp_mqtt_client_publish(client, "mariomp/f/activacionRele2", "OFF", 0, 1, 0);
    		}
		}
    
   
   // free(buf);// Libera la memoria después de procesar el mensaje
}

//  ISR (Manejador de Interrupción) - Se ejecuta cuando el GPIO cambia

static void IRAM_ATTR gpio_isr_handler(void *arg)
{

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Libera el semáforo para que la tarea procese el mensaje
    xSemaphoreGiveFromISR(lora_semaphore, &xHigherPriorityTaskWoken);
    // Realizar un cambio de contexto si es necesario
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken); //

}

void setup_interrupt()
{ // Pensar si está función es de verdad nececsaria
  //  Si ya está instalado, no instalo nuevamente el servicio de interrupciones
    static bool is_isr_service_installed = false;

    if (!is_isr_service_installed)
    {
        gpio_install_isr_service(0); // Agregar flags adecuados
        is_isr_service_installed = true;
    }

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_POSEDGE; // Se activa en nivel bajo?
    io_conf.pin_bit_mask = (1ULL << LORA_DIO0_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);

    gpio_isr_handler_add(LORA_DIO0_PIN, gpio_isr_handler, (void *)LORA_DIO0_PIN);

    ESP_LOGI(TAG, "Interrupcion configurada en GPIO %d", LORA_DIO0_PIN);
}

static portTASK_FUNCTION(RECEIVER, pvParameters)
{
    uint8_t message[14]; //TAMAÑO MAXIMO
    int received_bytes;

    while (1)
    {
        if (xSemaphoreTake(lora_semaphore, portMAX_DELAY) == pdTRUE)
        {
            received_bytes = lora_receive_packet(message, sizeof(message));
            printf("Received: %d bytes\n", received_bytes);
            
              // ✅ Verificar si realmente se recibieron datos antes de continuar
            if (received_bytes <= 0)
            {
                ESP_LOGW(TAG, "No se recibieron datos válidos.");
                continue; // Saltar a la siguiente iteración del bucle
            }
            // tambien podria usar malloc?
            //uint8_t *msg_ptr = malloc(sizeof(received_bytes));//Antes estaba esto
            uint8_t *msg_ptr = malloc(received_bytes);
            if (msg_ptr == NULL)
            {
                ESP_LOGE(TAG, "Error: No se pudo asignar memoria para el mensaje.");
                continue;
            }
            
                memcpy(msg_ptr, message, received_bytes);//antes estaba sizeof(received_bytes)
                // Enviar el mensaje a la cola(envio un puntero, no los directamente)
                if (xQueueSend(lora_queue, &msg_ptr, portMAX_DELAY) != pdPASS)
                {
                    ESP_LOGW(TAG, "Cola llena, mensaje descartado");
                    free(msg_ptr); // Liberar memoria si no se pudo encolar
                }
                else
                {
                    ESP_LOGI(TAG, "Mensaje encolado ");
                }
            
        }
    }
}

void configure_send_ack(uint8_t Node)
{

    // Crear un buffer ENVIAR el ack
    uint8_t buf[3];
    // Modo explicito es el que está configurado por defecto

    lora_enable_crc(); // Habilitar CRC para integridad de datos
    // Ojo en el receptor debo revisar si hay CRC

    // La tasa de corrección de errores por defecto es 4/5 creo (asi que no modifico coding rate)

    // Llenar el buffer con los valores de temperatura y humedad
    buf[0] = Node;      // direccion del modulo que recibe
    buf[1] = MasterNode; // direccion del módulo que envía
    if(Node == Node1)
    {
    	buf[2] = estado_Rele_Sector1;
    	if (estado_Rele_Sector1 == 1)
    	{
        	xTimerStart(timer_security_Sector1, pdMS_TO_TICKS(500));
    	}
    }
    else 
    {
		buf[2] = estado_Rele_Sector2;
    	if (estado_Rele_Sector2 == 1)
    	{
        	xTimerStart(timer_security_Sector2, pdMS_TO_TICKS(500));
    	}
	}
    // esta función escribe los datos en la fifo, pero además establece el RegPayloadLength->
    //->creo que se incluye automaticamente en el header
    lora_send_packet(buf, sizeof(buf));

    // Volver a Poner en modo recepcion
    lora_receive();

    // Mensaje de confirmación en el puerto serie
    ESP_LOGI(TAG, "Trama LoRa enviada con éxito.");
}




//termino: transpiracion del cultivo!!
// Función para calcular el VPD (Déficit de Presión de Vapor)
uint8_t calcular_dpv(float temperatura, float humedad_relativa, uint8_t sector)
{
    // Constante de la ecuación de presión de saturación de vapor (kPa)
    const float A = 0.6108;
    const float B = 17.27;
    const float C = 237.3;
    char payload[10] = {0};

    // Calcular la presión de vapor de saturación (es) en kPa
    float es = A * exp((B * temperatura) / (C + temperatura));

    // Calcular la presión de vapor real (ea) en kPa
    float ea = es * (humedad_relativa / 100.0);

    // Calcular el VPD (kPa)
    float dpv = es - ea;
	snprintf(payload, sizeof(payload), "%.2f", dpv);
	//poner dpv por pantalla:
	
	 // Leer valores desde NVS
    nvs_handle_t handle;
    esp_err_t err;
    int dpv_min = 0, dpv_max = 0;

    err = nvs_open("storage", NVS_READONLY, &handle);
    if (err != ESP_OK) {
        printf(" Error abriendo NVS: %s\n", esp_err_to_name(err));
        return -1;
    }

    if (sector == Node1) {
        nvs_get_i32(handle, "DPVMinS1", &dpv_min);
        nvs_get_i32(handle, "DPVMaxS1", &dpv_max);
    } else if (sector == Node2) {
        nvs_get_i32(handle, "DPVMinS2", &dpv_min);
        nvs_get_i32(handle, "DPVMaxS2", &dpv_max);
    }
	
	nvs_commit(handle); //Guarda en la flash
    nvs_close(handle);

    // Convertir valores guardados en multiplo de 10 a su valor real
    float dpv_min_f = dpv_min / 10.0f;
    float dpv_max_f = dpv_max / 10.0f;
    
	if(sector == Node1)
	{ 
		strncpy(DPVS1.valor, payload, SENSOR_STR_SIZE);
    	DPVS1.valor[SENSOR_STR_SIZE - 1] = '\0';
    	DPVS1.actualizado = true;

	 	if (dpv >= dpv_max_f) {
        	return 1;  // Hay que nebulizar
    	} 	else if (dpv <= dpv_min_f) {
        	return 0;  // No nebulizar
    	} else {
        	return -1; // Zona neutra (sin cambio)
    	}
    }
	else //Node2
    {

		strncpy(DPVS2.valor, payload, SENSOR_STR_SIZE);
    	DPVS2.valor[SENSOR_STR_SIZE - 1] = '\0';
    	DPVS2.actualizado = true;

		if (dpv >= dpv_max_f) {
        	return 1;  // Hay que nebulizar
    	} 	else if (dpv <= dpv_min_f) {
        	return 0;  // No nebulizar
    	} else {
        	return -1; // Zona neutra (sin cambio)
    	}
	}
    
   // return vpd;  // Retornar el resultado en kPa
}

// Devuelve 1 si el automatismo está desactivado, 0 si está activo, -1 si hay error
int automatismo_desactivado(void) {
    nvs_handle_t handle;
    esp_err_t err;
    int estado = 0;

    err = nvs_open("storage", NVS_READONLY, &handle);
    if (err != ESP_OK) {
        printf("Error abriendo NVS: %s\n", esp_err_to_name(err));
        return -1;
    }

    err = nvs_get_i32(handle, "AutoOFF", &estado);
    nvs_close(handle);

    if (err == ESP_OK) {
        return estado;  // 1 = desactivado, 0 = activado
    } else {
        printf("Error leyendo AutoOFF: %s\n", esp_err_to_name(err));
        return -1;
    }
}


typedef struct {
    lv_obj_t *label;
    char texto[10];
} ui_update_param_t;

static void label_update_cb(void *param) {
    ui_update_param_t *p = (ui_update_param_t *)param;
    if (p && lv_obj_is_valid(p->label)) {
        lv_label_set_text(p->label, p->texto);
    }
    free(p);
}

void actualizar_label_async(lv_obj_t *label, const char *texto) {
    if (!label || !texto) return;

    ui_update_param_t *param = malloc(sizeof(ui_update_param_t));
    if (!param) return;

    param->label = label;
    strncpy(param->texto, texto, sizeof(param->texto) - 1);
    param->texto[sizeof(param->texto) - 1] = '\0';

    lv_async_call(label_update_cb, param);
}

static void safe_update_labels(void *param) {
    // Solo se ejecuta dentro del contexto seguro de LVGL
    if (ui_VALORtempS1) {
        lv_label_set_text(ui_VALORtempS1, SectorS1Amb.temp_amb);
    }
}

