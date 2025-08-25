
// Librerías estándar
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
//  ESP-IDF core
#include "esp_err.h"
#include "esp_log.h"
#include "esp_sleep.h"
//  FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/projdefs.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
//  Drivers
#include "driver/gpio.h"
#include "hal/gpio_types.h"
//  Periféricos y sensores
#include "LoRa.h"
#include "sht40.h"
#include "THCPH-S.h"

//  Etiqueta para logging
static const char *TAG = "LoRa_App";

//  Definiciones y constantes
#define COMPROBAR_ACK  0x01     // Bit flag: esperar ACK
#define REEQUERIR_ACK  0x02     // Bit flag: reenviar si no hay ACK

#define uS_TO_S_FACTOR 1000000  // Conversión de segundos a microsegundos
#define TIME_TO_SLEEP  10       // Tiempo de sueño en segundos

//  Flags, tareas y temporizadores
static TimerHandle_t xTimerACK;
static TaskHandle_t manejador_tarea_ACK = NULL;
static EventGroupHandle_t FlagsEventos;

static int interval_ack = 10000;      // Intervalo entre intentos de ACK (ms)
static int timerACK_ID = 1;           // ID para el temporizador

//  Identificadores de nodos LoRa
uint8_t MasterNode = 0xFF;  // Nodo maestro (ej. para controlar nebulización)
uint8_t Node1 = 0xAA;       // Nodo esclavo

//  Datos de sensor ambiente
uint16_t raw_temperature;  // Considera usar 'volatile' si se accede desde ISR
uint16_t raw_humidity;
//Datos de sensor des uelo
uint16_t raw_hum;
uint16_t raw_temp;
uint16_t raw_cond; 
uint16_t raw_ph;

//  Funciones
void configure_send_trama_lora(uint16_t raw_temp_amb, uint16_t raw_hum_amb,
                                uint16_t raw_temp_soil, uint16_t raw_hum_soil,
                                uint16_t raw_conductivity, uint16_t raw_ph);
esp_err_t static set_timer_ACK(void);
esp_err_t init_ISR_LoRa(void);
esp_err_t comprobar_ACK(uint8_t *buffer);

//  Task function
static portTASK_FUNCTION(GESTIONAR_ACK, pvParameters);
void uart_event_task(void *pvParameter);

    
void app_main(void)
{
    // Inicialización de LoRa
    ESP_LOGI(TAG, "Inicializando LoRa...");
    if (!lora_init()) {
        ESP_LOGE(TAG, "Error al inicializar LoRa");
        return;
    }
    
    // Configuración básica
    ESP_LOGI(TAG, "Configurando LoRa...");
    lora_set_frequency(868E6);       // Configurar frecuencia ( 868 MHz)
    ESP_LOGI(TAG, "Frecuencia configurada a 868 MHz");
    
    lora_set_tx_power(10);           // Configurar potencia de transmisión
    ESP_LOGI(TAG, "Potencia de transmision configurada a 10 dBm");
    

    ESP_LOGI(TAG, "CRC habilitado para asegurar la integridad de los datos");
	
    // Configura GPIO48 como salida
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_NUM_48), // Bitmask para GPIO4
        .mode = GPIO_MODE_OUTPUT,             // Modo salida
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

	// Configura GPIO47 como salida
    gpio_config_t io_conf2 = {
        .pin_bit_mask = (1ULL << GPIO_NUM_47), // Bitmask para GPIO4
        .mode = GPIO_MODE_OUTPUT,             // Modo salida
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf2);
    // Pone GPIO48 para alimentar sensor haciendo conmutar mosfet
    gpio_set_level(GPIO_NUM_47, 1);
    gpio_set_level(GPIO_NUM_48, 1);
	vTaskDelay(pdMS_TO_TICKS(500));
///////////////////////////////////////////////	
    i2c_master_init_bus();
    i2c_master_init_handle( SHT40_I2C_ADDRESS);
    vTaskDelay(pdMS_TO_TICKS(200));  // Espera para que el sensor esté listo antes de leer
    
    //uart_flush(UART_PORT);  // Limpia posibles residuos en el buffer UART

    init_uart();
    gpio_reset_pin(RTS_PIN);
    gpio_set_direction(RTS_PIN,GPIO_MODE_OUTPUT);

    
    init_ISR_LoRa();
    lora_receive();
    
       FlagsEventos = xEventGroupCreate();
  		if (FlagsEventos == NULL)
  		{
    		while (1);
    	} 
 	//Tarea que gestiona el ACK
  	xTaskCreate(GESTIONAR_ACK, "SendMsgTask", 4096, NULL, 1, &manejador_tarea_ACK);
   
	xTaskCreate(uart_event_task, "uart_event_task", 2048 * 4, NULL, 5, NULL);
    
    gpio_reset_pin(RTS_PIN);
    gpio_set_direction(RTS_PIN,GPIO_MODE_OUTPUT);
    
    
        
    esp_err_t err1 = read_sht40_raw(&raw_temperature, &raw_humidity);
    
    if (err1 == ESP_OK) {
        ESP_LOGI(TAG, "Temperatura: %.2hu °C", raw_temperature);
        ESP_LOGI(TAG, "Humedad: %.2u %%", raw_humidity);
    } else {
        ESP_LOGE(TAG, "Error al leer el sensor SHT40_");
    }
    
	convert_raw_sensor_data(raw_temperature,raw_humidity);
	

	   vTaskDelay(pdMS_TO_TICKS(700));//dar tiempo a la uart?
       send_modbus_request();			

}





void configure_send_trama_lora(uint16_t raw_temp_amb, uint16_t raw_hum_amb,
                                uint16_t raw_temp_soil, uint16_t raw_hum_soil,
                                uint16_t raw_conductivity, uint16_t raw_ph) {
    
    uint8_t buf[14];

    lora_enable_crc();  // Verificación de integridad

    // Direcciones
    buf[0] = MasterNode;
    buf[1] = Node1;

    // Temp y Hum ambiente
    buf[2] = (raw_temp_amb >> 8) & 0xFF;
    buf[3] = raw_temp_amb & 0xFF;
    buf[4] = (raw_hum_amb >> 8) & 0xFF;
    buf[5] = raw_hum_amb & 0xFF;

    // Temp y Hum suelo
    buf[6] = (raw_temp_soil >> 8) & 0xFF;
    buf[7] = raw_temp_soil & 0xFF;
    buf[8] = (raw_hum_soil >> 8) & 0xFF;
    buf[9] = raw_hum_soil & 0xFF;

    // Conductividad
    buf[10] = (raw_conductivity >> 8) & 0xFF;
    buf[11] = raw_conductivity & 0xFF;

    // pH
    buf[12] = (raw_ph >> 8) & 0xFF;
    buf[13] = raw_ph & 0xFF;

    // Enviar
    for (int i = 0; i < sizeof(buf); i++) {
    printf("%02X ", buf[i]);  // Imprime en hexadecimal
	}

    lora_send_packet(buf, sizeof(buf));
    lora_receive(); // Esperar ACK

    if (xTimerACK == NULL) {
        set_timer_ACK();
        ESP_LOGI(TAG, "Timer activado (reenvío ACK → 10s).");
    }

    ESP_LOGI(TAG, "Trama LoRa enviada con éxito.");
}

void uart_event_task(void *pvParameter) {
    uart_event_t event;
    uint8_t response[13];  // Buffer para la respuesta del sensor (13 bytes)
    size_t len = 0;

    while (1) {
        if (xQueueReceive(uart_event_queue, (void*)&event, portMAX_DELAY) == pdTRUE) {
            switch (event.type) {
                case UART_DATA:
                    // Leer bytes del sensor (esperamos exactamente 13 bytes)
                    len = uart_read_bytes(UART_PORT, response, sizeof(response), pdMS_TO_TICKS(100));
                    if (len == 13) {
                        ESP_LOGI(TAG, "Respuesta recibida:");
                        for (int i = 0; i < len; i++) {
                            ESP_LOGI(TAG, "0x%02X", response[i]);
                        }

                        // Verificar CRC
                        uint16_t received_crc = (response[len - 1] << 8) | response[len - 2];
                        uint16_t calculated_crc = modbus_crc16(response, len - 2);
                        
                        if (received_crc == calculated_crc) {
                            // Extraer valores de los registros
                            uint16_t humidity_raw = (response[3] << 8) | response[4];
                            int16_t temperature_raw = (response[5] << 8) | response[6]; // Manejo de negativos
                            uint16_t conductivity_raw = (response[7] << 8) | response[8];
                            uint16_t ph_raw = (response[9] << 8) | response[10];

                            // Convertir a valores reales->solo para depuracion
                            float humidity = humidity_raw / 10.0;
                            float temperature = temperature_raw / 10.0;
                            float conductivity = conductivity_raw;
                            float ph = ph_raw / 10.0;

                            // Imprimir valores procesados
                            ESP_LOGI(TAG, "Humedad: %.1f %%RH", humidity);
                            ESP_LOGI(TAG, "Temperatura: %.1f °C", temperature);
                            ESP_LOGI(TAG, "Conductividad: %.0f uS/cm", conductivity);
                            ESP_LOGI(TAG, "pH: %.1f", ph);
                          
                            configure_send_trama_lora(raw_temperature, raw_humidity,temperature_raw  
      						,humidity_raw,conductivity_raw,ph_raw);
                        } else {
                            ESP_LOGE(TAG, "Error de CRC: Recibido=0x%04X, Esperado=0x%04X", received_crc, calculated_crc);
                            //Se reinicia
                            esp_restart();

                            
                        }
                    } else {
                        ESP_LOGE(TAG, "No se recibió la respuesta esperada del sensor (len=%d)", len);
                    }

                    memset(response, 0, sizeof(response)); // Limpiar el buffer
                    break;

                case UART_FRAME_ERR:
                    ESP_LOGE(TAG, "UART_FRAME_ERR");
                    break;

                default:
                    break;
            }
        }
    }
}

static portTASK_FUNCTION(GESTIONAR_ACK, pvParameters)
{ 

  EventBits_t eventosACK;
  uint8_t buf[10];  // Un buffer con espacio suficiente para recibir los datos
  int received_bytes;
  uint8_t reequerimiento_cont = 0;
 
  while (1)
  {
    eventosACK = xEventGroupWaitBits(FlagsEventos, COMPROBAR_ACK | REEQUERIR_ACK, pdFALSE, pdFALSE, portMAX_DELAY);
    // argumentos de la función:(manejador, máscara de bits,indicar si queremos borrar o no los  flags,bool,que se active cualquiera de los bits de la máscara o con ambos,espera_indefinida)
    // if (xSemaphoreTake(semaforo_comprobarACK, portMAX_DELAY) == pdTRUE)
	ESP_LOGI(TAG, "Evento recibido:" );

    if (eventosACK & COMPROBAR_ACK)
    {
    xEventGroupClearBits(FlagsEventos, COMPROBAR_ACK);
    ESP_LOGI(TAG,"Mensaje recibido.");      
    received_bytes= lora_receive_packet(buf,sizeof(buf));
    printf("Received: %d\n", received_bytes);
    
	if (received_bytes > 0) 
	{
    ESP_LOGI(TAG, "Datos recibidos:");
    	for (int i = 0; i < received_bytes; i++) 
    	{
        	ESP_LOGI(TAG, "Byte %d: 0x%02X", i, buf[i]); // Mostrar en formato hexadecimal
    	}
    }
    //comprobar que las direcciones de recepción y envío sean correctas
    //Comprobar que el ACK es correcto
    if(comprobar_ACK(buf) == ESP_OK)
    {
    	ESP_LOGI(TAG, "ACK correcto:");
    	
    		//desactivo el timer si es disntinto de NULL
		if(xTimerACK !=NULL)
		{
			xTimerStop(xTimerACK,0);//Revisar el segundo argumento. Paro el timer
								//Elimino el timer
			if (xTimerDelete(xTimerACK, pdMS_TO_TICKS(10)) == pdPASS) 
			{
   				ESP_LOGI(TAG, "TimerACK eliminado correctamente.\n");
			} 
			else 
			{
    			ESP_LOGI(TAG, "Error al eliminar el timer.\n");
			}
			//Determino el sleep_Time
			if(buf[2] == 1)
			{
				sleep_time = SHORT_TIME;
			}
			else 
			{
				sleep_time = LONG_TIME;
			}
		}

    esp_err_t err = esp_sleep_enable_timer_wakeup(sleep_time * uS_TO_S_FACTOR);//este tiempo dependerá de la variable preservada!
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Temporizador (deep sleep) configurado correctamente.");
    } else {
        ESP_LOGE(TAG, "Error al configurar el temporizador: %d", err);
    }
    
    // Entrar en Deep-sleep
    esp_deep_sleep_start();
     // Este mensaje nunca se imprimirá porque el ESP se reinicia al despertar
    ESP_LOGW(TAG, "ERROR: El ESP32 no debería llegar aquí");
    }

    else
    {
		ESP_LOGI(TAG,  "ACK incorrecto:");// si el ACK es incorrecto el timer no se elimina y se seguiría enviando el requerimiento
	} 	
    	
	
	}
     //lora_write_reg(REG_IRQ_FLAGS, 0xFF);  // Limpiar todas las banderas de interrupción

    else if (eventosACK & REEQUERIR_ACK)
    {
		if(reequerimiento_cont < 7)
		{
			ESP_LOGI(TAG,  "Reequerimiento ACK:");
      		xEventGroupClearBits(FlagsEventos, REEQUERIR_ACK);
      		//mandar mensaje para intentar reequerir el ACK. Esto tendrá un límite de reequerimientos
     		//por si el otro módulo no responde?-> Gestionar con una variable contadora?
      		configure_send_trama_lora(raw_temperature, raw_humidity,raw_temp  
      						,raw_hum,raw_cond,raw_ph);
      }
      else
      {
		  esp_restart();

	  }
    }
  }
}

esp_err_t comprobar_ACK (uint8_t *buffer)
{
  
    if (buffer[0] == Node1 && 
    buffer[1] == MasterNode )//&& buffer[2] == variablePreservada1)
	{
    	return ESP_OK;
	}

	return ESP_FAIL;
}
void vTimerAckCallback (TimerHandle_t timer)
{
BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  xEventGroupSetBitsFromISR(FlagsEventos, REEQUERIR_ACK, &xHigherPriorityTaskWoken);
  // Realizar un cambio de contexto si es necesario
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

esp_err_t static set_timer_ACK (void)
{
	xTimerACK = xTimerCreate("TimerACK",
							 pdMS_TO_TICKS(interval_ack),
							  pdTRUE, 
							  (void*)timerACK_ID, 
							  vTimerAckCallback);
							  //TimerCallbackFunction_t pxCallbackFunction);
	
	if(xTimerACK == NULL)
	{
		ESP_LOGE(TAG, "Error: No se pudo crear el temporizador");
    	return ESP_FAIL;  // Devuelves un error si no se pudo crear el temporizador
	}
	else 
	{
		if(xTimerStart(xTimerACK, 0)!=pdPASS)
		{
			ESP_LOGE(TAG, "Error: No se pudo iniciar el temporizador");
        	return ESP_FAIL;  // Si el temporizador no se pudo iniciar
		}
    }

	return ESP_OK;						  
							  
}


//ISR (LoRa)

static void IRAM_ATTR lora_on_dio0_rise(void* arg)
{
  BaseType_t higherPriorityTaskWoken = pdFALSE;
	
  xEventGroupSetBitsFromISR(FlagsEventos, COMPROBAR_ACK, &higherPriorityTaskWoken);
  // Todo el procesamiento lora lo hago fuera de la callback ISR

  portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

	esp_err_t init_ISR_LoRa(void)//init ISR
	{

        gpio_config_t io_conf = {
            .intr_type = GPIO_INTR_POSEDGE,  // Interrupción en flanco de subida
            .mode = GPIO_MODE_INPUT,         // Modo entrada
            .pin_bit_mask = (1ULL << LORA_DIO0_PIN), // Máscara del pin
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE
        };
        gpio_config(&io_conf);

        // Agregar el manejador de la interrupción
        gpio_install_isr_service(0);
        gpio_isr_handler_add(LORA_DIO0_PIN, lora_on_dio0_rise, NULL);

    return ESP_OK;	
}


