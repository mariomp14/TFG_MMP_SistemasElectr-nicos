#include "LoRa.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/projdefs.h"
#include "hal/gpio_types.h"
#include "sht40.h"
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
#include <esp_sleep.h>


//#include "esp_timer.h" //precision de microsegundos, no necesaria para mi aplicacion

static TimerHandle_t xTimerACK;
static TaskHandle_t manejador_tarea_ACK = NULL; // manejador de la tarea DEL ADC
static EventGroupHandle_t FlagsEventos;                 // flag de eventos
#define COMPROBAR_ACK 0x01                              //(1<<0) // 0x0001
#define REEQUERIR_ACK 0x02                              //(1<<1)// 0x0002
static int interval_ack = 10000;
static int timerACK_ID = 1;
static const char *TAG = "LoRa_App";
void configure_send_trama_lora(uint16_t raw_temperature, uint16_t raw_humidity);
esp_err_t static set_timer_ACK (void);
esp_err_t init_ISR_LoRa(void);
//static void IRAM_ATTR lora_on_dio0_rise(void* arg);
uint8_t MasterNode = 0xFF;//Nodo que controla la nebulización
uint8_t Node1 = 0xBB;

static portTASK_FUNCTION(GESTIONAR_ACK, pvParameters);
esp_err_t comprobar_ACK (uint8_t *buffer);
uint16_t raw_temperature; 
uint16_t raw_humidity;


#define uS_TO_S_FACTOR 1000000  // Conversión de segundos a microsegundos
#define TIME_TO_SLEEP 10         // Tiempo de sleep en segundos
    
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
    lora_set_frequency(868E6);       // Configurar frecuencia (por ejemplo, 868 MHz)
    ESP_LOGI(TAG, "Frecuencia configurada a 868 MHz");
    
    lora_set_tx_power(10);           // Configurar potencia de transmisión
    ESP_LOGI(TAG, "Potencia de transmision configurada a 10 dBm");
    

    ESP_LOGI(TAG, "CRC habilitado para asegurar la integridad de los datos");
	
    
    i2c_master_init_bus();
    i2c_master_init_handle( SHT40_I2C_ADDRESS);
    
    init_ISR_LoRa();
    lora_receive();//ojo-> esto no hace falta
 	//Tarea que gestiona el ACK
  	xTaskCreate(GESTIONAR_ACK, "SendMsgTask", 4096, NULL, 1, &manejador_tarea_ACK);
   
    FlagsEventos = xEventGroupCreate();
    if (FlagsEventos == NULL)
    {
     while (1)
      ;
    }

    // Configura GPIO48 como salida
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_NUM_48), // Bitmask para GPIO4
        .mode = GPIO_MODE_OUTPUT,             // Modo salida
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Pone GPIO48 para alimentar sensor haciendo conmutar mosfet
    gpio_set_level(GPIO_NUM_48, 1);

	vTaskDelay(pdMS_TO_TICKS(500));
      esp_err_t err1 = read_sht40_raw(&raw_temperature, &raw_humidity);
    
    if (err1 == ESP_OK) {
        ESP_LOGI(TAG, "Temperatura: %.2hu °C", raw_temperature);
        ESP_LOGI(TAG, "Humedad: %.2u %%", raw_humidity);
    } else {
        ESP_LOGE(TAG, "Error al leer el sensor SHT40_");
    }
	convert_raw_sensor_data(raw_temperature,raw_humidity);//no necesario aquí,solo para depuración

	configure_send_trama_lora(raw_temperature, raw_humidity);
		
}


void configure_send_trama_lora(uint16_t raw_temperature, uint16_t raw_humidity) {
	
	// Crear un buffer para almacenar los datos (4 bytes: 2 para temperatura, 2 para humedad)
    uint8_t buf[6];
    //Modo explicito es el que está configurado por defecto
    
 	 lora_enable_crc();               // Habilitar CRC para integridad de datos
   
   //La tasa de corrección de errores por defecto es 4/5 creo (asi que no modifico coding rate)	
  
    // Llenar el buffer con los valores de temperatura y humedad
    buf[0] = MasterNode; //direccion del modulo que recibe
    buf[1] = Node1;      //direccion del módulo que envía
    buf[2] = (raw_temperature >> 8) & 0xFF;  // Byte alto de la temperatura
    buf[3] = raw_temperature & 0xFF;         // Byte bajo de la temperatura
    buf[4] = (raw_humidity >> 8) & 0xFF;   // Byte alto de la humedad
    buf[5] = raw_humidity & 0xFF;          // Byte bajo de la humedad
    
   //esta función escribe los datos en la fifo, pero además establece el RegPayloadLength-> 
   //->se incluye automaticamente en el header
   //Mando los datos raw y los decodifico en el receptor
   lora_send_packet(buf, sizeof(buf));

	//Poner en modo recepcion para esperar el ACK
	lora_receive();
	if(xTimerACK ==NULL)
		{
			set_timer_ACK(); //Activo timer para reequerir el ACK en caso de que no se reciba
			ESP_LOGI(TAG,"Timer activado (reequerimiento ACK-> 10 Seconds).");
		}
	
    // Mensaje de confirmación en el puerto serie
    ESP_LOGI(TAG,"Trama LoRa enviada con exito.");
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

		// Configurar el temporizador para despertar el ESP32
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

    else if (eventosACK & REEQUERIR_ACK)
    {
		if(reequerimiento_cont < 7)
		{
			ESP_LOGI(TAG,  "Reequerimiento ACK:");
      		xEventGroupClearBits(FlagsEventos, REEQUERIR_ACK);
      		//mandar mensaje para intentar reequerir el ACK. Esto tendrá un límite de reequerimientos
     		//por si el otro módulo no responde?-> Gestionar con una variable contadora?
      		configure_send_trama_lora(raw_temperature, raw_humidity);
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
    buffer[1] == MasterNode )
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
