#include <stdio.h>
#include "include/my_mqtt.h"
#include "esp_log.h"
#include "soc/gpio_num.h"
#include "driver/gpio.h"
#include "ui.h"
#include "screen.h"

esp_mqtt_client_handle_t client = NULL;  
static const char *TAG_MQTT = "MQTT_EXAMPLE";

//este semaforo me ayudará a que el main no continue su ejecución hasta que se haya producido la conexión wifi y mqtt!!
 extern SemaphoreHandle_t systemReadySemaphore ;  // Semáforo global

QueueHandle_t mqtt_queue; //  Aquí se define la cola

// Definimos las variables (Solo en un archivo fuente .c)
SemaphoreHandle_t mutexUmbrales = NULL;
float umbral_temperatura = 0;
float umbral_humedad = 0;

SemaphoreHandle_t xReleMutex;//como el estado de los reles se modifica desde distintos contextos. contextod de pantalla, mqtt
//y automatizacion usamos un mutex para evitar condiciones de carrera

TimerHandle_t timerSector1;
TimerHandle_t timerSector2;

volatile uint8_t estado_Rele_Sector1 = 0;//volatile par que no la optimice el compilador!!
volatile uint8_t estado_Rele_Sector2 = 0;
const TickType_t tiempo_on  = pdMS_TO_TICKS(7000);
const TickType_t tiempo_off = pdMS_TO_TICKS(30000);

void parpadeoAsimCallbackRelay1(TimerHandle_t xTimer)
{
    //estado_Rele_Sector1 = !estado_Rele_Sector1; //ojo esto no es del todo correcto, esta variable representa si el proceso de nebulizacion esta o no activo
	//necesito otra variable para realizar el blink, sin modificar esta ya que esto puede afectar a que se cierre o no el RELAY_3
	 static bool estado_blink_sector1 = false;  // Variable local estática para el estado del parpadeo
	
	estado_blink_sector1 = !estado_blink_sector1;
    gpio_set_level(RELAY_1, estado_blink_sector1 ? 0 : 1);  // parpadeo
    ESP_LOGI(TAG_MQTT, "Sector1 %s", estado_blink_sector1 ? "ON" : "OFF");

    // Reiniciar el mismo timer con el siguiente tiempo
    TickType_t next_period = estado_blink_sector1 ? tiempo_off : tiempo_on;
    xTimerChangePeriod(timerSector1, next_period, 0);
}

void parpadeoAsimCallbackRelay2(TimerHandle_t xTimer)
{
    //estado_Rele_Sector2 = !estado_Rele_Sector2;
	static bool estado_blink_sector2 = false;  // Variable local estática para el estado del parpadeo
	
	estado_blink_sector2 = !estado_blink_sector2;
	
    gpio_set_level(RELAY_2, estado_blink_sector2 ? 0 : 1);  // parpadeo
    ESP_LOGI(TAG_MQTT, "Sector1 %s", estado_blink_sector2 ? "ON" : "OFF");

    // Reiniciar el mismo timer con el siguiente tiempo
    TickType_t next_period = estado_blink_sector2 ? tiempo_off : tiempo_on;
    xTimerChangePeriod(timerSector2, next_period, 0);
}

// Función para leer los datos desde NVS
mqtt_config_t leer_mqtt_desde_nvs() {
    mqtt_config_t config = {0};  // Inicializa la estructura 

    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG_MQTT, "Error abriendo NVS");
        return config;  // Devuelve estructura vacía
    }

    size_t url_len = sizeof(config.mqtt_url);
    size_t user_len = sizeof(config.mqtt_user);
    size_t pass_len = sizeof(config.mqtt_pass);

    nvs_get_str(nvs_handle, "mqtt_url", config.mqtt_url, &url_len);
    nvs_get_str(nvs_handle, "mqtt_user", config.mqtt_user, &user_len);
    nvs_get_str(nvs_handle, "mqtt_pass", config.mqtt_pass, &pass_len);

    ESP_LOGI(TAG_MQTT, "MQTT URL: %s", config.mqtt_url);
    ESP_LOGI(TAG_MQTT, "MQTT User: %s", config.mqtt_user);
    ESP_LOGI(TAG_MQTT, "MQTT Pass: %s", config.mqtt_pass);

    nvs_close(nvs_handle);
    return config;  // Devuelve la estructura con los datos
}

void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    client = event->client;

    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG_MQTT, "Conectado al servidor MQTT!");
        esp_mqtt_client_subscribe(client, "mariomp/f/activacionRele1", 1); // Suscribirse a un tópico
        esp_mqtt_client_subscribe(client, "mariomp/f/activacionRele2", 1); // Suscribirse a un tópico
        // Ahora que WiFi y MQTT están listos, liberamos el semáforo global
        xSemaphoreGive(systemReadySemaphore);
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG_MQTT, "Desconectado del servidor MQTT!");
        // Implementar lógica de reconexión?
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG_MQTT, "Suscripcion exitosa, msg_id=%d\n", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI("MQTT", "Mensaje publicado con ID: %d", event->msg_id);
        break;
     case MQTT_EVENT_DATA:
        {
            mqtt_message_t msg;
            strncpy(msg.topic, event->topic, event->topic_len);
            msg.topic[event->topic_len] = '\0';

            strncpy(msg.message, event->data, event->data_len);
            msg.message[event->data_len] = '\0';

            if (xQueueSend(mqtt_queue, &msg, portMAX_DELAY) != pdTRUE)//portMAX_DELAY??
            {
                ESP_LOGE(TAG_MQTT, "Error enviando a la cola");
            }
            break;
        }
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG_MQTT, "Error en MQTT\n");
        break;
    case MQTT_EVENT_BEFORE_CONNECT:
        ESP_LOGI(TAG_MQTT, "En proceso de conexion al broker\n");
        break;
    default:
        ESP_LOGI(TAG_MQTT, "Evento MQTT no manejado: %ld", (long)event_id);

        break;
    }
}

// Configurar y conectar el cliente MQTT
void mqtt_app_start() 
{
	mqtt_config_t mqtt_cfg_data = leer_mqtt_desde_nvs(); // Obtener configuración sin variables globales

	const esp_mqtt_client_config_t mqtt_cfg = 
	{
	.broker.address.uri = mqtt_cfg_data.mqtt_url,
	.credentials.username = mqtt_cfg_data.mqtt_user,
	.credentials.authentication.password = mqtt_cfg_data.mqtt_pass
	};

	esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
	esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
    
	// Esperar hasta que MQTT esté conectado
	ESP_LOGI(TAG_MQTT, "Esperando conexión MQTT...");
}

//TAREA que procesa los mensajes que llegan de MQTT!!
void MQTT(void *pvParameters)
{
    mqtt_message_t received_msg;

    while (1)
    {
        // Esperar mensaje de la cola
        if (xQueueReceive(mqtt_queue, &received_msg, portMAX_DELAY))
        {
            ESP_LOGI(TAG_MQTT, "Procesando mensaje de la cola: %s - %s", received_msg.topic, received_msg.message);

            if (strcmp(received_msg.topic, "mariomp/f/activacionRele1") == 0)
            {
                if (strcmp(received_msg.message, "ON") == 0)
                {
					if (xSemaphoreTake(xReleMutex, portMAX_DELAY)) {
						estado_Rele_Sector1 = 1;//actualizo la variable portegida que gestiona el blink
						xSemaphoreGive(xReleMutex);
					}
					gpio_set_level(RELAY_3, 1);
                    xTimerStart(timerSector1, portMAX_DELAY);             
                    ESP_LOGI(TAG_MQTT, "RELE 1 ENCENDIDO");
                    
                    //Actualizo también la pantalla física
					Rele1.estadoReles = true;
					Rele1.actualizado = true;
					
                }
                else if (strcmp(received_msg.message, "OFF") == 0)
                {
					if (xSemaphoreTake(xReleMutex, portMAX_DELAY)) {
						if(!estado_Rele_Sector2)//esto indica si la nebulizacion del otro sector esta activa o no. Sin tener en ceunta el blink!!
						{
						gpio_set_level(RELAY_3, 0);
						}
						estado_Rele_Sector1 = 0;//actualizo la variable portegida que gestiona el blink
						xSemaphoreGive(xReleMutex);
					}
                    xTimerStop(timerSector1, portMAX_DELAY);
					gpio_set_level(RELAY_1, 0);  // apagar relé,por si justo en el momento de parar el timer estaba encendido!
                    ESP_LOGI(TAG_MQTT, "RELE 1 APAGADO");
               		Rele1.estadoReles = false;
					Rele1.actualizado = true;
					
                }
            }
            else if (strcmp(received_msg.topic, "mariomp/f/activacionRele2") == 0)
            {
                if (strcmp(received_msg.message, "ON") == 0)
                {
					if (xSemaphoreTake(xReleMutex, portMAX_DELAY)) {
						estado_Rele_Sector2 = 1;//actualizo la variable protegida que gestiona el blink
						xSemaphoreGive(xReleMutex);
					 }
					gpio_set_level(RELAY_3, 1);
                    xTimerStart(timerSector2, portMAX_DELAY); 
                    ESP_LOGI(TAG_MQTT, "RELE 2 ENCENDIDO");
					Rele2.estadoReles = true;
					Rele2.actualizado = true;
					 
				
                }
                else if (strcmp(received_msg.message, "OFF") == 0)
                {
					if (xSemaphoreTake(xReleMutex, portMAX_DELAY)) {
						if(!estado_Rele_Sector1)//esto indica si la nebulizacion esta activa o no. Sin tener en ceunta el blink!!
						{
							gpio_set_level(RELAY_3, 0);
						}
						estado_Rele_Sector2 = 0;//actualizo la variable portegida que gestiona el blink
						xSemaphoreGive(xReleMutex);
					 }	
                    xTimerStop(timerSector2, portMAX_DELAY);
					gpio_set_level(RELAY_2, 0);  // apagar relé,por si justo en el momento de parar el timer estaba encendido!
                    ESP_LOGI(TAG_MQTT, "RELE 2 APAGADO");
					Rele2.estadoReles = false;
					Rele2.actualizado = true;
                }
            }
        }
    }
}


