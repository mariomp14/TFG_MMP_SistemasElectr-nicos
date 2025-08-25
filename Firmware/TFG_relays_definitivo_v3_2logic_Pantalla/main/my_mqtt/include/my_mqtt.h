#ifndef MY_MQTT_H
#define MY_MQTT_H


#include <stdio.h>
#include "mqtt_client.h"
#include "nvs_flash.h"

extern esp_mqtt_client_handle_t client ;//declaracion

//Mutex para acceder a las variables umbral_temperatura y umbral_humedad ya que
//puede darse el caso de que sea accedida simultaneamente por la tarea que comprueba el umbral para nebulizar o no
//o por la callback que modifica el valor de dicha variable
extern SemaphoreHandle_t mutexUmbrales;
extern float umbral_temperatura;
extern float umbral_humedad;
// Estructura para almacenar los datos MQTT
typedef struct {
    char mqtt_url[50];
    char mqtt_user[20];
    char mqtt_pass[50];
} mqtt_config_t;

typedef struct {
    char topic[50];  // Ajusta el tamaño
    char message[10];
} mqtt_message_t;

extern QueueHandle_t mqtt_queue; // Cola para mensajes MQTT

// Declaración del handle para que sea visible en otros ficheros
extern TimerHandle_t timerSector1;
extern TimerHandle_t timerSector2;
extern const TickType_t tiempo_on ;   // 3s encendido
extern const TickType_t tiempo_off ;  // 7s apagado
															
void parpadeoAsimCallbackRelay1(TimerHandle_t xTimer);
void parpadeoAsimCallbackRelay2(TimerHandle_t xTimer);

extern volatile uint8_t estado_Rele_Sector1;
extern volatile uint8_t estado_Rele_Sector2;

#define RELAY_1 GPIO_NUM_6
#define RELAY_2 GPIO_NUM_7
#define RELAY_3 GPIO_NUM_8
#define RELAY_4 GPIO_NUM_9

extern SemaphoreHandle_t xReleMutex;

mqtt_config_t leer_mqtt_desde_nvs(void);
void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
void mqtt_app_start(void); 
void MQTT(void *pvParameters);
#endif  
