#ifndef SCREEN_H
#define SCREEN_H

#include "misc/lv_types.h"
#include <freertos/FreeRTOS.h>

#include <freertos/semphr.h>
#include <freertos/timers.h>
#include <driver/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif
extern const gpio_num_t SPI_CLOCK;
extern const gpio_num_t SPI_MOSI;
extern const gpio_num_t SPI_MISO;
extern const gpio_num_t TFT_CS;
extern const gpio_num_t TFT_RESET;
extern const gpio_num_t TFT_DC;
extern const gpio_num_t TOUCH_CS;
extern const gpio_num_t TFT_BL;

// Variables externas
extern SemaphoreHandle_t screen_mutex;
extern SemaphoreHandle_t semaforo_activar_timer_screen;
extern TaskHandle_t screen_task_handle;
extern TimerHandle_t apagar_screen_timer;
extern volatile bool screen_on;


static bool ignore_touch = false;

//para comunicar desde el resto de pantalla con lvgl se usa:

typedef enum {
    GUI_MSG_LABEL_SET_TEXT,
    GUI_MSG_SWITCH_SET_STATE
} lvgl_msg_type_t;

typedef struct {
    lv_obj_t *obj;
    lvgl_msg_type_t tipo;

    union {
        char texto[10];  // Para labels
        bool estado;     // Para switches
    } data;

} lvgl_msg_t;
// Funciones públicas
void SCREEN(void *param);
void SCREEN_ON_OFF(void *param);
void apagar_screen_task_cb(TimerHandle_t xTimer);
void encender_pantalla(void);
void apagar_pantalla(TimerHandle_t xTimer);
void actualizar_datos_sensores(void);

void async_gui_callback(void *data) ;
void enviar_texto_label_async(lv_obj_t *label, const char *texto);
#ifdef __cplusplus
}
#endif

#endif // SCREEN_H


