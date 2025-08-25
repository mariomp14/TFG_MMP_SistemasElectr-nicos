#include "screen.h"
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_ili9488.h>
#include <esp_lcd_touch_xpt2046.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <lvgl.h>
#include <string.h>
#include "sdkconfig.h"
#include "hal/color_types.h"
#include "soc/gpio_num.h"
#include "ui.h"  // UI generada con SquareLine Studio

//static const char *TAG_SCREEN = "screen";

static const int DISPLAY_HORIZONTAL_PIXELS = 480;
static const int DISPLAY_VERTICAL_PIXELS = 320;
static const int DISPLAY_COMMAND_BITS = 8;
static const int DISPLAY_PARAMETER_BITS = 8;
static const unsigned int DISPLAY_REFRESH_HZ = 40000000;
static const int DISPLAY_SPI_QUEUE_LEN = 10;
static const int SPI_MAX_TRANSFER_SIZE = 32768;

const gpio_num_t SPI_CLOCK = GPIO_NUM_18;
const gpio_num_t SPI_MOSI  = GPIO_NUM_11;
const gpio_num_t SPI_MISO  = GPIO_NUM_19;
const gpio_num_t TFT_CS    = GPIO_NUM_16;
const gpio_num_t TFT_RESET = GPIO_NUM_17;
const gpio_num_t TFT_DC    = GPIO_NUM_2;
const gpio_num_t TOUCH_CS  = GPIO_NUM_15;
const gpio_num_t TFT_BL    = GPIO_NUM_3;


static const lcd_rgb_element_order_t TFT_COLOR_MODE = COLOR_RGB_ELEMENT_ORDER_BGR;
static const size_t LV_BUFFER_SIZE = DISPLAY_HORIZONTAL_PIXELS * 20;
static const int LVGL_UPDATE_PERIOD_MS = 5;

static esp_lcd_panel_io_handle_t lcd_io_handle = NULL;
static esp_lcd_panel_handle_t lcd_handle = NULL;
esp_lcd_touch_handle_t touch_handle = NULL;
esp_lcd_panel_io_handle_t tp_io_handle = NULL;
static lv_display_t *display = NULL;
static lv_color_t *lv_buf_1 = NULL;
static lv_color_t *lv_buf_2 = NULL;

TaskHandle_t screen_task_handle = NULL;
TimerHandle_t apagar_screen_timer = NULL;
SemaphoreHandle_t screen_mutex;
SemaphoreHandle_t semaforo_activar_timer_screen;
volatile bool screen_on = false;



static void IRAM_ATTR lvgl_tick_cb(void *param) {
    lv_tick_inc(LVGL_UPDATE_PERIOD_MS);
}

static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
    esp_lcd_panel_draw_bitmap(lcd_handle, area->x1, area->y1, area->x2 + 1, area->y2 + 1, px_map);
    lv_display_flush_ready(disp);
}

static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io,
                                    esp_lcd_panel_io_event_data_t *edata,
                                    void *user_ctx) {
    lv_display_t *disp = (lv_display_t *)user_ctx;
    lv_display_flush_ready(disp);
    return false;
}

static void initialize_spi() {
    spi_bus_config_t bus = {
        .mosi_io_num = SPI_MOSI,
        .miso_io_num = SPI_MISO,
        .sclk_io_num = SPI_CLOCK,
        .max_transfer_sz = SPI_MAX_TRANSFER_SIZE,
        .flags = SPICOMMON_BUSFLAG_SCLK | SPICOMMON_BUSFLAG_MISO |
                 SPICOMMON_BUSFLAG_MOSI | SPICOMMON_BUSFLAG_MASTER,
        .intr_flags = ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &bus, SPI_DMA_CH_AUTO));
}

static void initialize_display() {
    const esp_lcd_panel_io_spi_config_t io_config = {
        .cs_gpio_num = TFT_CS,
        .dc_gpio_num = TFT_DC,
        .spi_mode = 0,
        .pclk_hz = DISPLAY_REFRESH_HZ,
        .trans_queue_depth = DISPLAY_SPI_QUEUE_LEN,
        .on_color_trans_done = notify_lvgl_flush_ready,
        .user_ctx = &display,
        .lcd_cmd_bits = DISPLAY_COMMAND_BITS,
        .lcd_param_bits = DISPLAY_PARAMETER_BITS,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,4,0)
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
#endif
        .flags = {
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
            .dc_as_cmd_phase = 0,
            .dc_low_on_data = 0,
            .octal_mode = 0,
            .lsb_first = 0
#else
            .dc_low_on_data = 0,
            .octal_mode = 0,
            .sio_mode = 0,
            .lsb_first = 0,
            .cs_high_active = 0
#endif
        }
    };

    const esp_lcd_panel_dev_config_t lcd_config = {
        .reset_gpio_num = TFT_RESET,
        .color_space = TFT_COLOR_MODE,
        .bits_per_pixel = 18,
        .flags = { .reset_active_high = 0 },
        .vendor_config = NULL
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI3_HOST, &io_config, &lcd_io_handle));
    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9488(lcd_io_handle, &lcd_config, LV_BUFFER_SIZE, &lcd_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(lcd_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(lcd_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(lcd_handle, false));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(lcd_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(lcd_handle, true, false));
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(lcd_handle, 0, 0));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(lcd_handle, true));
}

static void initialize_touch() {
    esp_lcd_panel_io_spi_config_t tp_io_config = ESP_LCD_TOUCH_IO_SPI_XPT2046_CONFIG(TOUCH_CS);
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI3_HOST, &tp_io_config, &tp_io_handle));

    esp_lcd_touch_config_t tp_cfg = {
        .x_max = DISPLAY_HORIZONTAL_PIXELS,
        .y_max = DISPLAY_VERTICAL_PIXELS,
        .rst_gpio_num = -1,
        .int_gpio_num = GPIO_NUM_NC,
        .levels = { .interrupt = 0 },
        .flags = {
            .swap_xy = true,
            .mirror_x = true,
            .mirror_y = true,
        },
        .user_data = NULL
    };

    ESP_ERROR_CHECK(esp_lcd_touch_new_spi_xpt2046(tp_io_handle, &tp_cfg, &touch_handle));
}

static void reset_touch_flag_cb(lv_timer_t * t)
{
    ignore_touch = false;
    lv_timer_del(t);  // Borra el timer después de ejecutarse una vez
}

static void lvgl_touch_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    uint16_t raw_x[1], raw_y[1];
    uint8_t count = 0;

    static int16_t last_x = -1, last_y = -1;
    
   if (touch_handle == NULL) {
        data->state = LV_INDEV_STATE_RELEASED;
        return;
    }

	esp_lcd_touch_read_data(touch_handle);
		if (esp_lcd_touch_get_coordinates(touch_handle, raw_x, raw_y,
											 NULL, &count, 1)/*&& count > 0*/) {

        lv_indev_reset(lv_indev_get_next(NULL), NULL);
        data->state = LV_INDEV_STATE_RELEASED;
        
        if (count > 0) {
            // Escalado de X: de [25–300] → [0–480]
            int scaled_x = (raw_x[0] - 25) * 480 / (300 - 25);
            if (scaled_x < 0) scaled_x = 0;
            if (scaled_x > 479) scaled_x = 479;

            // Escalado de Y: de [60–400] → [0–320]
            int scaled_y = (raw_y[0] - 60) * 320 / (400 - 60);
            if (scaled_y < 0) scaled_y = 0;
            if (scaled_y > 319) scaled_y = 319;
			
			
			//if (((abs(scaled_x - last_x) > 20 && abs(scaled_y - last_y) > 20)) && ignore_touch)
			if(ignore_touch)
			{
				return;	
			}

            last_x = scaled_x;
            last_y = scaled_y;
			
        	
            data->point.x = scaled_x;
            data->point.y = scaled_y;
            data->state = LV_INDEV_STATE_PRESSED;
		
	    	// Crear timer que desactiva el bloqueo de toques después de 500 ms
			ignore_touch = true;
			lv_timer_create(reset_touch_flag_cb, 500, NULL);
          
        } 
   
        }
		else
    {
        // No hay toque activo
		data->point.x = last_x;
        data->point.y = last_y;
        data->state = LV_INDEV_STATE_RELEASED;
    }

}
static void initialize_input(void) {
    lv_indev_t *input_dev = lv_indev_create();
    lv_indev_set_type(input_dev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(input_dev, lvgl_touch_cb); // 
}

static void initialize_lvgl() {
    lv_init();

    lv_buf_1 = (lv_color_t *)heap_caps_malloc(LV_BUFFER_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
#if USE_DOUBLE_BUFFERING
    lv_buf_2 = (lv_color_t *)heap_caps_malloc(LV_BUFFER_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
#endif

    display = lv_display_create(DISPLAY_HORIZONTAL_PIXELS, DISPLAY_VERTICAL_PIXELS);
    lv_display_set_buffers(display, lv_buf_1, lv_buf_2, LV_BUFFER_SIZE, LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_flush_cb(display, lvgl_flush_cb);
    lv_display_set_user_data(display, lcd_handle);

    initialize_input();
    ui_init();

    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &lvgl_tick_cb,
        .name = "lvgl_tick"
    };

    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_UPDATE_PERIOD_MS * 1000));
}

void actualizar_datos_sensores() {
    // Sector 1 ambiente
    actualizar_label_con_float(12.34, SectorS1Amb.dpv);
    actualizar_label_con_float(23.56, SectorS1Amb.temp_amb);
    actualizar_label_con_float(45.67, SectorS1Amb.hum_amb);

    // Sector 2 ambiente
    actualizar_label_con_float(21.43, SectorS2Amb.dpv);
    actualizar_label_con_float(32.65, SectorS2Amb.temp_amb);
    actualizar_label_con_float(54.76, SectorS2Amb.hum_amb);

    // Datos suelo
    actualizar_label_con_float(33.11, SoilData1.hum);
    actualizar_label_con_float(24.78, SoilData1.temp);
    actualizar_label_con_float(6.45, SoilData1.ph);
    actualizar_label_con_float(1.23, SoilData1.conduc);
    }
    
void SCREEN(void *param) {
    initialize_spi();
    initialize_display();
    initialize_touch();
    initialize_lvgl();
    //screen_on = true;
     esp_lcd_panel_disp_on_off(lcd_handle, false);
     gpio_set_level(TFT_BL, 0);
     vTaskSuspend(screen_task_handle);
	
   // actualizar_datos_sensores();

    while (1) {
        		lv_timer_handler(); 
        //unica tarea que iunteractua con lvgl ya que no es threadSafe por si misma
if (temp_S1.actualizado && ui_VALORtempS1 != NULL && lv_obj_is_valid(ui_VALORtempS1)) {
    lv_label_set_text(ui_VALORtempS1, temp_S1.valor);
    temp_S1.actualizado = false;
}

if (hum_S1.actualizado && ui_VALORhumS1 != NULL && lv_obj_is_valid(ui_VALORhumS1)) {
    lv_label_set_text(ui_VALORhumS1, hum_S1.valor);
    hum_S1.actualizado = false;
}

if (temp_S2.actualizado && ui_VALORtempS2 != NULL && lv_obj_is_valid(ui_VALORtempS2)) {
    lv_label_set_text(ui_VALORtempS2, temp_S2.valor);
    temp_S2.actualizado = false;
}

if (hum_S2.actualizado && ui_VALORhumS2 != NULL && lv_obj_is_valid(ui_VALORhumS2)) {
    lv_label_set_text(ui_VALORhumS2, hum_S2.valor);
    hum_S2.actualizado = false;
}

if (temp_suelo.actualizado && ui_VALORtempSuelo != NULL && lv_obj_is_valid(ui_VALORtempSuelo)) {
    lv_label_set_text(ui_VALORtempSuelo, temp_suelo.valor);
    temp_suelo.actualizado = false;
}

if (hum_suelo.actualizado && ui_VALORhumSuelo != NULL && lv_obj_is_valid(ui_VALORhumSuelo)) {
    lv_label_set_text(ui_VALORhumSuelo, hum_suelo.valor);
    hum_suelo.actualizado = false;
}

if (cond_suelo.actualizado && ui_VALORConducSuelo != NULL && lv_obj_is_valid(ui_VALORConducSuelo)) {
    lv_label_set_text(ui_VALORConducSuelo, cond_suelo.valor);
    cond_suelo.actualizado = false;
}

if (ph_suelo.actualizado && ui_VALORPHSuelo != NULL && lv_obj_is_valid(ui_VALORPHSuelo)) {
    lv_label_set_text(ui_VALORPHSuelo, ph_suelo.valor);
    ph_suelo.actualizado = false;
}

if (Rele1.actualizado && ui_NEBULIZAR_SECTOR1 != NULL && lv_obj_is_valid(ui_NEBULIZAR_SECTOR1)) {
    if (Rele1.estadoReles) {
        lv_obj_add_state(ui_NEBULIZAR_SECTOR1, LV_STATE_CHECKED);  // Encendido
    } else {
        lv_obj_clear_state(ui_NEBULIZAR_SECTOR1, LV_STATE_CHECKED);  // Apagado
    }
    Rele1.actualizado = false;
}

// Relé 2 (switch)
if (Rele2.actualizado && ui_NEBULIZAR_SECTOR2 != NULL && lv_obj_is_valid(ui_NEBULIZAR_SECTOR2)) {
    if (Rele2.estadoReles) {
        lv_obj_add_state(ui_NEBULIZAR_SECTOR2, LV_STATE_CHECKED);
    } else {
        lv_obj_clear_state(ui_NEBULIZAR_SECTOR2, LV_STATE_CHECKED);
    }
    Rele2.actualizado = false;
}

if (DPVS1.actualizado && ui_VALORDPVS1 != NULL && lv_obj_is_valid(ui_VALORDPVS1)) {
    lv_label_set_text(ui_VALORDPVS1, DPVS1.valor);
    DPVS1.actualizado = false;
}

if (DPVS2.actualizado && ui_VALORDPVS2 != NULL && lv_obj_is_valid(ui_VALORDPVS2)) {
    lv_label_set_text(ui_VALORDPVS2, DPVS2.valor);
    DPVS2.actualizado = false;
}

if (DPVminS1.actualizado && ui_consignaDPVMinS1 != NULL && lv_obj_is_valid(ui_consignaDPVMinS1)) {
    lv_label_set_text(ui_consignaDPVMinS1, DPVminS1.valor);
    DPVminS1.actualizado = false;
}

if (DPVmaxS1.actualizado && ui_consignaDPVMaxS1 != NULL && lv_obj_is_valid(ui_consignaDPVMaxS1)) {
    lv_label_set_text(ui_consignaDPVMaxS1, DPVmaxS1.valor);
    DPVmaxS1.actualizado = false;
}

if (DPVminS2.actualizado && ui_consignaDPVMinS2 != NULL && lv_obj_is_valid(ui_consignaDPVMinS2)) {
    lv_label_set_text(ui_consignaDPVMinS2, DPVminS2.valor);
    DPVminS2.actualizado = false;
}

if (DPVmaxS2.actualizado && ui_consignaDPVMaxS2 != NULL && lv_obj_is_valid(ui_consignaDPVMaxS2)) {
    lv_label_set_text(ui_consignaDPVMaxS2, DPVmaxS2.valor);
    DPVmaxS2.actualizado = false;
}

// Automatismo (switch)
if (estado_automatismo.actualizado && ui_SwitchDesactivarAutomatismo != NULL && lv_obj_is_valid(ui_SwitchDesactivarAutomatismo)) {
    if (estado_automatismo.estadoReles) {
        lv_obj_add_state(ui_SwitchDesactivarAutomatismo, LV_STATE_CHECKED);
    } else {
        lv_obj_clear_state(ui_SwitchDesactivarAutomatismo, LV_STATE_CHECKED);
    }
    estado_automatismo.actualizado = false;
}
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void SCREEN_ON_OFF(void *param) {
    //control_task_handle = xTaskGetCurrentTaskHandle();

    for (;;) {
        if (xSemaphoreTake(semaforo_activar_timer_screen, portMAX_DELAY) == pdTRUE) 
        {

        	if (!screen_on) {
            	encender_pantalla();  // 🔁 Enciende si estaba apagada
        	} else {
            // Si ya estaba encendida, solo reinicia el timer
            if (apagar_screen_timer) {
                xTimerReset(apagar_screen_timer, 0);
            	}
        	}
    	}
    }
}

void encender_pantalla() {
    if (xSemaphoreTake(screen_mutex, portMAX_DELAY)) {
        if (!screen_on) {
            screen_on = true;
            esp_lcd_panel_disp_on_off(lcd_handle, true);
            gpio_set_level(TFT_BL, 1);
            vTaskResume(screen_task_handle);
        }
        xSemaphoreGive(screen_mutex);
    }
    if (apagar_screen_timer) {
        xTimerReset(apagar_screen_timer, 0);
    } else {
        xTimerStart(apagar_screen_timer, 0);
    }
}

void apagar_pantalla(TimerHandle_t xTimer) {
    if (xSemaphoreTake(screen_mutex, portMAX_DELAY)) {
        if (screen_on) {
            screen_on = false;
            esp_lcd_panel_disp_on_off(lcd_handle, false);
            gpio_set_level(TFT_BL, 0);
            vTaskSuspend(screen_task_handle);
        }
        xSemaphoreGive(screen_mutex);
    }
}


void apagar_screen_task_cb(TimerHandle_t xTimer) {
    ESP_LOGI("MAIN", "Apagando pantalla tras 2 minutos");
    apagar_pantalla(NULL);
}


/*
void async_gui_callback(void *data) {
    lvgl_msg_t *msg = (lvgl_msg_t *)data;

    if (msg == NULL || msg->obj == NULL || !lv_obj_is_valid(msg->obj)||!screen_on) {
        free(msg);
        return;
    }

    switch (msg->tipo) {
        case GUI_MSG_LABEL_SET_TEXT:
            lv_label_set_text(msg->obj, msg->data.texto);
            break;

        case GUI_MSG_SWITCH_SET_STATE:
            if (msg->data.estado)
                lv_obj_add_state(msg->obj, LV_STATE_CHECKED);
            else
                lv_obj_clear_state(msg->obj, LV_STATE_CHECKED);
            break;
    }

    free(msg);
}
void enviar_texto_label_async(lv_obj_t *label, const char *texto) {
    if (label == NULL || texto == NULL) return;

    lvgl_msg_t *msg = malloc(sizeof(lvgl_msg_t));
    if (msg == NULL) return;

    msg->obj = label;

    if (label == ui_NEBULIZAR_SECTOR1 || label == ui_NEBULIZAR_SECTOR2) {
        msg->tipo = GUI_MSG_SWITCH_SET_STATE;
        msg->data.estado = (strcmp(texto, "1") == 0);  // Activar si texto es "1"
    } else {
        msg->tipo = GUI_MSG_LABEL_SET_TEXT;
        snprintf(msg->data.texto, sizeof(msg->data.texto), "%s", texto);
    }

    lv_async_call(async_gui_callback, msg);
}*/
