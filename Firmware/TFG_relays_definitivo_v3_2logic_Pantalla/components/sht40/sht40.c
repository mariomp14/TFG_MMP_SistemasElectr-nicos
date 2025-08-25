#include "include/sht40.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "soc/gpio_num.h"

#define I2C_MASTER_TIMEOUT_MS  1000

static const char *TAG = "SHT40";
static i2c_master_bus_handle_t bus_handle = NULL;
static i2c_master_dev_handle_t dev_handle = NULL;

void i2c_master_init_bus() {
	
 // Verificar si el bus ya está inicializado
    if (bus_handle != NULL) {
        ESP_LOGW(TAG, "Liberando dispositivo I2C...");
        if (dev_handle != NULL) {
            i2c_master_bus_rm_device(dev_handle);  // Quitar dispositivo antes de liberar el bus
            dev_handle = NULL;
        }

        ESP_LOGW(TAG, "Liberando bus I2C antes de reinicializar...");
        i2c_del_master_bus(bus_handle);
        bus_handle = NULL;
    }

    
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = GPIO_NUM_8,
        .scl_io_num = GPIO_NUM_9,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));
}

void i2c_master_init_handle(uint8_t address) {
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle));
}

esp_err_t write_byte_i2c(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr) {
    return i2c_master_transmit(dev_handle, &reg_addr, sizeof(reg_addr), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t read_byte_i2c_no_reg(i2c_master_dev_handle_t dev_handle, uint8_t *data, size_t len) {
    return i2c_master_receive(dev_handle, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t read_sht40(float *temperature, float *humidity) {

    uint8_t data[6] = {0};  // 2 bytes for raw temperature data, 2 bytes for raw humidity data
    esp_err_t err;

    // Enviar comando para medición de alta precisión
    err = write_byte_i2c(dev_handle, SHT40_CMD_MEASURE_HIGH_PRECISION);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error al escribir en el dispositivo I2C");
        return err;
    }

    // Esperar 100 ms para que los datos sean disponibles
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // Leer 6 bytes de datos (2 bytes para temperatura, 2 bytes para humedad)
    err = read_byte_i2c_no_reg(dev_handle, data, 6);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error al leer datos desde el sensor");
        return err;
    }
    //iMPLEMENTAR COMPROBACIÓN DE CRC?

    // Procesar los datos de temperatura y humedad
    uint16_t raw_temp = (data[0] << 8) | data[1];
    uint16_t raw_humidity = (data[3] << 8) | data[4];

    // Convertir a valores reales de temperatura y humedad según el datasheet
    *temperature = -45.0f + 175.0f * ((float)raw_temp / 65535.0f);
    *humidity = 125.0f * ((float)raw_humidity / 65535.0f) - 6.0f;

    ESP_LOGI(TAG, "Temperatura: %.2f °C", *temperature);
    ESP_LOGI(TAG, "Humedad: %.2f %%", *humidity);

    return ESP_OK;
}

esp_err_t read_sht40_raw(uint16_t *raw_temperature, uint16_t *raw_humidity) {

    uint8_t data[6] = {0};  // 2 bytes for raw temperature data, 2 bytes for raw humidity data
    esp_err_t err;

    // Enviar comando para medición de alta precisión
    err = write_byte_i2c(dev_handle, SHT40_CMD_MEASURE_HIGH_PRECISION);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error al escribir en el dispositivo I2C");
        return err;
    }

    // Esperar 100 ms para que los datos sean disponibles
    vTaskDelay(100 / portTICK_PERIOD_MS);//mirar datasheet (no es necesario tanta espera)

    // Leer 6 bytes de datos (2 bytes para temperatura, 2 bytes para humedad)
    err = read_byte_i2c_no_reg(dev_handle, data, 6);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error al leer datos desde el sensor");
        return err;
    }
    
    // Procesar los datos de temperatura y humedad
     *raw_temperature = (data[0] << 8) | data[1];
     *raw_humidity = (data[3] << 8) | data[4];
    //iMPLEMENTAR COMPROBACIÓN DE CRC?

    return ESP_OK;
}

//En este programa no utilizo está función
/*
esp_err_t convert_raw_sensor_data(uint16_t raw_temp, uint16_t raw_hum)
{
	float temp, hum;
  
    // Convertir a valores reales de temperatura y humedad según el datasheet
    temp = -45.0f + 175.0f * ((float)raw_temp / 65535.0f);
    hum = 125.0f * ((float)raw_hum / 65535.0f) - 6.0f;

    ESP_LOGI(TAG, "Temperatura: %.2f °C", temp);
    ESP_LOGI(TAG, "Humedad: %.2f %%", hum);
	
	if(hum < 60.0 || (hum < 70.0 && temp > 30.0))
	{
		//variablePreservada = 1;
		sleep_time = 180; //3 minutos en segundos
	}
	else 
	{
		//variablePreservada = 0;
		sleep_time = 900; // 15 minutos en segundos
	}
	return ESP_OK;
}*/
    
void convert4bytes_to_data(uint8_t *buf, float *temp, float *hum)
{
	uint16_t raw_temp, raw_hum;
	
	raw_temp = (buf[2] << 8) | buf[3];
    raw_hum = (buf[4] << 8) | buf[5];
    
    // Convertir a valores reales de temperatura y humedad según el datasheet
    *temp = -45.0f + 175.0f * ((float)raw_temp / 65535.0f);
    *hum = 125.0f * ((float)raw_hum / 65535.0f) - 6.0f;
    
}

void convert12bytes_to_data(uint8_t *buf,
                                    float *temp_amb, float *hum_amb,
                                    float *temp_suelo, float *hum_suelo,
                                    float *cond, float *ph)
{
    int16_t raw_temp_amb     = (buf[2] << 8) | buf[3];
    uint16_t raw_hum_amb     = (buf[4] << 8) | buf[5];
    int16_t raw_temp_suelo  = (buf[6] << 8) | buf[7];
    uint16_t raw_hum_suelo   = (buf[8] << 8) | buf[9];
    uint16_t raw_cond        = (buf[10] << 8) | buf[11];
    uint16_t raw_ph          = (buf[12] << 8) | buf[13];

    // Conversión según fórmulas datahseetsHT40 
    *temp_amb   = -45.0f + 175.0f * ((float)raw_temp_amb / 65535.0f);
    *hum_amb    = 125.0f * ((float)raw_hum_amb / 65535.0f) - 6.0f;

    *temp_suelo = raw_temp_suelo / 10.0f;  // si el sensor devuelve °C * 10
    *hum_suelo  = raw_hum_suelo / 10.0f;

    *cond       = raw_cond* 1.0f;  ;                // Conductividad sin conversión
    *ph         = raw_ph /10.0f;                  // 
}