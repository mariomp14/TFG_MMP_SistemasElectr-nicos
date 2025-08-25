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


RTC_DATA_ATTR volatile uint16_t sleep_time = 0;

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
    //i2c_master_bus_handle_t bus_handle;
    //i2c_master_dev_handle_t dev_handle;
   // i2c_master_init_bus();
   // i2c_master_init_handle( SHT40_I2C_ADDRESS);

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

esp_err_t convert_raw_sensor_data(uint16_t raw_temp, uint16_t raw_hum)
{
	float temp, hum;
  
    // Convertir a valores reales de temperatura y humedad según el datasheet
    temp = -45.0f + 175.0f * ((float)raw_temp / 65535.0f);
    hum = 125.0f * ((float)raw_hum / 65535.0f) - 6.0f;

    ESP_LOGI(TAG, "Temperatura: %.2f °C", temp);
    ESP_LOGI(TAG, "Humedad: %.2f %%", hum);
	
	//lo de la variable preservada ya no tiene sentido ya que las condiciones de DPV son consignables 
	//en el nodo Maestro y el Nodo de dentro del invernadero no lo conoce. El tiempo de sleep ahora lo 
	//determino segun el ack que me devuelva el nodo relays
	
	/*if(hum < 60.0 || (hum < 70.0 && temp > 30.0))
	//if(hum > 60.0)
	{
		variablePreservada1 = 1;
		sleep_time = SHORT_TIME; //3 minutos en segundos
		ESP_LOGE(TAG, "Valor sleep_time: 180 segundos");
	}
	else 
	{
		variablePreservada1 = 0;
		ESP_LOGE(TAG, "Valor sleep_time 660 segundos");
		sleep_time = LONG_TIME; // 15 minutos en segundos (900)
	}*/
	return ESP_OK;
}
    
