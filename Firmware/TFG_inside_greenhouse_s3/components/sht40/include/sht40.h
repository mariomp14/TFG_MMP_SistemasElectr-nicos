#ifndef SHT40_H
#define SHT40_H

#include <stdint.h>
#include "esp_attr.h"
#include "esp_err.h"
#include "driver/i2c_master.h"

// Dirección I2C del sensor SHT40
#define SHT40_I2C_ADDRESS 0x44

// Comandos específicos del SHT40
#define SHT40_CMD_MEASURE_HIGH_PRECISION  0xFD
#define SHT40_CMD_MEASURE_MEDIUM_PRECISION  0xF6
#define SHT40_CMD_MEASURE_LOW_PRECISION  0xE0

//Checkar si estos tipos de datos son correctos/necesarios
extern RTC_DATA_ATTR volatile uint8_t variablePreservada1 ;// variable que me sirve para comprobar que el ACK recibido es correcto 
extern RTC_DATA_ATTR volatile uint16_t sleep_time ;

#define SHORT_TIME 180
#define LONG_TIME 660
// Función para inicializar el bus I2C
void i2c_master_init_bus();

// Función para inicializar el dispositivo SHT40 en el bus I2C
void i2c_master_init_handle( uint8_t address);

// Función para escribir un byte en el sensor SHT40
esp_err_t write_byte_i2c(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr);

// Función para leer datos desde el sensor SHT40 sin enviar un registro previo
esp_err_t read_byte_i2c_no_reg(i2c_master_dev_handle_t dev_handle, uint8_t *data, size_t len);

// Tarea de lectura del sensor SHT40
// Función para leer una sola medición de temperatura y humedad del SHT40
esp_err_t read_sht40(float *temperature, float *humidity);

//Lee temperatura y humedad sin convertir (raw)
esp_err_t read_sht40_raw(uint16_t *raw_temperature, uint16_t *raw_humidity);

//Está función convierte los datos de temperatura y humedad a valores "reales"
//y devuelve un 1 o un 0 según dichos valores para actualizar el valor de la variable preservada
//que es la que determina cada cuanto tiempo se sale de deep sleep
esp_err_t convert_raw_sensor_data(uint16_t raw_temp, uint16_t raw_hum);
#endif // SHT40_H

