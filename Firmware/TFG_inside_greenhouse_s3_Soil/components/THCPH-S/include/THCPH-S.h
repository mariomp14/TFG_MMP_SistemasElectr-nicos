#ifndef MODBUS_MANUAL_H
#define MODBUS_MANUAL_H

#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Etiqueta de log
#define TAG_MODBUS "MODBUS"


#define TXD_PIN 17      // TX del ESP32 → RXD del MAX485
#define RXD_PIN 18      // RX del ESP32 ← TXD del MAX485
#define RTS_PIN 19
#define UART_PORT UART_NUM_1

// Parámetros del sensor
#define SENSOR_ADDRESS 0x01
#define REG_TEMP 0x0001
#define BAUDRATE 4800

#ifdef __cplusplus
extern "C" {
#endif

extern QueueHandle_t uart_event_queue;

// Funciones de dirección de transmisión
void RS485_SetTX(void);
void RS485_SetRX(void);

// Función para calcular CRC-16 Modbus
uint16_t modbus_crc16(const uint8_t *data, uint8_t len);

// Funciuón que envia solicitud Modbus
void send_modbus_request(void);

// Tarea que maneja eventos UART (recepción de respuesta Modbus)
esp_err_t uart_read_sensor_THCPHs(uint16_t *humidity_raw, uint16_t *temperature_raw, uint16_t *conductivity_raw, uint16_t *ph_raw);

// Función para inizializar la UART con configuración Modbus RS-485
void init_uart(void);

#ifdef __cplusplus
}
#endif

#endif // MODBUS_MANUAL_H
