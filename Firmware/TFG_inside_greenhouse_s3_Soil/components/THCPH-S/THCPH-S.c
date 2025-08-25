
#include <stdio.h>
#include <string.h>
#include "include/THCPH-S.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "freertos/projdefs.h"
#include "hal/uart_types.h"


// Configuración RS-485
// Configuración de pines RS-485 (ajusta según tu hardware)
//OJO : TXD y RXD del MAX485 se conectan invertidos en el ESP32 
//(RXD del MAX485 va a TXD del ESP32, y TXD del MAX485 va a RXD del ESP32).
#define TXD_PIN 17      // TX del ESP32 → RXD del MAX485
#define RXD_PIN 18      // RX del ESP32 ← TXD del MAX485
#define RTS_PIN 19 //enable pin  Para controlar la dirección del MAX485 (modo TX o RX).
#define UART_PORT UART_NUM_1 // 2 o 1

// Configuración del sensor THCPH-S
#define SENSOR_ADDRESS 0x01    // Dirección por defecto del sensor 0x01
#define REG_TEMP 0x0001        // Dirección del registro de temperatura
#define BAUDRATE 4800          // Baudrate según el datasheet 4800

 QueueHandle_t uart_event_queue;

void RS485_SetTX()
{
    gpio_set_level(RTS_PIN,1);
}
void RS485_SetRX()
{
	//uart_wait_tx_done(UART_PORT, portMAX_DELAY);
    gpio_set_level(RTS_PIN,0);
}
// Función para calcular el CRC-16 Modbus
uint16_t modbus_crc16(const uint8_t *data, uint8_t len) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

void send_modbus_request() {
    uint8_t request[8]; // Trama de solicitud Modbus

    // Construcción de la trama Modbus RTU para leer 3 registros (humedad, temperatura, conductividad)
    request[0] = SENSOR_ADDRESS;    // Dirección del esclavo
    request[1] = 0x03;              // Función: Read Holding Registers
    request[2] = 0x00;              // Dirección de inicio (Hi)
    request[3] = 0x00;              // Dirección de inicio (Lo)
    request[4] = 0x00;              // Número de registros (Hi)
    request[5] = 0x04;              // Número de registros (Lo)

    // Calcular el CRC-16 Modbus y agregarlo a la trama
    uint16_t crc = modbus_crc16(request, 6);
    request[6] = crc & 0xFF;       // CRC Low Byte
    request[7] = (crc >> 8) & 0xFF; // CRC High Byte
	
	    // Imprimir la trama enviada
    ESP_LOGI(TAG_MODBUS, "Enviando trama Modbus:");
    for (int i = 0; i < 8; i++) {
        ESP_LOGI(TAG_MODBUS, "0x%02X", request[i]);
    }
    
	//uart_flush_input(UART_PORT); // Limpiar antes de enviar
	RS485_SetTX();

    // Enviar la solicitud Modbus
    uart_write_bytes(UART_PORT, (const char *)request, sizeof(request));
    uart_wait_tx_done(UART_PORT,portMAX_DELAY);

    // Cambiar a modo recepción
     RS485_SetRX();
	
}



esp_err_t uart_read_sensor_THCPHs(uint16_t *humidity_raw, uint16_t *temperature_raw, uint16_t *conductivity_raw, uint16_t *ph_raw) {
    uint8_t response[13] = {0};
    int len = uart_read_bytes(UART_PORT, response, sizeof(response), pdMS_TO_TICKS(1000 ));
	
	if (len == 13) {
        ESP_LOGI(TAG_MODBUS, "Respuesta recibida:");
        for (int i = 0; i < len; i++) {
            ESP_LOGI(TAG_MODBUS, "0x%02X", response[i]);
        }
	
 
        // Verificar CRC
        uint16_t received_crc = response[len - 2] | (response[len - 1] << 8);  //  Little endian
        uint16_t calculated_crc = modbus_crc16(response, len - 2);

        if (received_crc == calculated_crc) {
            // Extraer datos
            *humidity_raw     = (response[3] << 8) | response[4];
            *temperature_raw  = (response[5] << 8) | response[6];
            *conductivity_raw = (response[7] << 8) | response[8];
            *ph_raw           = (response[9] << 8) | response[10];


            return ESP_OK;

        } else {
            ESP_LOGE(TAG_MODBUS, "CRC inválido: Recibido=0x%04X, Esperado=0x%04X", received_crc, calculated_crc);
            return ESP_ERR_INVALID_CRC;
        }
    }

    ESP_LOGE(TAG_MODBUS, "No se recibió la respuesta esperada del sensor (len=%d)", len);
    return ESP_FAIL;
}


void init_uart() {
    uart_config_t uart_config = {
        .baud_rate = BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE, //sEGÚN DATAHSEEt!!
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
	
    // Configurar UART
    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_PORT, TXD_PIN, RXD_PIN, RTS_PIN, UART_PIN_NO_CHANGE);
    
    
    
    uart_driver_install(UART_PORT, 512, 512, 10, &uart_event_queue, 0);

}

