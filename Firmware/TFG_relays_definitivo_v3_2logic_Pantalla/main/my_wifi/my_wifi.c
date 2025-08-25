#include <stdio.h>
#include <string.h>
#include "include/my_wifi.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_netif.h"
//#include "esp_wifi_types_generic.h"

#include "include/my_wifi.h"
#include "../my_mqtt/include/my_mqtt.h"

static const char *TAG_WIFI = "WIFI_SETUP";


const char *html_page = "<!DOCTYPE html>"
"<html lang=\"es\">"
"<head>"
"    <meta charset=\"UTF-8\">"
"    <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"
"    <title>Configurar WiFi y MQTT - ESP32</title>"
"    <style>"
"        body {"
"            font-family: Arial, sans-serif;"
"            text-align: center;"
"            background-color: #f4f4f4;"
"            margin: 0;"
"            padding: 0;"
"        }"
"        .container {"
"            width: 90%;"
"            max-width: 400px;"
"            background: white;"
"            padding: 20px;"
"            border-radius: 10px;"
"            box-shadow: 0px 4px 10px rgba(0, 0, 0, 0.1);"
"            margin: 50px auto;"
"        }"
"        h2 {"
"            color: #333;"
"        }"
"        input {"
"            width: calc(100% - 20px);"
"            padding: 10px;"
"            margin: 10px 0;"
"            border: 1px solid #ccc;"
"            border-radius: 5px;"
"            font-size: 16px;"
"        }"
"        button {"
"            width: 100%;"
"            padding: 12px;"
"            background: #007BFF;"
"            color: white;"
"            border: none;"
"            border-radius: 5px;"
"            font-size: 18px;"
"            cursor: pointer;"
"            transition: 0.3s;"
"        }"
"        button:hover {"
"            background: #0056b3;"
"        }"
"        .footer {"
"            margin-top: 20px;"
"            font-size: 14px;"
"            color: #777;"
"        }"
"    </style>"
"</head>"
"<body>"
"    <div class=\"container\">"
"        <h2>Configurar WiFi y MQTT</h2>"
"        <form action=\"/config\" method=\"post\">"
"            <h3>WiFi</h3>"
"            <input type=\"text\" name=\"ssid\" placeholder=\"Nombre de la Red (SSID)\" required>"
"            <input type=\"password\" name=\"wifi_pass\" placeholder=\"Contraseña WiFi\" required>"
"            <h3>MQTT</h3>"
"            <input type=\"text\" name=\"mqtt_url\" placeholder=\"URL del Broker MQTT (ej: mqtt://io.adafruit.com/)\" required>"
"            <input type=\"text\" name=\"mqtt_user\" placeholder=\"Usuario MQTT\" required>"
"            <input type=\"password\" name=\"mqtt_pass\" placeholder=\"Contraseña MQTT\" required>"
"            <button type=\"submit\">Guardar y Conectar</button>"
"        </form>"
"        <p class=\"footer\">ESP32 WiFi & MQTT Setup</p>"
"    </div>"
"</body>"
"</html>";


void wifi_init_ap() {
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_AP_SSID,
            .ssid_len = strlen(WIFI_AP_SSID),
            .password = WIFI_AP_PASS,
            .max_connection = 3,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        }
    };
    
    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    esp_wifi_start();

    ESP_LOGI(TAG_WIFI, "WiFi AP creado: SSID=%s, PASS=%s", WIFI_AP_SSID, WIFI_AP_PASS);
}

void url_decode(char *src, char *dest) {
    char *p = src;
    char code[3] = {0};
    while (*p) {
        if (*p == '%' && *(p + 1) && *(p + 2)) {
            code[0] = *(p + 1);
            code[1] = *(p + 2);
            *dest++ = (char) strtol(code, NULL, 16);
            p += 3;
        } else if (*p == '+') {
            *dest++ = ' '; // '+' se convierte en espacio
            p++;
        } else {
            *dest++ = *p++;
        }
    }
    *dest = '\0';
}


 // Manejador de eventos WiFi
void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT) 
    {
        switch (event_id) 
        {
            case WIFI_EVENT_STA_START:
                ESP_LOGI(TAG_WIFI, "Intentando conectar a WiFi...");
                break;

            case WIFI_EVENT_STA_CONNECTED:
                ESP_LOGI(TAG_WIFI, "Conectado a la red WiFi, esperando IP...");
                break;

            case WIFI_EVENT_STA_DISCONNECTED:
                ESP_LOGW(TAG_WIFI, "WiFi desconectado, intentando reconectar...");
                if(client != NULL)
                {
                	esp_mqtt_client_stop(client); // Detener MQTT al perder WiFi, revisar
                }
                esp_wifi_connect();
                break;
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) 
    {
        ESP_LOGI(TAG_WIFI, "IP asignada, iniciando MQTT...");
        mqtt_app_start(); // Iniciar MQTT solo cuando ya se tenga una IP
    }
}
 
 //Cuando un usuario accede a la IP del ESP32, esta función envía la página web donde puede ingresar el SSID y la contraseña.
esp_err_t get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html_page, strlen(html_page));
    
    return ESP_OK;
}

void wifi_connect_sta() {
	
	//wifiSemaphore_AP = xSemaphoreCreateBinary(); // Crear semáforo
	nvs_handle_t nvs_handle;
    nvs_open("storage", NVS_READWRITE, &nvs_handle);
    
      char ssid[32] = {0}, pass[64] = {0};
    size_t ssid_len = sizeof(ssid), pass_len = sizeof(pass);

    nvs_get_str(nvs_handle, "ssid", ssid, &ssid_len);
    nvs_get_str(nvs_handle, "pass", pass, &pass_len);
    nvs_close(nvs_handle);
    
    //Si no hay credenciales guardadas inicia un punto de acceso (AP),para configuración
    if (strlen(ssid) == 0) {
        ESP_LOGI(TAG_WIFI, "No hay credenciales guardadas, iniciando en modo AP...");
        wifi_init_ap();
        start_webserver();
        //xSemaphoreTake(wifiSemaphore_AP, portMAX_DELAY);
        return;//termina aqui ejecución de la función
    }
    
    
    // Decodificar el SSID
 //   char decoded_ssid[32] = {0};
//    url_decode(ssid, decoded_ssid);
	ESP_LOGI(TAG_WIFI, "Intentando conectar a SSID: %s, PASS: %s", ssid, pass);
    ESP_LOGE(TAG_WIFI, "Conectando a SSID: %s", ssid);

    // 2) Create the network interface handle
    esp_netif_create_default_wifi_sta();
    //Aquí se inicializa una estructura de configuración para Wi-Fi con los valores predeterminados.
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    //Esta función inicializa la infraestructura Wi-Fi en el ESP32 usando la configuración predeterminada que creaste en el paso anterior
    esp_wifi_init(&cfg);
		wifi_config_t wifi_config = {
    .sta = {
        .ssid = {0},          // Inicialización explícita del arreglo ssid
        .password = {0},      // Inicialización explícita del arreglo password
    	},
	};

	memcpy(wifi_config.sta.ssid, ssid, strlen(ssid));
	memcpy(wifi_config.sta.password, pass, strlen(pass));
    
     esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
     // Registrar el manejador de eventos
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL);

    esp_wifi_start();
       
    // Habilitar el ahorro de energía WiFi, sin perder conexión
    esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    esp_wifi_connect();

}

//Esta función responde a solicitudes POST, leyendo los datos ingresados por el usuario y guardándolos en la memoria NVS.
esp_err_t wifi_post_handler(httpd_req_t *req) {
    char buf[200];
    int ret, remaining = req->content_len;

    while (remaining > 0) {
        if ((ret = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)))) <= 0) {
            return ESP_FAIL;
        }
        remaining -= ret;
    }

    buf[req->content_len] = '\0';

    char ssid[32], wifi_pass[64];
      char mqtt_url[100] = {0}, mqtt_user[50] = {0}, mqtt_pass[50] = {0};//*
   /* sscanf(buf, "ssid=%31[^&]&pass=%63s", ssid, pass);*/
      // Extraer datos del formulario con sscanf
    sscanf(buf, "ssid=%31[^&]&wifi_pass=%63[^&]&mqtt_url=%99[^&]&mqtt_user=%49[^&]&mqtt_pass=%49s",
           ssid, wifi_pass, mqtt_url, mqtt_user, mqtt_pass);
    
     // Decodificar cada uno de los valores
    url_decode(ssid, ssid);
    url_decode(wifi_pass, wifi_pass);
    url_decode(mqtt_url, mqtt_url);
    url_decode(mqtt_user, mqtt_user);
    url_decode(mqtt_pass, mqtt_pass);

    // Imprimir datos recibidos
    ESP_LOGI(TAG_WIFI, "SSID: %s", ssid);
    ESP_LOGI(TAG_WIFI, "WiFi PASS: %s", wifi_pass);
    ESP_LOGI(TAG_WIFI, "MQTT URL: %s", mqtt_url);
    ESP_LOGI(TAG_WIFI, "MQTT User: %s", mqtt_user);
    ESP_LOGI(TAG_WIFI, "Recibido SSID: %s, PASS: %s", ssid, wifi_pass);

    nvs_handle_t nvs_handle;
    nvs_open("storage", NVS_READWRITE, &nvs_handle);
    nvs_set_str(nvs_handle, "ssid", ssid);
    nvs_set_str(nvs_handle, "pass", wifi_pass);
    nvs_set_str(nvs_handle, "mqtt_url", mqtt_url);//
    nvs_set_str(nvs_handle, "mqtt_user", mqtt_user);//
    nvs_set_str(nvs_handle, "mqtt_pass", mqtt_pass);//
    nvs_commit(nvs_handle);
    nvs_close(nvs_handle);

    const char *resp = "Datos guardados. Reiniciando...";
    httpd_resp_send(req, resp, strlen(resp));

    vTaskDelay(pdMS_TO_TICKS(5000));
    //esp_restart();// Reiniciar ESP32 para conectar a la nueva red// wifi_connect_sta(); //
	wifi_connect_sta();
	//xSemaphoreGive(wifiSemaphore_AP); // Liberar semáforo cuando haya recibido la info para conectarse al wifi
    return ESP_OK;
}

void start_webserver() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        // Registrar el manejador para mostrar el formulario HTML
        httpd_uri_t uri_get = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = get_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &uri_get);

        // Registrar el manejador para recibir datos del formulario

        httpd_uri_t uri_config = {
            .uri = "/config",
            .method = HTTP_POST,
            .handler = wifi_post_handler, // O usa otra función si es diferente
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &uri_config);
    }
}


