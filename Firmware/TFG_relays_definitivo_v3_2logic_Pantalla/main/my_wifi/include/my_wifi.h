#ifndef HTML_PAGE_H
#define HTML_PAGE_H

#include <stdint.h>
#include "esp_event.h"
#include "esp_http_server.h"

extern const char *html_page;

#define MIN(a, b) ((a) < (b) ? (a) : (b))

#define WIFI_AP_SSID "ESP32_Config"
#define WIFI_AP_PASS "123456789"


void wifi_connect_sta(void);
void wifi_init_ap(void);
void start_webserver(void);
void url_decode(char *src, char *dest);
void wifi_event_handler(void *ctx, esp_event_base_t event_base, int32_t event_id, void *event_data);
esp_err_t get_handler(httpd_req_t *req);
esp_err_t wifi_post_handler(httpd_req_t *req);


#endif  // HTML_PAGE_H
