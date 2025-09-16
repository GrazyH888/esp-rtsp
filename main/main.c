/* RTSP Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "string.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "audio_mem.h"
#include "rtsp_service.h"
#include "av_stream_hal.h"

#define TAG         "ESP_RTSP"

// 修改WiFi配置
// #define WIFI_SSID  "GrazyH"
// #define WIFI_PWD   "123456789"
// #define WIFI_SSID  "Skyris2"
#define WIFI_SSID  "skyris_debug"
#define WIFI_PWD   "skyrisai"

static av_stream_handle_t av_stream;

// WiFi事件处理函数
void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "WiFi disconnected, retrying...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "WiFi connected! IP: " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "RTSP URL: rtsp://" IPSTR ":8554/live", IP2STR(&event->ip_info.ip));
    }
}

void setup_wifi(void)
{
    // 初始化网络和事件
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));
    
    // 创建WiFi STA接口
    esp_netif_create_default_wifi_sta();
    
    // 初始化WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    
    // 配置WiFi
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PWD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {.capable = true, .required = false},
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "WiFi initialization completed");
}

void app_main()
{
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("AUDIO_ELEMENT", ESP_LOG_ERROR);
    esp_log_level_set("AFE_VC", ESP_LOG_ERROR);
    AUDIO_MEM_SHOW(TAG);

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {

        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_LOGI(TAG, "[ 1 ] Initialize WiFi");
    setup_wifi();

    ESP_LOGI(TAG, "[ 2 ] Initialize av stream");
    av_stream_config_t av_stream_config = {
        .enable_aec = false,  // 关闭AEC，简化音频处理
        .acodec_samplerate = 8000,   // 8kHz编码采样率，与G711A兼容
        .acodec_type = AV_ACODEC_G711A,  // 使用G711A编码，与RTSP协议兼容
        .vcodec_type = AV_VCODEC_MJPEG,
        .hal = {
            .uac_en = false,
            .uvc_en = false,
            .video_soft_enc = false,
            .audio_samplerate = 16000,    // 8kHz硬件采样率，避免重采样
            .audio_framesize = 480,       
            .video_framesize = AV_FRAMESIZE_HVGA,
        },
    };
    av_stream = av_stream_init(&av_stream_config);
    
    // 模式0:RTSP服务器
    rtsp_service_start(av_stream, 0, NULL);
}
