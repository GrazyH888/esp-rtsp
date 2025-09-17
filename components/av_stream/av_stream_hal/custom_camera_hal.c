/*
 * Custom Camera HAL for XIAO ESP32-S3-Sense
 * 简化版本 - 直接使用esp_camera_init
 */

 #include "esp_log.h"
 #include "audio_mem.h"
 #include "av_stream_hal.h"
 #include "esp_camera.h"
 
 static const char *TAG = "CUSTOM_CAMERA_HAL";
 
 // XIAO ESP32-S3-Sense 官方摄像头引脚配置
#define CAM_PIN_PWDN    -1
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK    10
#define CAM_PIN_SIOD    40
#define CAM_PIN_SIOC    39

#define CAM_PIN_D7      48
#define CAM_PIN_D6      11
#define CAM_PIN_D5      12
#define CAM_PIN_D4      14
#define CAM_PIN_D3      16
#define CAM_PIN_D2      18
#define CAM_PIN_D1      17
#define CAM_PIN_D0      15
#define CAM_PIN_VSYNC   38
#define CAM_PIN_HREF    47
#define CAM_PIN_PCLK    13
 

const av_resolution_info_t av_resolution[AV_FRAMESIZE_INVALID] = {
    {   96,   96 }, /* 96x96 */
    {  160,  120 }, /* QQVGA */
    {  176,  144 }, /* QCIF  */
    {  240,  176 }, /* HQVGA */
    {  240,  240 }, /* 240x240 */
    {  320,  240 }, /* QVGA  */
    {  400,  296 }, /* CIF   */
    {  480,  320 }, /* HVGA  */
    {  640,  480 }, /* VGA   */
    {  800,  600 }, /* SVGA  */
    { 1024,  768 }, /* XGA   */
    { 1280,  720 }, /* HD    */
    { 1280, 1024 }, /* SXGA  */
    { 1600, 1200 }, /* UXGA  */
    // 3MP Sensors
    { 1920, 1080 }, /* FHD   */
    {  720, 1280 }, /* Portrait HD   */
    {  864, 1536 }, /* Portrait 3MP   */
    { 2048, 1536 }, /* QXGA  */
    // 5MP Sensors
    { 2560, 1440 }, /* QHD    */
    { 2560, 1600 }, /* WQXGA  */
    { 1088, 1920 }, /* Portrait FHD   */
    { 2560, 1920 }, /* QSXGA  */
};

 // 简化的摄像头配置
 static camera_config_t custom_camera_config = {
     .pin_pwdn       = CAM_PIN_PWDN,
     .pin_reset      = CAM_PIN_RESET,
     .pin_xclk       = CAM_PIN_XCLK,
     .pin_sccb_sda   = CAM_PIN_SIOD,
     .pin_sccb_scl   = CAM_PIN_SIOC,
     .pin_d7         = CAM_PIN_D7,
     .pin_d6         = CAM_PIN_D6,
     .pin_d5         = CAM_PIN_D5,
     .pin_d4         = CAM_PIN_D4,
     .pin_d3         = CAM_PIN_D3,
     .pin_d2         = CAM_PIN_D2,
     .pin_d1         = CAM_PIN_D1,
     .pin_d0         = CAM_PIN_D0,
     .pin_vsync      = CAM_PIN_VSYNC,
     .pin_href       = CAM_PIN_HREF,
     .pin_pclk       = CAM_PIN_PCLK,
     
     .xclk_freq_hz   = 20000000,        // 20MHz
     .ledc_channel   = LEDC_CHANNEL_0,
     .ledc_timer     = LEDC_TIMER_0,
     
     .pixel_format   = PIXFORMAT_JPEG,
     .frame_size     = FRAMESIZE_HVGA,  // 640x480
     .jpeg_quality   = 20,  // 降低质量，提高流畅度
     .fb_count       = 2,
     .grab_mode      = CAMERA_GRAB_WHEN_EMPTY,
 };
 
 int custom_av_stream_camera_init(av_stream_hal_config_t *config, void *cb, void *arg)
 {
     ESP_LOGI(TAG, "Initializing custom camera for XIAO ESP32-S3-Sense");
     
     // 直接调用esp_camera_init，不需要手动I2C初始化
     esp_err_t err = esp_camera_init(&custom_camera_config);
     if (err != ESP_OK) {
         ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
         return ESP_FAIL;
     }
     
     // 在摄像头初始化成功后，直接设置翻转
     ESP_LOGI(TAG, "Setting camera flip settings...");
     sensor_t *sensor = esp_camera_sensor_get();
     if (sensor) {
         
         // 垂直翻转（上下翻转）
         if (sensor->set_vflip) {
             sensor->set_vflip(sensor, 1);    // 1=开启垂直翻转
         }
         
         // 如果需要只翻转其中一个方向，可以注释掉另一个：
         // sensor->set_hmirror(sensor, 1);  // 只水平翻转
         // sensor->set_vflip(sensor, 0);    // 不垂直翻转
     } else {
         ESP_LOGW(TAG, "Camera sensor not available for flip settings");
     }
     
     ESP_LOGI(TAG, "Custom camera initialized successfully");
     return ESP_OK;
 }
 
 int custom_av_stream_camera_deinit(bool uvc_en)
 {
     ESP_LOGI(TAG, "Deinitializing custom camera");
     
     // 直接调用esp_camera_deinit
     esp_err_t err = esp_camera_deinit();
     if (err != ESP_OK) {
         ESP_LOGE(TAG, "Camera deinit failed with error 0x%x", err);
         return ESP_FAIL;
     }
     
     ESP_LOGI(TAG, "Custom camera deinitialized successfully");
     return ESP_OK;
 }