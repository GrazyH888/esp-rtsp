/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2023 <ESPRESSIF SYSTEMS (SHANGHAI) CO., LTD>
 *
 * Permission is hereby granted for use on all ESPRESSIF SYSTEMS products, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */
#ifndef _AV_STREAM_HAL_H
#define _AV_STREAM_HAL_H

#include "esp_log.h"
#include "driver/i2s.h"
#include "driver/i2s_pdm.h"
// #include "esp_peripherals.h"  // 不再需要，直接使用ESP-IDF WiFi API
// #include "i2s_stream.h"  // 注释掉，使用自定义音频HAL
// #include "board.h"  // 注释掉，audio_board组件已删除
// #include "esp_lcd_panel_ops.h"  // 注释掉，不需要LCD功能

// 自定义audio_board_handle_t类型定义
typedef void* audio_board_handle_t;

#ifdef __cplusplus
extern "C" {
#endif

#define VIDEO_FPS                   15
#define VIDEO_DEC_SIZE              480*320*2
#define VIDEO_MAX_SIZE              100*1024
#define AUDIO_MAX_SIZE              4*1024  // 增大音频缓冲区，减少丢包
#define AUDIO_HAL_SAMPLE_RATE       16000

#define I2S_DEFAULT_PORT            I2S_NUM_0
#define I2S_DEFAULT_BITS            CODEC_ADC_BITS_PER_SAMPLE

#if CONFIG_ESP32_S3_KORVO2L_V1_BOARD // ES8311 need ADCL + DACR for AEC feature
#define I2S_CHANNELS                I2S_CHANNEL_FMT_RIGHT_LEFT
#else
#define I2S_CHANNELS                I2S_CHANNEL_FMT_ONLY_LEFT
#endif

#define USB_CAMERA_FRAME_INTERVAL(fps) ((10000000) / fps)

typedef enum {
    AV_FRAMESIZE_96X96,    // 96x96
    AV_FRAMESIZE_QQVGA,    // 160x120
    AV_FRAMESIZE_QCIF,     // 176x144
    AV_FRAMESIZE_HQVGA,    // 240x176
    AV_FRAMESIZE_240X240,  // 240x240
    AV_FRAMESIZE_QVGA,     // 320x240
    AV_FRAMESIZE_CIF,      // 400x296
    AV_FRAMESIZE_HVGA,     // 480x320
    AV_FRAMESIZE_VGA,      // 640x480
    AV_FRAMESIZE_SVGA,     // 800x600
    AV_FRAMESIZE_XGA,      // 1024x768
    AV_FRAMESIZE_HD,       // 1280x720
    AV_FRAMESIZE_SXGA,     // 1280x1024
    AV_FRAMESIZE_UXGA,     // 1600x1200
    // 3MP Sensors
    AV_FRAMESIZE_FHD,      // 1920x1080
    AV_FRAMESIZE_P_HD,     //  720x1280
    AV_FRAMESIZE_P_3MP,    //  864x1536
    AV_FRAMESIZE_QXGA,     // 2048x1536
    // 5MP Sensors
    AV_FRAMESIZE_QHD,      // 2560x1440
    AV_FRAMESIZE_WQXGA,    // 2560x1600
    AV_FRAMESIZE_P_FHD,    // 1080x1920
    AV_FRAMESIZE_QSXGA,    // 2560x1920
    AV_FRAMESIZE_INVALID
} av_framesize_t;

typedef struct {
    const uint16_t width;
    const uint16_t height;
} av_resolution_info_t;

extern const av_resolution_info_t av_resolution[];

/**
 * @brief AV stream hal configurations
 */
typedef struct {
    bool                        uvc_en;             /*!< Use USB video camera, but user need check uvc MACROS */
    bool                        uac_en;             /*!< Use USB audio device, but user need check uac MACROS */
    // bool                        lcd_en;             /*!< Have LCD to render */  // 注释掉，不需要LCD功能
    void*                       set;                /*!< The handle of esp_periph - no longer used */
    bool                        video_soft_enc;     /*!< Use software JPEG / H264 encoder */
    uint32_t                    audio_samplerate;   /*!< Audio sample rate */
    uint32_t                    audio_framesize;    /*!< Audio frame size */
    av_framesize_t              video_framesize;    /*!< Video frame size */
} av_stream_hal_config_t;


esp_err_t custom_i2s_mic_init(uint32_t sample_rate, uint8_t bit_depth, uint8_t channels, uint32_t buffer_size);

/**
 * @brief      Deinitialize I2S PDM microphone
 *
 * @return
 *     - ESP_OK on success
 *     - ESP_FAIL on error
 */
esp_err_t custom_i2s_mic_deinit(void);

/**
 * @brief      Set microphone gain
 *
 * @param[in]  gain_db        Gain value (1=original volume, 2=2x volume, etc.)
 *
 * @return
 *     - ESP_OK on success
 *     - ESP_FAIL on error
 */
esp_err_t custom_i2s_mic_set_gain(int gain_db);

/**
 * @brief      Start I2S PDM microphone recording
 *
 * @return
 *     - ESP_OK on success
 *     - ESP_FAIL on error
 */
esp_err_t custom_i2s_mic_start(void);

/**
 * @brief      Stop I2S PDM microphone recording
 *
 * @return
 *     - ESP_OK on success
 *     - ESP_FAIL on error
 */
esp_err_t custom_i2s_mic_stop(void);

/**
 * @brief      Read audio data from I2S PDM microphone
 *
 * @param[in]  buf            Audio data buffer
 * @param[in]  len            Number of bytes to read
 * @param[in]  wait_time      Wait time (FreeRTOS ticks)
 *
 * @return
 *     - Number of bytes read on success
 *     - ESP_FAIL on error
 */
int custom_av_stream_audio_read(char *buf, int len, TickType_t wait_time);

/**
 * @brief      Write audio data to output (XIAO ESP32-S3-Sense has no speaker, returns 0)
 *
 * @param[in]  buf            Audio data buffer
 * @param[in]  len            Number of bytes to write
 * @param[in]  wait_time      Wait time (FreeRTOS ticks)
 *
 * @return
 *     - 0 (no speaker output)
 */
int custom_av_stream_audio_write(char *buf, int len, TickType_t wait_time);

/**
 * @brief      Initialize custom audio HAL
 *
 * @param[in]  ctx            Context pointer
 * @param[in]  arg            Argument pointer
 * @param[in]  config         Audio HAL configuration
 *
 * @return
 *     - Audio board handle on success
 *     - NULL on failure
 */
audio_board_handle_t custom_av_stream_audio_init(void *ctx, void *arg, av_stream_hal_config_t *config);

/**
 * @brief      Deinitialize custom audio HAL
 *
 * @param[in]  ctx            Context pointer
 * @param[in]  uac_en         UAC enable flag
 *
 * @return
 *     - ESP_OK on success
 *     - Other value on failure
 */
int custom_av_stream_audio_deinit(void *ctx, bool uac_en);

// ============================================================================
// Custom Camera HAL Functions (XIAO ESP32-S3-Sense Camera)
// ============================================================================

/**
 * @brief      Initialize custom camera for XIAO ESP32-S3-Sense
 *
 * @param[in]  config         HAL configuration
 * @param[in]  cb             Camera callback function
 * @param[in]  arg            Callback argument
 *
 * @return
 *     - ESP_OK on success
 *     - ESP_FAIL on failure
 */
int custom_av_stream_camera_init(av_stream_hal_config_t *config, void *cb, void *arg);

/**
 * @brief      Deinitialize custom camera
 *
 * @param[in]  uvc_en         Whether UVC is enabled
 *
 * @return
 *     - ESP_OK on success
 *     - ESP_FAIL on failure
 */
int custom_av_stream_camera_deinit(bool uvc_en);

/*
 * LCD相关函数已注释掉，因为不需要LCD功能
 *
 * esp_lcd_panel_handle_t av_stream_lcd_init(esp_periph_set_handle_t set);
 * int av_stream_lcd_deinit(esp_lcd_panel_handle_t panel_handle);
 * int av_stream_lcd_render(esp_lcd_panel_handle_t panel_handle, unsigned char *data);
 */

#ifdef __cplusplus
}
#endif

#endif