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

#include "string.h"
#include "inttypes.h"
#include "audio_pipeline.h"
#include "audio_thread.h"
#include "audio_mem.h"
#include "esp_g711.h"
#include "esp_camera.h"
#include "esp_timer.h"
#include "av_stream.h"
#include "av_stream_hal.h"
#include "esp_jpeg_enc.h"
static const char *TAG = "AV_STREAM";

const int ENCODER_STOPPED_BIT = BIT0;

struct _av_stream_handle
{
    // 编码运行状态标志
    bool venc_run;
    bool aenc_run;

    // 音频位置计数器
    uint64_t audio_pos;

    // 视频编码器句柄
    void *video_enc;

    // 摄像头计数器和时间戳
    uint64_t camera_count;
    uint64_t camera_first_pts;

    // 配置结构体
    av_stream_config_t config;

    // 音频板句柄
    audio_board_handle_t board_handle;

    // 编码队列
    QueueHandle_t aenc_queue;
    QueueHandle_t venc_queue;

    // 编码状态事件组
    EventGroupHandle_t aenc_state;
    EventGroupHandle_t venc_state;

    // 音频编码管道
    audio_pipeline_handle_t audio_enc;
};

static uint32_t _time_ms()
{
    return esp_timer_get_time() / 1000;
}

static bool _have_hardware_ref(av_stream_handle_t av_stream)
{
    if (av_stream->config.hal.uac_en)
    {
        return false;
    }

#if !RECORD_HARDWARE_AEC
    return false;
#endif
    return true;
}

// JPEG编码器初始化函数
static void init_jpeg_encoder(av_stream_handle_t av_stream)
{
    jpeg_enc_config_t config = DEFAULT_JPEG_ENC_CONFIG();
    config.width = av_resolution[av_stream->config.hal.video_framesize].width;
    config.height = av_resolution[av_stream->config.hal.video_framesize].height;
    config.src_type = JPEG_PIXEL_FORMAT_YCbYCr;
    config.subsampling = JPEG_SUBSAMPLE_420;
    config.quality = 40;
    jpeg_enc_open(&config, &av_stream->video_enc);
}

// static int init_h264_encoder(av_stream_handle_t av_stream)
// {
// #if CONFIG_IDF_TARGET_ESP32S3
//     esp_h264_enc_cfg_t cfg = DEFAULT_H264_ENCODER_CONFIG();
//     cfg.pic_type = ESP_H264_RAW_FMT_YUV422;
//     cfg.width = av_resolution[av_stream->config.hal.video_framesize].width;
//     cfg.height = av_resolution[av_stream->config.hal.video_framesize].height;
//     cfg.fps = VIDEO_FPS;
//     cfg.gop_size = VIDEO_FPS * 3;
//     cfg.target_bitrate = 400000;
//     return esp_h264_enc_open(&cfg, &av_stream->video_enc);
// #else
//     return ESP_ERR_NOT_SUPPORTED;
// #endif
// }

static void _audio_enc(void *pv)
{
    av_stream_handle_t av_stream = (av_stream_handle_t)pv;
    xEventGroupClearBits(av_stream->aenc_state, ENCODER_STOPPED_BIT);

    char *frame_buf = NULL;
    int timeout = 5 / portTICK_PERIOD_MS; // 减少超时时间，提高响应速度

    // 只支持G711A，使用标准内存分配
    frame_buf = audio_calloc(1, AUDIO_MAX_SIZE);
    if (!frame_buf)
    {
        ESP_LOGE(TAG, "frame_buf is NULL");
        return;
    }

    while (av_stream->aenc_run)
    {
        // 硬件采集：从16kHz麦克风读取音频数据
        int read_len = custom_av_stream_audio_read((char *)frame_buf, av_stream->config.hal.audio_framesize, 100);
        if (read_len <= 0)
        {
            continue;
        }

        av_stream_frame_t enc;
        enc.data = audio_calloc(1, AUDIO_MAX_SIZE);
        if (!enc.data)
        {
            ESP_LOGE(TAG, "enc.data is NULL");
            break;
        }

        // 只支持G711A编码
        if (av_stream->config.acodec_type == AV_ACODEC_G711A)
        {
            int16_t *input_samples = (int16_t *)frame_buf;
            uint32_t hardware_samplerate = av_stream->config.hal.audio_samplerate; // 16kHz
            uint32_t encoder_samplerate = av_stream->config.acodec_samplerate;     // 8kHz

            int input_samples_count = read_len / 2; // 16bit samples
            int output_samples_count = 0;

            // 重采样：16kHz -> 8kHz (每2个样本取1个)
            int downsample_ratio = hardware_samplerate / encoder_samplerate; // 2
            for (int i = 0; i < input_samples_count; i += downsample_ratio)
            {
                if (output_samples_count < AUDIO_MAX_SIZE)
                {
                    enc.data[output_samples_count] = esp_g711a_encode(input_samples[i]);
                    output_samples_count++;
                }
            }

            enc.len = output_samples_count;
            av_stream->audio_pos += enc.len * 2; // 更新位置计数器
            enc.pts = (av_stream->audio_pos * 1000) / (encoder_samplerate * 1 * 16 / 8);

            // 音频处理：16kHz->8kHz重采样，然后G711A编码
        }
        else
        {
            ESP_LOGE(TAG, "Only G711A encoding is supported");
            free(enc.data);
            continue;
        }

        if (xQueueSend(av_stream->aenc_queue, &enc, timeout) != pdTRUE)
        {
            ESP_LOGD(TAG, "send aenc buf queue timeout !");
            free(enc.data);
        }
    }
    ESP_LOGI(TAG, "_audio_enc task stoped");

    // 只支持G711A，使用标准内存释放
    free(frame_buf);

    xEventGroupSetBits(av_stream->aenc_state, ENCODER_STOPPED_BIT);
    vTaskDelete(NULL);
}

int av_audio_enc_start(av_stream_handle_t av_stream)
{
    if (!av_stream)
    {
        ESP_LOGE(TAG, "av_stream is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    if (av_stream->aenc_run)
    {
        return ESP_OK;
    }

    if (av_stream->config.acodec_type != AV_ACODEC_G711A)
    {
        ESP_LOGE(TAG, "Only G711A encoding is supported, current: %d", av_stream->config.acodec_type);
        return ESP_ERR_NOT_SUPPORTED;
    }

    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    av_stream->audio_enc = audio_pipeline_init(&pipeline_cfg);
    if (!av_stream->audio_enc)
    {
        ESP_LOGE(TAG, "audio_enc is NULL");
        return ESP_FAIL;
    }

    ESP_LOGW(TAG, "Resampling disabled, using raw audio data");

    av_stream->aenc_run = true;
    av_stream->audio_pos = 0;
    audio_pipeline_run(av_stream->audio_enc);
    av_stream->aenc_queue = xQueueCreate(1, sizeof(av_stream_frame_t));
    av_stream->aenc_state = xEventGroupCreate();
    if (audio_thread_create(NULL, "_audio_enc", _audio_enc, av_stream, 4 * 1024, 22, true, 0) != ESP_OK)
    {
        ESP_LOGE(TAG, "Can not start _audio_enc task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "audio_enc started");
    return ESP_OK;
}

int av_audio_enc_stop(av_stream_handle_t av_stream)
{
    if (!av_stream)
    {
        ESP_LOGE(TAG, "av_stream is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    if (!av_stream->aenc_run)
    {
        return ESP_OK;
    }

    av_stream->aenc_run = false;
    audio_pipeline_stop(av_stream->audio_enc);
    audio_pipeline_wait_for_stop(av_stream->audio_enc);
    audio_pipeline_deinit(av_stream->audio_enc);

    xEventGroupWaitBits(av_stream->aenc_state, ENCODER_STOPPED_BIT, false, true, portMAX_DELAY);
    vEventGroupDelete(av_stream->aenc_state);

    av_stream_frame_t enc;
    if (xQueueReceive(av_stream->aenc_queue, &enc, 0) == pdTRUE)
    {
        audio_free(enc.data);
    }
    vQueueDelete(av_stream->aenc_queue);
    av_stream->aenc_queue = NULL;

    if (!_have_hardware_ref(av_stream))
    {
    }

    if (av_stream->config.acodec_type == AV_ACODEC_AAC_LC)
    {
    }
    return ESP_OK;
}

int av_audio_dec_start(av_stream_handle_t av_stream)
{
    ESP_LOGW(TAG, "Audio decoding disabled - only encoding for RTSP streaming");
    return ESP_OK;
}

int av_audio_dec_stop(av_stream_handle_t av_stream)
{

    ESP_LOGW(TAG, "Audio decoding disabled - only encoding for RTSP streaming");
    return ESP_OK;
}

int av_audio_enc_read(av_stream_frame_t *frame, void *ctx)
{
    av_stream_handle_t av_stream = (av_stream_handle_t)ctx;
    if (!av_stream)
    {
        ESP_LOGE(TAG, "av_stream is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    if (av_stream->aenc_queue == NULL)
    {
        frame->len = 0;
        return ESP_FAIL;
    }

    av_stream_frame_t enc;
    if (xQueueReceive(av_stream->aenc_queue, &enc, 0) != pdTRUE)
    {
        frame->len = 0;
        // ESP_LOGW(TAG, "Get enc buffer timeout!");
        return ESP_FAIL;
    }
    else
    {
        frame->len = enc.len;
        frame->pts = enc.pts;
        memcpy(frame->data, enc.data, enc.len);
        audio_free(enc.data);
    }

    return ESP_OK;
}

int av_audio_dec_write(av_stream_frame_t *frame, void *ctx)
{
    ESP_LOGW(TAG, "Audio decoding disabled - only encoding for RTSP streaming");
    return ESP_OK;
}

int av_audio_set_vol(av_stream_handle_t av_stream, int vol)
{
    return ESP_OK;
}

int av_audio_get_vol(av_stream_handle_t av_stream, int *vol)
{
    // 简化版本：设置默认音量，因为使用自定义音频HAL
    if (vol)
    {
        *vol = 50; // 默认音量50%
    }
    return ESP_OK;
}

static void _video_enc(void *pv)
{
    av_stream_handle_t av_stream = (av_stream_handle_t)pv;
    xEventGroupClearBits(av_stream->venc_state, ENCODER_STOPPED_BIT);
    while (av_stream->venc_run)
    {
        camera_fb_t *pic = esp_camera_fb_get();
        if (pic)
        {
            av_stream_frame_t enc;
            if (av_stream->camera_first_pts == 0)
            {
                av_stream->camera_first_pts = _time_ms();
                enc.pts = 0;
            }
            else
            {
                enc.pts = _time_ms() - av_stream->camera_first_pts;
            }

            enc.data = audio_calloc(1, VIDEO_MAX_SIZE);
            // 直接使用摄像头输出的JPEG数据，不需要额外编码
            memcpy(enc.data, pic->buf, pic->len);
            enc.len = pic->len;
            av_stream->camera_count++;
            esp_camera_fb_return(pic);
            if (xQueueSend(av_stream->venc_queue, &enc, 50 / portTICK_PERIOD_MS) != pdTRUE)
            {
                ESP_LOGD(TAG, "send enc buf queue timeout !");
                free(enc.data);
            }
        }
        else
        {
            vTaskDelay(5 / portTICK_PERIOD_MS);
        }
    }
    ESP_LOGI(TAG, "_video_enc task stoped");
    xEventGroupSetBits(av_stream->venc_state, ENCODER_STOPPED_BIT);
    vTaskDelete(NULL);
}

int av_video_enc_start(av_stream_handle_t av_stream)
{
    if (!av_stream)
    {
        ESP_LOGE(TAG, "av_stream is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    if (av_stream->venc_run)
    {
        return ESP_OK;
    }

    // 初始化摄像头计数器和时间戳
    av_stream->camera_count = 0;
    av_stream->camera_first_pts = 0;
    av_stream->venc_run = true;
    av_stream->venc_queue = xQueueCreate(1, sizeof(av_stream_frame_t));

    if (!av_stream->config.hal.uvc_en)
    {
        if (av_stream->config.hal.video_soft_enc)
        {
            if (av_stream->config.vcodec_type == AV_VCODEC_MJPEG)
            {
                init_jpeg_encoder(av_stream);
            }
            else
            {
                ESP_LOGW(TAG, "Only MJPEG encoding is supported, current: %d", av_stream->config.vcodec_type);
                return ESP_ERR_NOT_SUPPORTED;
            }
            if (!av_stream->video_enc)
            {
                ESP_LOGE(TAG, "video_enc is NULL");
                return ESP_FAIL;
            }
        }
        av_stream->venc_state = xEventGroupCreate();
        if (audio_thread_create(NULL, "_video_enc", _video_enc, av_stream, 15 * 1024, 20, true, 1) != ESP_OK)
        {
            ESP_LOGE(TAG, "Can not start _video_enc task");
            return ESP_FAIL;
        }
    }
    else
    {
        // USB相关功能已删除
    }

    return ESP_OK;
}

int av_video_enc_stop(av_stream_handle_t av_stream)
{
    if (!av_stream)
    {
        ESP_LOGE(TAG, "av_stream is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    if (!av_stream->venc_run)
    {
        return ESP_OK;
    }

    av_stream->venc_run = false;
    vTaskDelay(50 / portTICK_PERIOD_MS);

    if (!av_stream->config.hal.uvc_en)
    {
        xEventGroupWaitBits(av_stream->venc_state, ENCODER_STOPPED_BIT, false, true, portMAX_DELAY);
        vEventGroupDelete(av_stream->venc_state);
        if (av_stream->config.hal.video_soft_enc)
        {
            if (av_stream->config.vcodec_type == AV_VCODEC_MJPEG)
            {
                jpeg_enc_close(av_stream->video_enc);
            }
        }
    }
    else
    {
        // USB相关功能已删除
    }

    av_stream_frame_t enc;
    if (xQueueReceive(av_stream->venc_queue, &enc, 0) == pdTRUE)
    {
        audio_free(enc.data);
    }
    vQueueDelete(av_stream->venc_queue);
    av_stream->venc_queue = NULL;
    // 简化版本：不需要图像计数器
    return ESP_OK;
}

int av_video_enc_read(av_stream_frame_t *frame, void *ctx)
{
    av_stream_handle_t av_stream = (av_stream_handle_t)ctx;
    if (!av_stream)
    {
        ESP_LOGE(TAG, "av_stream is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    if (av_stream->venc_queue == NULL)
    {
        return ESP_FAIL;
    }

    av_stream_frame_t enc;
    if (xQueueReceive(av_stream->venc_queue, &enc, 0) != pdTRUE)
    {
        frame->len = 0;
        // ESP_LOGW(TAG, "Get enc buffer timeout!");
        return ESP_FAIL;
    }
    else
    {
        frame->len = enc.len;
        frame->pts = enc.pts;
        memcpy(frame->data, enc.data, enc.len);
        audio_free(enc.data);

        ESP_LOGD(TAG, "send video %d pts %lld !", (int)enc.len, enc.pts);
    }

    return ESP_OK;
}

av_stream_handle_t av_stream_init(av_stream_config_t *config)
{
    if (!config)
    {
        ESP_LOGE(TAG, "config is NULL");
        return NULL;
    }
    av_stream_handle_t av_stream = audio_calloc(1, sizeof(struct _av_stream_handle));
    if (!av_stream)
    {
        ESP_LOGE(TAG, "av_stream is NULL");
        return NULL;
    }
    memcpy(&av_stream->config, config, sizeof(av_stream_config_t));

    if (config->acodec_type != AV_ACODEC_NULL)
    {
        av_stream->board_handle = custom_av_stream_audio_init(NULL, NULL, &av_stream->config.hal);
    }

    if (config->vcodec_type != AV_VCODEC_NULL)
    {
        custom_av_stream_camera_init(&av_stream->config.hal, NULL, NULL);
    }
    return av_stream;
}

int av_stream_deinit(av_stream_handle_t av_stream)
{
    if (!av_stream)
    {
        ESP_LOGE(TAG, "av_stream is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    av_audio_enc_stop(av_stream);
    av_video_enc_stop(av_stream);

    if (av_stream->config.acodec_type != AV_ACODEC_NULL)
    {
        custom_av_stream_audio_deinit(av_stream->board_handle, av_stream->config.hal.uac_en);
    }

    audio_free(av_stream);
    av_stream = NULL;
    return ESP_OK;
}
