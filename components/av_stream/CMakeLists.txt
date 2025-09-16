
set(COMPONENT_SRCDIRS . av_stream_hal)
set(COMPONENT_SRCS 
    "av_stream.c"
    "av_stream_hal/custom_audio_hal.c"
    "av_stream_hal/custom_camera_hal.c"
)

set(COMPONENT_ADD_INCLUDEDIRS . av_stream_hal)

set(COMPONENT_REQUIRES 
    esp-adf-libs 
    esp_timer 
    driver 
    esp_wifi
    audio_pipeline
    esp32-camera
)

# if(IDF_TARGET STREQUAL "esp32s3")
#     list(APPEND COMPONENT_REQUIRES usb_stream)
# endif()

register_component()
