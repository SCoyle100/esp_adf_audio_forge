

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/uart.h"


#include "audio_element.h"
#include "audio_pipeline.h"
#include "audio_event_iface.h"
#include "audio_common.h"
#include "i2s_stream.h"
#include "raw_stream.h"
#include "fatfs_stream.h"
#include "wav_decoder.h"
#include "audio_forge.h"
#include "board.h"
#include "esp_peripherals.h"
#include "periph_sdcard.h"

#define ECHO_TEST_TXD (GPIO_NUM_1) // Correct TXD0 pin on ESP32 LyraT Mini
#define ECHO_TEST_RXD (GPIO_NUM_3) // Correct RXD0 pin on ESP32 LyraT Mini
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

#define ECHO_UART_PORT_NUM      (1)
#define ECHO_UART_BAUD_RATE     (9600)
#define ECHO_TASK_STACK_SIZE    (2048)

#define BUF_SIZE (1024)
#define TRIGGER_CHAR 'V'

static const char *TAG = "AUDIO_FORGE_PIPELINE";

#define DEFAULT_SAMPLERATE 44100
#define DEFAULT_CHANNEL 1
#define DEST_SAMPLERATE 44100
#define DEST_CHANNEL 1
#define TRANSMITTIME 0
#define MUSIC_GAIN_DB 0
#define NUMBER_SOURCE_FILE 2

audio_pipeline_handle_t pipeline[NUMBER_SOURCE_FILE] = {NULL};
audio_element_handle_t fats_rd_el[NUMBER_SOURCE_FILE] = {NULL};
audio_element_handle_t wav_decoder[NUMBER_SOURCE_FILE] = {NULL};
audio_element_handle_t el_raw_write[NUMBER_SOURCE_FILE] = {NULL};

audio_pipeline_handle_t pipeline_mix;
audio_element_handle_t audio_forge;

int audio_forge_wr_cb(audio_element_handle_t el, char *buf, int len, TickType_t wait_time, void *ctx)
{
    audio_element_handle_t i2s_wr = (audio_element_handle_t)ctx;
    int ret = audio_element_output(i2s_wr, buf, len);
    return ret;
}

void init_audio_pipeline()
{
    ESP_LOGI(TAG, "[1.0] Start audio codec chip");
    audio_board_handle_t board_handle = audio_board_init();
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_DECODE, AUDIO_HAL_CTRL_START);

    ESP_LOGI(TAG, "[2.0] Start and wait for SDCARD to mount");
    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);
    audio_board_sdcard_init(set, SD_MODE_1_LINE);

    ESP_LOGI(TAG, "[3.0] Create pipeline_mix to mix");
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    pipeline_mix = audio_pipeline_init(&pipeline_cfg);

    ESP_LOGI(TAG, "[3.1] Create audio_forge");
    audio_forge_cfg_t audio_forge_cfg = AUDIO_FORGE_CFG_DEFAULT();
    audio_forge_cfg.audio_forge.component_select = AUDIO_FORGE_SELECT_DOWNMIX;
    audio_forge_cfg.audio_forge.dest_samplerate = DEST_SAMPLERATE;
    audio_forge_cfg.audio_forge.dest_channel = DEST_CHANNEL;
    audio_forge_cfg.audio_forge.source_num = NUMBER_SOURCE_FILE;
    audio_forge = audio_forge_init(&audio_forge_cfg);
    audio_forge_src_info_t source_information = {
        .samplerate = DEFAULT_SAMPLERATE,
        .channel = DEFAULT_CHANNEL,
    };

    audio_forge_downmix_t downmix_information = {
        .gain = {0, MUSIC_GAIN_DB},
        .transit_time = TRANSMITTIME,
    };
    audio_forge_src_info_t source_info[NUMBER_SOURCE_FILE] = {0};
    audio_forge_downmix_t downmix_info[NUMBER_SOURCE_FILE];
    for (int i = 0; i < NUMBER_SOURCE_FILE; i++)
    {
        source_info[i] = source_information;
        downmix_info[i] = downmix_information;
    }
    audio_forge_source_info_init(audio_forge, source_info, downmix_info);

    ESP_LOGI(TAG, "[3.2] Create i2s stream to read audio data from codec chip");
    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg.type = AUDIO_STREAM_WRITER;
    i2s_cfg.task_stack = 0;
    i2s_cfg.out_rb_size = 0;
    audio_element_handle_t i2s_writer = i2s_stream_init(&i2s_cfg);
    i2s_stream_set_clk(i2s_writer, DEST_SAMPLERATE, 16, DEST_CHANNEL);

    ESP_LOGI(TAG, "[3.3] Link elements together audio_forge-->i2s_writer");
    audio_pipeline_register(pipeline_mix, audio_forge, "audio_forge");
    audio_element_set_write_cb(audio_forge, audio_forge_wr_cb, i2s_writer);
    audio_element_process_init(i2s_writer);

    ESP_LOGI(TAG, "[3.4] Link elements together audio_forge-->i2s_stream-->[codec_chip]");
    audio_pipeline_link(pipeline_mix, (const char *[]){"audio_forge"}, 1);

    ESP_LOGI(TAG, "[4.0] Create Fatfs stream to read input data");
    fatfs_stream_cfg_t fatfs_cfg = FATFS_STREAM_CFG_DEFAULT();
    fatfs_cfg.type = AUDIO_STREAM_READER;

    ESP_LOGI(TAG, "[4.1] Create wav decoder to decode wav file");
    wav_decoder_cfg_t wav_cfg = DEFAULT_WAV_DECODER_CONFIG();
    wav_cfg.task_core = 0;

    ESP_LOGI(TAG, "[4.2] Create raw stream of base wav to write data");
    raw_stream_cfg_t raw_cfg = RAW_STREAM_CFG_DEFAULT();
    raw_cfg.type = AUDIO_STREAM_WRITER;

    ESP_LOGI(TAG, "[5.0] Set up event listener");
    audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
    audio_event_iface_handle_t evt = audio_event_iface_init(&evt_cfg);

    // Setup pipeline for continuous music playback
    pipeline[0] = audio_pipeline_init(&pipeline_cfg);
    fats_rd_el[0] = fatfs_stream_init(&fatfs_cfg);
    audio_element_set_uri(fats_rd_el[0], "/sdcard/music/music1.wav");
    wav_decoder[0] = wav_decoder_init(&wav_cfg);
    el_raw_write[0] = raw_stream_init(&raw_cfg);
    audio_pipeline_register(pipeline[0], fats_rd_el[0], "file");
    audio_pipeline_register(pipeline[0], wav_decoder[0], "wav");
    audio_pipeline_register(pipeline[0], el_raw_write[0], "raw");

    const char *link_tag[3] = {"file", "wav", "raw"};
    audio_pipeline_link(pipeline[0], &link_tag[0], 3);
    ringbuf_handle_t rb0 = audio_element_get_input_ringbuf(el_raw_write[0]);
    audio_element_set_multi_input_ringbuf(audio_forge, rb0, 0);
    audio_pipeline_set_listener(pipeline[0], evt);

    // Setup pipeline for triggered sound effect
    pipeline[1] = audio_pipeline_init(&pipeline_cfg);
    fats_rd_el[1] = fatfs_stream_init(&fatfs_cfg);
    audio_element_set_uri(fats_rd_el[1], "/sdcard/effect/effect1.wav");
    wav_decoder[1] = wav_decoder_init(&wav_cfg);
    el_raw_write[1] = raw_stream_init(&raw_cfg);
    audio_pipeline_register(pipeline[1], fats_rd_el[1], "file");
    audio_pipeline_register(pipeline[1], wav_decoder[1], "wav");
    audio_pipeline_register(pipeline[1], el_raw_write[1], "raw");

    audio_pipeline_link(pipeline[1], &link_tag[0], 3);
    ringbuf_handle_t rb1 = audio_element_get_input_ringbuf(el_raw_write[1]);
    audio_element_set_multi_input_ringbuf(audio_forge, rb1, 1);
    audio_pipeline_set_listener(pipeline[1], evt);

    audio_pipeline_set_listener(pipeline_mix, evt);
    ESP_LOGI(TAG, "[5.1] Listening event from peripherals");
    audio_event_iface_set_listener(esp_periph_set_get_event_iface(set), evt);

    audio_pipeline_run(pipeline[0]);
    audio_pipeline_run(pipeline_mix);
}

void uart_task(void *arg)
{
    uart_config_t uart_config = {
        .baud_rate = ECHO_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));

    uint8_t *data = (uint8_t *)malloc(BUF_SIZE);

    while (1)
    {
        int len = uart_read_bytes(ECHO_UART_PORT_NUM, data, (BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
        uart_write_bytes(ECHO_UART_PORT_NUM, (const char *)data, len);
        if (len)
        {
            data[len] = '\0';
            ESP_LOGI(TAG, "Recv str: %s", (char *)data);
            if (strchr((char *)data, TRIGGER_CHAR))
            {
                ESP_LOGI(TAG, "Trigger character detected");
                audio_pipeline_stop(pipeline[1]);
                audio_pipeline_wait_for_stop(pipeline[1]);
                audio_pipeline_reset_ringbuffer(pipeline[1]);
                audio_pipeline_reset_elements(pipeline[1]);
                audio_pipeline_run(pipeline[1]);
            }
        }
    }
}

void app_main(void)
{
    init_audio_pipeline();
    xTaskCreate(uart_task, "uart_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
}







