

// trying out 12/14/2024 - maybe this will work 

/*
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


audio_event_iface_handle_t evt;


#define ECHO_TEST_TXD (GPIO_NUM_1) // Correct TXD0 pin on ESP32 LyraT Mini
#define ECHO_TEST_RXD (GPIO_NUM_3) // Correct RXD0 pin on ESP32 LyraT Mini
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

#define ECHO_UART_PORT_NUM      (1)
#define ECHO_UART_BAUD_RATE     (9600)
#define ECHO_TASK_STACK_SIZE    (4096)

#define BUF_SIZE (1024)
#define TRIGGER_CHAR 'V'
#define SWITCH_CHAR 'S'

static const char *TAG = "AUDIO_FORGE_PIPELINE";

#define DEFAULT_SAMPLERATE 44100
#define DEFAULT_CHANNEL 1
#define DEST_SAMPLERATE 44100
#define DEST_CHANNEL 1
#define TRANSMITTIME 0
#define MUSIC_GAIN_DB 12
#define NUMBER_SOURCE_FILE 2

audio_event_iface_handle_t music_evt;
audio_event_iface_handle_t effects_evt;


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



void restart_pipeline(audio_pipeline_handle_t pipeline, audio_element_handle_t element, const char *uri_template, int index)
{
    char uri[30];
    snprintf(uri, sizeof(uri), uri_template, index);

    // Stop the pipeline
    audio_pipeline_stop(pipeline);
    audio_pipeline_wait_for_stop(pipeline);
    audio_pipeline_reset_ringbuffer(pipeline);
    audio_pipeline_reset_elements(pipeline);

    // Set the new URI and start the pipeline
    audio_element_set_uri(element, uri);
    audio_pipeline_run(pipeline);
}


void init_audio_pipeline()
{
    ESP_LOGI(TAG, "[1.0] Start audio codec chip");
    audio_board_handle_t board_handle = audio_board_init();
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_DECODE, AUDIO_HAL_CTRL_START);

    ESP_LOGI(TAG, "[2.0] Start and wait for SDCARD to mount");
    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);  // Initialize 'set'
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

    ESP_LOGI(TAG, "[5.0] Set up event listeners");
    // Move the declaration here
    audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();

    // Initialize event listeners
    evt = audio_event_iface_init(&evt_cfg);
    music_evt = audio_event_iface_init(&evt_cfg);
    effects_evt = audio_event_iface_init(&evt_cfg);

    // Assign event listeners to pipelines
    audio_pipeline_set_listener(pipeline[0], music_evt);
    audio_pipeline_set_listener(pipeline[1], effects_evt);
    audio_pipeline_set_listener(pipeline_mix, evt);

    audio_event_iface_set_listener(esp_periph_set_get_event_iface(set), evt);

    ESP_LOGI(TAG, "[5.1] Listening event from peripherals");

    // Setup pipeline for continuous music playback
    pipeline[0] = audio_pipeline_init(&pipeline_cfg);
    fats_rd_el[0] = fatfs_stream_init(&fatfs_cfg);
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

    // Setup pipeline for sound effects or different audio on trigger
    pipeline[1] = audio_pipeline_init(&pipeline_cfg);
    fats_rd_el[1] = fatfs_stream_init(&fatfs_cfg);
    wav_decoder[1] = wav_decoder_init(&wav_cfg);
    el_raw_write[1] = raw_stream_init(&raw_cfg);
    audio_pipeline_register(pipeline[1], fats_rd_el[1], "file");
    audio_pipeline_register(pipeline[1], wav_decoder[1], "wav");
    audio_pipeline_register(pipeline[1], el_raw_write[1], "raw");

    audio_pipeline_link(pipeline[1], &link_tag[0], 3);
    ringbuf_handle_t rb1 = audio_element_get_input_ringbuf(el_raw_write[1]);
    audio_element_set_multi_input_ringbuf(audio_forge, rb1, 1);
    audio_pipeline_set_listener(pipeline[1], evt);

    ESP_LOGI(TAG, "[6.0] Starting audio pipeline mix");
    audio_pipeline_run(pipeline_mix);
}



int current_effect_index = -1;  // Global or static variable to store the current effect index



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
            
            // If a number between 1 and 9 is received, change the music file and set the effect index
            if (data[0] >= '1' && data[0] <= '9')
            {
                int music_index = data[0] - '0';
                current_effect_index = music_index;  // Set the effect index based on the received number
                ESP_LOGI(TAG, "Playing music: /sdcard/music/music%d.wav and setting effect%d.wav for V trigger", music_index, music_index);

                // Restart the music pipeline with a new file
                restart_pipeline(pipeline[0], fats_rd_el[0], "/sdcard/music/music%d.wav", music_index);
            }

            // If the trigger character is detected, play the stored effect file
            if (strchr((char *)data, TRIGGER_CHAR) && current_effect_index != -1)
            {
                ESP_LOGI(TAG, "Trigger character detected, playing effect: /sdcard/effect/effect%d.wav", current_effect_index);

                // Restart the effect pipeline with the new effect file
                restart_pipeline(pipeline[1], fats_rd_el[1], "/sdcard/effect/effect%d.wav", current_effect_index);
            }
        }

        // Handle music pipeline events
        audio_event_iface_msg_t msg_music;
        if (audio_event_iface_listen(music_evt, &msg_music, 0) == ESP_OK) {
            if (msg_music.source == (void *)wav_decoder[0] && msg_music.cmd == AEL_MSG_CMD_REPORT_STATUS && msg_music.data == (void *)AEL_STATUS_STATE_FINISHED) {
                ESP_LOGI(TAG, "Music finished, restarting music pipeline...");
                audio_pipeline_stop(pipeline[0]);
                audio_pipeline_wait_for_stop(pipeline[0]);
                audio_pipeline_reset_ringbuffer(pipeline[0]);
                audio_pipeline_reset_elements(pipeline[0]);
                audio_pipeline_run(pipeline[0]);
            }
        }

        // Handle effects pipeline events (if needed, e.g., when an effect finishes)
        audio_event_iface_msg_t msg_effects;
        if (audio_event_iface_listen(effects_evt, &msg_effects, 0) == ESP_OK) {
            // Handle effect-specific events if required
        }
    }
}


void app_main(void)
{
    init_audio_pipeline();
    xTaskCreate(uart_task, "uart_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
}

*/



//MAYBE USE THIS ONE - THIS ONE DOESN'T HAVE THE GIF INDEXING.....MAYBE DELETE THIS ONE
/*
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include <sys/stat.h>



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
#define LIMIT_SWITCH_CHAR 'S'

static const char *TAG = "AUDIO_FORGE_PIPELINE";

#define DEFAULT_SAMPLERATE 44100
#define DEFAULT_CHANNEL 1
#define DEST_SAMPLERATE 44100
#define DEST_CHANNEL 1
#define TRANSMITTIME 0
#define MUSIC_GAIN_DB 0
#define NUMBER_SOURCE_FILE 14
#define NUMBER_OF_CASES 3

audio_pipeline_handle_t music_pipeline[NUMBER_OF_CASES] = {NULL};
audio_pipeline_handle_t effects_pipeline;
audio_element_handle_t fats_rd_el[NUMBER_SOURCE_FILE * NUMBER_OF_CASES + 1] = {NULL}; // +1 for effects
audio_element_handle_t wav_decoder[NUMBER_SOURCE_FILE * NUMBER_OF_CASES + 1] = {NULL}; // +1 for effects
audio_element_handle_t el_raw_write[NUMBER_SOURCE_FILE * NUMBER_OF_CASES + 1] = {NULL}; // +1 for effects

audio_pipeline_handle_t pipeline_mix;
audio_element_handle_t audio_forge;

int audio_forge_wr_cb(audio_element_handle_t el, char *buf, int len, TickType_t wait_time, void *ctx)
{
    audio_element_handle_t i2s_wr = (audio_element_handle_t)ctx;
    int ret = audio_element_output(i2s_wr, buf, len);
    return ret;
}

void play_audio_by_index(int index)
{
    char uri[40];
    snprintf(uri, sizeof(uri), "/sdcard/effect/effect%d.wav", index);
    audio_pipeline_stop(effects_pipeline); // Stop the pipeline for sound effects
    audio_pipeline_wait_for_stop(effects_pipeline);
    audio_pipeline_reset_ringbuffer(effects_pipeline);
    audio_pipeline_reset_elements(effects_pipeline);
    audio_element_set_uri(fats_rd_el[NUMBER_SOURCE_FILE * NUMBER_OF_CASES], uri); // Set the URI for the sound effect
    audio_pipeline_run(effects_pipeline);
}

bool file_exists(const char *path)
{
    struct stat buffer;
    return (stat(path, &buffer) == 0);
}

void play_music_by_index(int index)
{
    char uri[40];
    snprintf(uri, sizeof(uri), "/sdcard/music/music%d.wav", index);

    if (file_exists(uri)) {
        audio_pipeline_stop(music_pipeline[index]); // Stop the current music pipeline
        audio_pipeline_wait_for_stop(music_pipeline[index]);
        audio_pipeline_reset_ringbuffer(music_pipeline[index]);
        audio_pipeline_reset_elements(music_pipeline[index]);
        audio_element_set_uri(fats_rd_el[index * 2], uri); // Set the URI for the music
        audio_pipeline_run(music_pipeline[index]);
    } else {
        ESP_LOGI(TAG, "Music file for case %d does not exist, skipping.", index);
    }
}




void init_audio_pipeline()
{

    const char *link_tag[3] = {"file", "wav", "raw"};

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

    for (int i = 0; i < NUMBER_OF_CASES; i++)
    {
        // Setup pipeline for continuous music playback for each case
        music_pipeline[i] = audio_pipeline_init(&pipeline_cfg);
        fats_rd_el[i * 2] = fatfs_stream_init(&fatfs_cfg);
        wav_decoder[i * 2] = wav_decoder_init(&wav_cfg);
        el_raw_write[i * 2] = raw_stream_init(&raw_cfg);
        audio_pipeline_register(music_pipeline[i], fats_rd_el[i * 2], "file");
        audio_pipeline_register(music_pipeline[i], wav_decoder[i * 2], "wav");
        audio_pipeline_register(music_pipeline[i], el_raw_write[i * 2], "raw");

        const char *link_tag[3] = {"file", "wav", "raw"};
        audio_pipeline_link(music_pipeline[i], &link_tag[0], 3);
        ringbuf_handle_t rb0 = audio_element_get_input_ringbuf(el_raw_write[i * 2]);
        audio_element_set_multi_input_ringbuf(audio_forge, rb0, 0);
        audio_pipeline_set_listener(music_pipeline[i], evt);
    }

    // Setup pipeline for sound effects
    effects_pipeline = audio_pipeline_init(&pipeline_cfg);
    fats_rd_el[NUMBER_SOURCE_FILE * NUMBER_OF_CASES] = fatfs_stream_init(&fatfs_cfg);
    wav_decoder[NUMBER_SOURCE_FILE * NUMBER_OF_CASES] = wav_decoder_init(&wav_cfg);
    el_raw_write[NUMBER_SOURCE_FILE * NUMBER_OF_CASES] = raw_stream_init(&raw_cfg);
    audio_pipeline_register(effects_pipeline, fats_rd_el[NUMBER_SOURCE_FILE * NUMBER_OF_CASES], "file");
    audio_pipeline_register(effects_pipeline, wav_decoder[NUMBER_SOURCE_FILE * NUMBER_OF_CASES], "wav");
    audio_pipeline_register(effects_pipeline, el_raw_write[NUMBER_SOURCE_FILE * NUMBER_OF_CASES], "raw");

    audio_pipeline_link(effects_pipeline, &link_tag[0], 3);
    ringbuf_handle_t rb1 = audio_element_get_input_ringbuf(el_raw_write[NUMBER_SOURCE_FILE * NUMBER_OF_CASES]);
    audio_element_set_multi_input_ringbuf(audio_forge, rb1, 1);
    audio_pipeline_set_listener(effects_pipeline, evt);

    audio_pipeline_set_listener(pipeline_mix, evt);
    ESP_LOGI(TAG, "[5.1] Listening event from peripherals");
    audio_event_iface_set_listener(esp_periph_set_get_event_iface(set), evt);
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
    int currentGame = 0; // Initialize current game to 0
    int lastGame = -1;   // Variable to keep track of the last game

    while (1)
    {
        int len = uart_read_bytes(ECHO_UART_PORT_NUM, data, (BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
        uart_write_bytes(ECHO_UART_PORT_NUM, (const char *)data, len);
        if (len)
        {
            data[len] = '\0';
            ESP_LOGI(TAG, "Recv str: %s", (char *)data);

            // Check if the received data is a numeric value
            bool isNumeric = true;
            for (int i = 0; i < len && data[i] != '\0'; i++)
            {
                if (data[i] < '0' || data[i] > '9')
                {
                    isNumeric = false;
                    break;
                }
            }

            if (isNumeric)
            {
                int receivedNumber = atoi((char *)data);

                if (receivedNumber >= 0 && receivedNumber < NUMBER_OF_CASES)
                {
                    if (currentGame != receivedNumber)
                    {
                        lastGame = currentGame;
                        currentGame = receivedNumber; // Switch game
                        ESP_LOGI(TAG, "Switch to game: %d", currentGame);
                        audio_pipeline_stop(music_pipeline[lastGame]);
                        audio_pipeline_wait_for_stop(music_pipeline[lastGame]);
                        audio_pipeline_reset_ringbuffer(music_pipeline[lastGame]);
                        audio_pipeline_reset_elements(music_pipeline[lastGame]);
                        play_music_by_index(currentGame); // Start the music for the new game
                    }
                }

                // Handle trigger and limit switch characters within the current game
                switch (currentGame)
                {
                    case 0:
                        if (strchr((char *)data, TRIGGER_CHAR))
                        {
                            play_audio_by_index(0);
                        }
                        if (strchr((char *)data, LIMIT_SWITCH_CHAR))
                        {
                            play_audio_by_index(1);
                        }
                        break;
                    case 1:
                        if (strchr((char *)data, TRIGGER_CHAR))
                        {
                            play_audio_by_index(2);
                        }
                        if (strchr((char *)data, LIMIT_SWITCH_CHAR))
                        {
                            play_audio_by_index(3);
                        }
                        break;
                    case 2:
                        if (strchr((char *)data, TRIGGER_CHAR))
                        {
                            play_audio_by_index(4);
                        }
                        if (strchr((char *)data, LIMIT_SWITCH_CHAR))
                        {
                            play_audio_by_index(5);
                        }
                        break;

                        
                    case 3:
                        if (strchr((char *)data, TRIGGER_CHAR))
                        {
                            play_audio_by_index(6);
                        }
                        if (strchr((char *)data, LIMIT_SWITCH_CHAR))
                        {
                            play_audio_by_index(7);
                        }
                        break;
                    case 4:
                        if (strchr((char *)data, TRIGGER_CHAR))
                        {
                            play_audio_by_index(8);
                        }
                        if (strchr((char *)data, LIMIT_SWITCH_CHAR))
                        {
                            play_audio_by_index(9);
                        }
                        break;
                    case 5:
                        if (strchr((char *)data, TRIGGER_CHAR))
                        {
                            play_audio_by_index(10);
                        }
                        if (strchr((char *)data, LIMIT_SWITCH_CHAR))
                        {
                            play_audio_by_index(11);
                        }
                        break;
                    case 6:
                        if (strchr((char *)data, TRIGGER_CHAR))
                        {
                            play_audio_by_index(12);
                        }
                        if (strchr((char *)data, LIMIT_SWITCH_CHAR))
                        {
                            play_audio_by_index(13);
                        }
                        break;
                    case 7:
                        if (strchr((char *)data, TRIGGER_CHAR))
                        {
                            play_audio_by_index(14);
                        }
                        if (strchr((char *)data, LIMIT_SWITCH_CHAR))
                        {
                            play_audio_by_index(15);
                        }
                        break;
                    case 8:
                        if (strchr((char *)data, TRIGGER_CHAR))
                        {
                            play_audio_by_index(16);
                        }
                        if (strchr((char *)data, LIMIT_SWITCH_CHAR))
                        {
                            play_audio_by_index(17);
                        }
                        break;
                    case 9:
                        if (strchr((char *)data, TRIGGER_CHAR))
                        {
                            play_audio_by_index(18);
                        }
                        if (strchr((char *)data, LIMIT_SWITCH_CHAR))
                        {
                            play_audio_by_index(19);
                        }
                        break;
                    case 10:
                        if (strchr((char *)data, TRIGGER_CHAR))
                        {
                            play_audio_by_index(20);
                        }
                        if (strchr((char *)data, LIMIT_SWITCH_CHAR))
                        {
                            play_audio_by_index(21);
                        }
                        break;
                    case 11:
                        if (strchr((char *)data, TRIGGER_CHAR))
                        {
                            play_audio_by_index(22);
                        }
                        if (strchr((char *)data, LIMIT_SWITCH_CHAR))
                        {
                            play_audio_by_index(23);
                        }
                        break;
                    case 12:
                        if (strchr((char *)data, TRIGGER_CHAR))
                        {
                            play_audio_by_index(24);
                        }
                        if (strchr((char *)data, LIMIT_SWITCH_CHAR))
                        {
                            play_audio_by_index(25);
                        }
                        break;
                    case 13:
                        if (strchr((char *)data, TRIGGER_CHAR))
                        {
                            play_audio_by_index(26);
                        }
                        if (strchr((char *)data, LIMIT_SWITCH_CHAR))
                        {
                            play_audio_by_index(27);
                        }
                        break;

                        
                    // Add more cases as needed
                }
            } // Close the isNumeric block
        } // Close the len check
    } // Close the while loop
} // Close the function


void app_main(void)
{
    init_audio_pipeline();
    xTaskCreate(uart_task, "uart_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
}

*/







//LAST ONE WE DID TRYING TO CHANGE MUSIC AND SOUND EFFECTS - WORKED BUT DIDN'T WORK - 12/13/2024

//IT FREAKING WORKS! MUSIC LOOPS! SOUND EFFECTS TRIGGER! - 12/15/2024
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


audio_event_iface_handle_t evt;


#define ECHO_TEST_TXD (GPIO_NUM_1) // Correct TXD0 pin on ESP32 LyraT Mini
#define ECHO_TEST_RXD (GPIO_NUM_3) // Correct RXD0 pin on ESP32 LyraT Mini
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

#define ECHO_UART_PORT_NUM      (1)
#define ECHO_UART_BAUD_RATE     (9600)
#define ECHO_TASK_STACK_SIZE    (4096)

#define BUF_SIZE (1024)
#define TRIGGER_CHAR 'V'
#define SWITCH_CHAR 'S'

static const char *TAG = "AUDIO_FORGE_PIPELINE";

#define DEFAULT_SAMPLERATE 44100
#define DEFAULT_CHANNEL 1
#define DEST_SAMPLERATE 44100
#define DEST_CHANNEL 1
#define TRANSMITTIME 0
#define MUSIC_GAIN_DB 1
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
    esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);  // Initialize 'set'
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
    //audio_event_iface_handle_t evt = audio_event_iface_init(&evt_cfg);
    evt = audio_event_iface_init(&evt_cfg);

    // Setup pipeline for continuous music playback
    pipeline[0] = audio_pipeline_init(&pipeline_cfg);
    fats_rd_el[0] = fatfs_stream_init(&fatfs_cfg);
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

    // Setup pipeline for sound effects or different audio on trigger
    pipeline[1] = audio_pipeline_init(&pipeline_cfg);
    fats_rd_el[1] = fatfs_stream_init(&fatfs_cfg);
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
    audio_event_iface_set_listener(esp_periph_set_get_event_iface(set), evt);  // Use 'set' here

    // Start with playing the first music file
    //audio_element_set_uri(fats_rd_el[0], "/sdcard/music/music1.wav");  this was probably causing the issue
    
    //audio_pipeline_run(pipeline[0]); // this was probably causing the issue as well
    audio_pipeline_run(pipeline_mix);
}


int current_effect_index = -1;  // Global or static variable to store the current effect index



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
            
            // If a number between 1 and 9 is received, change the music file and set the effect index
            if (data[0] >= '1' && data[0] <= '9')
            {
                int music_index = data[0] - '0';
                current_effect_index = music_index;  // Set the effect index based on the received number
                ESP_LOGI(TAG, "Playing music: /sdcard/music/music%d.wav and setting effect%d.wav for V trigger", music_index, music_index);

                // Stop current music pipeline
                audio_pipeline_stop(pipeline[0]);
                audio_pipeline_wait_for_stop(pipeline[0]);
                audio_pipeline_reset_ringbuffer(pipeline[0]);
                audio_pipeline_reset_elements(pipeline[0]);

                // Change the music file
                char music_uri[30];
                snprintf(music_uri, sizeof(music_uri), "/sdcard/music/music%d.wav", music_index);
                audio_element_set_uri(fats_rd_el[0], music_uri);

                // Restart the music pipeline with the new file
                audio_pipeline_run(pipeline[0]);
            }

            // If the trigger character is detected, play the stored effect file
            if (strchr((char *)data, TRIGGER_CHAR) && current_effect_index != -1)
            {
                ESP_LOGI(TAG, "Trigger character detected, playing effect: /sdcard/effect/effect%d.wav", current_effect_index);

                // Stop current effect pipeline
                audio_pipeline_stop(pipeline[1]);
                audio_pipeline_wait_for_stop(pipeline[1]);
                audio_pipeline_reset_ringbuffer(pipeline[1]);
                audio_pipeline_reset_elements(pipeline[1]);

                // Change the effect file
                char effect_uri[30];
                snprintf(effect_uri, sizeof(effect_uri), "/sdcard/effect/effect%d.wav", current_effect_index);
                audio_element_set_uri(fats_rd_el[1], effect_uri);

                // Restart the effect pipeline with the new file
                audio_pipeline_run(pipeline[1]);
            }
        }

        /*
        // Check if the music file has finished playing
        audio_event_iface_msg_t msg;
        esp_err_t ret = audio_event_iface_listen(evt, &msg, 0);
        if (ret == ESP_OK && msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && msg.source == (void *)wav_decoder[0] && msg.cmd == AEL_MSG_CMD_REPORT_STATUS && msg.data == (void *)AEL_STATUS_STATE_FINISHED)
        {
            // Instead of restarting the pipeline, retrigger the URI for looping
            ESP_LOGI(TAG, "Music finished, retriggering URI for looping...");
            audio_element_reset_state(wav_decoder[0]);  // Reset the decoder state
            audio_element_reset_input_ringbuf(fats_rd_el[0]);  // Reset the ringbuffer
            audio_element_set_uri(fats_rd_el[0], audio_element_get_uri(fats_rd_el[0]));  // Retrigger the same URI
            audio_pipeline_run(pipeline[0]);  // Continue the pipeline
        }

        */


       // Check if the music file has finished playing
        audio_event_iface_msg_t msg;
        esp_err_t ret = audio_event_iface_listen(evt, &msg, 0);
        if (ret == ESP_OK && msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT &&
        msg.source == (void *)wav_decoder[0] &&
        msg.cmd == AEL_MSG_CMD_REPORT_STATUS &&
        msg.data == (void *)AEL_STATUS_STATE_FINISHED)
        {
            ESP_LOGI(TAG, "Music finished. Restarting for loop...");

        // Properly stop the pipeline
            audio_pipeline_stop(pipeline[0]);
            audio_pipeline_wait_for_stop(pipeline[0]);

    // Reset the pipeline elements and buffers
            audio_pipeline_reset_ringbuffer(pipeline[0]);
            audio_pipeline_reset_elements(pipeline[0]);

    // Restart the pipeline to loop the track
            audio_pipeline_run(pipeline[0]);
        }

    }
}

void app_main(void)
{
    init_audio_pipeline();
    xTaskCreate(uart_task, "uart_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
}







//CURRENT TEST 8/23/2024 - only plays music once, but the music isn't getting all caught up and stopping out.

/*
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


audio_event_iface_handle_t evt;


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
    esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);  // Initialize 'set'
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

    // Setup pipeline for sound effects or different audio on trigger
    pipeline[1] = audio_pipeline_init(&pipeline_cfg);
    fats_rd_el[1] = fatfs_stream_init(&fatfs_cfg);
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
    audio_event_iface_set_listener(esp_periph_set_get_event_iface(set), evt);  // Use 'set' here

    // Start with playing the first music file
    audio_element_set_uri(fats_rd_el[0], "/sdcard/music/music1.wav");
    
    audio_pipeline_run(pipeline[0]); // Start the music pipeline
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
            
            // Play the first file in the effect directory when trigger character is detected
            if (strchr((char *)data, TRIGGER_CHAR))
            {
                ESP_LOGI(TAG, "Trigger character detected");

                // Stop current effect pipeline
                audio_pipeline_stop(pipeline[1]);
                audio_pipeline_wait_for_stop(pipeline[1]);
                audio_pipeline_reset_ringbuffer(pipeline[1]);
                audio_pipeline_reset_elements(pipeline[1]);

                // Play the first effect file
                audio_element_set_uri(fats_rd_el[1], "/sdcard/effect/effect1.wav");
                audio_pipeline_run(pipeline[1]);
            }

            // If a number between 1 and 9 is received, change the music file
            if (data[0] >= '1' && data[0] <= '9')
            {
                int music_index = data[0] - '0';
                ESP_LOGI(TAG, "Playing music: /sdcard/music/music%d.wav", music_index);

                // Stop current music pipeline
                audio_pipeline_stop(pipeline[0]);
                audio_pipeline_wait_for_stop(pipeline[0]);
                audio_pipeline_reset_ringbuffer(pipeline[0]);
                audio_pipeline_reset_elements(pipeline[0]);

                // Change the music file
                char music_uri[30];
                snprintf(music_uri, sizeof(music_uri), "/sdcard/music/music%d.wav", music_index);
                audio_element_set_uri(fats_rd_el[0], music_uri);

                // Restart the music pipeline with the new file
                audio_pipeline_run(pipeline[0]);
            }
        }

        // Check if the music file has finished playing
        audio_event_iface_msg_t msg;
        esp_err_t ret = audio_event_iface_listen(evt, &msg, 0);
        if (ret == ESP_OK && msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && msg.source == (void *)wav_decoder[0] && msg.cmd == AEL_MSG_CMD_REPORT_STATUS && msg.data == (void *)AEL_STATUS_STATE_FINISHED)
        {
            // Restart the pipeline to loop the music
            ESP_LOGI(TAG, "Music finished, restarting pipeline...");
            audio_pipeline_stop(pipeline[0]);
            audio_pipeline_wait_for_stop(pipeline[0]);
            audio_pipeline_reset_ringbuffer(pipeline[0]);  // Reset the ringbuffer for seamless looping
            audio_pipeline_reset_elements(pipeline[0]);  // Reset the elements
            audio_pipeline_run(pipeline[0]);  // Restart the pipeline
        }
    }
}



void app_main(void)
{
    init_audio_pipeline();
    xTaskCreate(uart_task, "uart_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
}
*/



/*
void app_main(void)
{
    init_audio_pipeline();
    
    // Simplified loop check without UART task
    while (1) {
        audio_event_iface_msg_t msg;
        esp_err_t ret = audio_event_iface_listen(evt, &msg, portMAX_DELAY);
        if (ret == ESP_OK && msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && msg.source == (void *)wav_decoder[0] &&
            msg.cmd == AEL_MSG_CMD_REPORT_STATUS && msg.data == (void *)AEL_STATUS_STATE_FINISHED) {
            ESP_LOGI(TAG, "Music finished, restarting pipeline...");
            audio_pipeline_stop(pipeline[0]);
            audio_pipeline_wait_for_stop(pipeline[0]);
            audio_pipeline_reset_ringbuffer(pipeline[0]);
            audio_pipeline_reset_elements(pipeline[0]);
            audio_pipeline_run(pipeline[0]);
        }
    }
}
*/









//TESTING CONTINUOUS PLAYBACK - Working, but the music isn't looping and is acting strange

/*
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


audio_event_iface_handle_t evt;


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

    // Setup pipeline for sound effects or different audio on trigger
    pipeline[1] = audio_pipeline_init(&pipeline_cfg);
    fats_rd_el[1] = fatfs_stream_init(&fatfs_cfg);
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

    // Start with playing the first music file
    audio_element_set_uri(fats_rd_el[0], "/sdcard/music/music1.wav");
    

    audio_pipeline_run(pipeline[0]); // Start the music pipeline
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
            
            // Play the first file in the effect directory when trigger character is detected
            if (strchr((char *)data, TRIGGER_CHAR))
            {
                ESP_LOGI(TAG, "Trigger character detected");

                // Stop current effect pipeline
                audio_pipeline_stop(pipeline[1]);
                audio_pipeline_wait_for_stop(pipeline[1]);
                audio_pipeline_reset_ringbuffer(pipeline[1]);
                audio_pipeline_reset_elements(pipeline[1]);

                // Play the first effect file
                audio_element_set_uri(fats_rd_el[1], "/sdcard/effect/effect1.wav");
                audio_pipeline_run(pipeline[1]);
            }

            // If a number between 1 and 9 is received, change the music file
            if (data[0] >= '1' && data[0] <= '9')
            {
                int music_index = data[0] - '0';
                ESP_LOGI(TAG, "Playing music: /sdcard/music/music%d.wav", music_index);

                // Stop current music pipeline
                audio_pipeline_stop(pipeline[0]);
                audio_pipeline_wait_for_stop(pipeline[0]);
                audio_pipeline_reset_ringbuffer(pipeline[0]);
                audio_pipeline_reset_elements(pipeline[0]);

                // Change the music file
                char music_uri[30];
                snprintf(music_uri, sizeof(music_uri), "/sdcard/music/music%d.wav", music_index);
                audio_element_set_uri(fats_rd_el[0], music_uri);

                // Restart the music pipeline with the new file
                audio_pipeline_run(pipeline[0]);
            }
        }

        // Check if the music file has finished playing
        audio_event_iface_msg_t msg;
        esp_err_t ret = audio_event_iface_listen(evt, &msg, 0);
        if (ret == ESP_OK && msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && msg.source == (void *)wav_decoder[0] && msg.cmd == AEL_MSG_CMD_REPORT_STATUS && ((int)msg.data == AEL_STATUS_STATE_FINISHED))
        {
            // Restart the pipeline to loop the music
            ESP_LOGI(TAG, "Music finished, restarting pipeline...");
            audio_pipeline_stop(pipeline[0]);
            audio_pipeline_wait_for_stop(pipeline[0]);
            audio_pipeline_run(pipeline[0]);
        }
    }
}


void app_main(void)
{
    init_audio_pipeline();
    xTaskCreate(uart_task, "uart_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
}
*/







// WORKING SIMULTANEOUS MUSIC AND SOUND EFFECTS - doesnt loop - 12/13/2024

/*
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

*/









// sensor only triggers sound once, music plays simultaneously though  - 12/13/2024

/*
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
#define MUSIC_GAIN_DB 8
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

*/









//******************Works for sound effects only! 12/13/2024***********************

/*

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "audio_element.h"
#include "audio_pipeline.h"
#include "audio_event_iface.h"
#include "i2s_stream.h"
#include "wav_decoder.h"
#include "fatfs_stream.h"
#include "board.h"

#define ECHO_TEST_TXD (GPIO_NUM_1) // Correct TXD0 pin on ESP32 LyraT Mini
#define ECHO_TEST_RXD (GPIO_NUM_3) // Correct RXD0 pin on ESP32 LyraT Mini
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

#define ECHO_UART_PORT_NUM      (1)
#define ECHO_UART_BAUD_RATE     (9600)
#define ECHO_TASK_STACK_SIZE    (2048)

#define BUF_SIZE (1024)
#define TRIGGER_CHAR 'V'
#define DEBOUNCE_DELAY 500 // Debounce delay in milliseconds

#define DEFAULT_SAMPLERATE 44100
#define DEFAULT_CHANNEL 1  // Mono
#define DEST_SAMPLERATE 44100
#define DEST_CHANNEL 1  // Mono
#define TRANSMITTIME 0
#define MUSIC_GAIN_DB 12
#define NUMBER_SOURCE_FILE 2

static const char *TAG = "UART_AUDIO_TEST";

audio_pipeline_handle_t pipeline;
audio_element_handle_t fatfs_stream_reader, wav_decoder, i2s_stream_writer;

void init_audio_pipeline() {
    ESP_LOGI(TAG, "[1.0] Start audio codec chip");
    audio_board_handle_t board_handle = audio_board_init();
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_DECODE, AUDIO_HAL_CTRL_START);

    ESP_LOGI(TAG, "[2.0] Start and wait for SDCARD to mount");
    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);
    audio_board_sdcard_init(set, SD_MODE_1_LINE);

    ESP_LOGI(TAG, "[3.0] Create audio pipeline");
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    pipeline = audio_pipeline_init(&pipeline_cfg);

    ESP_LOGI(TAG, "[3.1] Create fatfs stream to read input data");
    fatfs_stream_cfg_t fatfs_cfg = FATFS_STREAM_CFG_DEFAULT();
    fatfs_cfg.type = AUDIO_STREAM_READER;
    fatfs_stream_reader = fatfs_stream_init(&fatfs_cfg);

    ESP_LOGI(TAG, "[3.2] Create wav decoder to decode wav file");
    wav_decoder_cfg_t wav_cfg = DEFAULT_WAV_DECODER_CONFIG();
    wav_decoder = wav_decoder_init(&wav_cfg);

    ESP_LOGI(TAG, "[3.3] Create i2s stream to write data to codec chip");
    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg.type = AUDIO_STREAM_WRITER;
    i2s_stream_writer = i2s_stream_init(&i2s_cfg);
    i2s_stream_set_clk(i2s_stream_writer, DEST_SAMPLERATE, 16, DEST_CHANNEL);

    ESP_LOGI(TAG, "[3.4] Register all elements to audio pipeline");
    audio_pipeline_register(pipeline, fatfs_stream_reader, "fatfs");
    audio_pipeline_register(pipeline, wav_decoder, "wav");
    audio_pipeline_register(pipeline, i2s_stream_writer, "i2s");

    ESP_LOGI(TAG, "[3.5] Link it together [sdcard]-->fatfs_stream-->wav_decoder-->i2s_stream-->[codec_chip]");
    const char *link_tag[3] = {"fatfs", "wav", "i2s"};
    audio_pipeline_link(pipeline, &link_tag[0], 3);
}

void play_effect() {
    ESP_LOGI(TAG, "Playing 8_bit_sound.wav");
    audio_element_set_uri(fatfs_stream_reader, "/sdcard/effect/effect1.wav");
    audio_pipeline_stop(pipeline);
    audio_pipeline_wait_for_stop(pipeline);
    audio_pipeline_reset_ringbuffer(pipeline);
    audio_pipeline_reset_elements(pipeline);
    audio_pipeline_run(pipeline);
}

static void echo_task(void *arg)
{
    uart_config_t uart_config = {
        .baud_rate = ECHO_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
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

    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

    while (1) {
        int len = uart_read_bytes(ECHO_UART_PORT_NUM, data, (BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
        uart_write_bytes(ECHO_UART_PORT_NUM, (const char *) data, len);
        if (len) {
            data[len] = '\0';
            ESP_LOGI(TAG, "Recv str: %s", (char *) data);
            if (strchr((char *)data, TRIGGER_CHAR)) {
                ESP_LOGI(TAG, "Trigger character detected");
                play_effect();
            }
        }
    }
}

void app_main(void)
{
    init_audio_pipeline();
    // Test the audio pipeline
    ESP_LOGI(TAG, "Testing audio pipeline by playing sound on startup");
    play_effect();
    vTaskDelay(5000 / portTICK_PERIOD_MS); // Wait for 5 seconds to ensure the sound plays

    xTaskCreate(echo_task, "uart_echo_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
}

*/









/* WORKING AUDIO START UP - DOESN'T NEED TO PRESS BUTTON SO THAT IS GOOD, BUT UART ISN'T WORKING
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
#include "periph_button.h"

static const char *TAG = "AUDIO_FORGE_PIPELINE";

#define DEFAULT_SAMPLERATE 44100
#define DEFAULT_CHANNEL 1  // Mono
#define DEST_SAMPLERATE 44100
#define DEST_CHANNEL 1  // Mono
#define TRANSMITTIME 0
#define MUSIC_GAIN_DB 0
#define NUMBER_SOURCE_FILE 2

#define UART_NUM UART_NUM_1
#define BUF_SIZE (1024)
#define TRIGGER_CHAR 'V'
#define DEBOUNCE_DELAY 500 // Debounce delay in milliseconds

audio_pipeline_handle_t pipeline[NUMBER_SOURCE_FILE] = {NULL};
audio_element_handle_t fats_rd_el[NUMBER_SOURCE_FILE] = {NULL};
audio_element_handle_t wav_decoder[NUMBER_SOURCE_FILE] = {NULL};
audio_element_handle_t el_raw_write[NUMBER_SOURCE_FILE] = {NULL};
audio_element_handle_t audio_forge = NULL; // Ensure this is declared globally

unsigned long lastDebounceTime = 0; // Timestamp of the last command received

QueueHandle_t uart0_queue; // Declare the UART queue handle

int audio_forge_wr_cb(audio_element_handle_t el, char *buf, int len, TickType_t wait_time, void *ctx)
{
    audio_element_handle_t i2s_wr = (audio_element_handle_t)ctx;
    int ret = audio_element_output(i2s_wr, buf, len);
    return ret;
}

void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t* dtmp = (uint8_t*) malloc(BUF_SIZE);

    while (1) {
        // Waiting for UART event.
        if (xQueueReceive(uart0_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            bzero(dtmp, BUF_SIZE);
            ESP_LOGI(TAG, "UART event received");
            switch (event.type) {
                // Event of UART receiving data
                case UART_DATA:
                    uart_read_bytes(UART_NUM, dtmp, event.size, portMAX_DELAY);
                    ESP_LOGI(TAG, "Received UART data: %s", dtmp);
                    unsigned long currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    if (dtmp[0] == TRIGGER_CHAR && (currentTime - lastDebounceTime > DEBOUNCE_DELAY)) {
                        ESP_LOGI(TAG, "Trigger character received, playing second sound");
                        // Run the second pipeline when the trigger character is received
                        audio_pipeline_run(pipeline[1]);
                        audio_forge_downmix_set_input_rb_timeout(audio_forge, 50, 1); // Provide the correct arguments
                        lastDebounceTime = currentTime; // Update debounce timestamp
                    }
                    break;
                case UART_FIFO_OVF:
                    ESP_LOGW(TAG, "UART FIFO overflow");
                    uart_flush_input(UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                case UART_BUFFER_FULL:
                    ESP_LOGW(TAG, "UART buffer full");
                    uart_flush_input(UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                case UART_PARITY_ERR:
                    ESP_LOGE(TAG, "UART parity error");
                    break;
                case UART_FRAME_ERR:
                    ESP_LOGE(TAG, "UART frame error");
                    break;
                default:
                    ESP_LOGW(TAG, "Unknown UART event type: %d", event.type);
                    break;
            }
        }
    }
    free(dtmp);
    vTaskDelete(NULL);
}

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(TAG, ESP_LOG_INFO);

    ESP_LOGI(TAG, "[1.0] Start audio codec chip");
    audio_board_handle_t board_handle = audio_board_init();
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_DECODE, AUDIO_HAL_CTRL_START);

    ESP_LOGI(TAG, "[2.0] Start and wait for SDCARD to mount");
    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);
    audio_board_sdcard_init(set, SD_MODE_1_LINE);
    audio_board_key_init(set);

    ESP_LOGI(TAG, "[3.0] Create pipeline_mix to mix");
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    audio_pipeline_handle_t pipeline_mix = audio_pipeline_init(&pipeline_cfg);

    ESP_LOGI(TAG, "[3.1] Create audio_forge");
    audio_forge_cfg_t audio_forge_cfg = AUDIO_FORGE_CFG_DEFAULT();
    audio_forge_cfg.audio_forge.component_select = AUDIO_FORGE_SELECT_DOWNMIX;
    audio_forge_cfg.audio_forge.dest_samplerate = DEST_SAMPLERATE;
    audio_forge_cfg.audio_forge.dest_channel = DEST_CHANNEL;
    audio_forge_cfg.audio_forge.source_num = NUMBER_SOURCE_FILE;
    audio_forge = audio_forge_init(&audio_forge_cfg); // Ensure this is initialized globally
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
    for (int i = 0; i < NUMBER_SOURCE_FILE; i++) {
        source_info[i] = source_information;
        downmix_info[i] = downmix_information;
    }
    audio_forge_source_info_init(audio_forge, source_info, downmix_info);

    ESP_LOGI(TAG, "[3.2] Create i2s stream to read audio data from codec chip");
    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg.type = AUDIO_STREAM_WRITER;
    audio_element_handle_t i2s_writer = i2s_stream_init(&i2s_cfg);
    i2s_stream_set_clk(i2s_writer, DEST_SAMPLERATE, 16, DEST_CHANNEL);

    ESP_LOGI(TAG, "[3.3] Link elements together audio_forge-->i2s_writer");
    audio_pipeline_register(pipeline_mix, audio_forge, "audio_forge");
    audio_element_set_write_cb(audio_forge, audio_forge_wr_cb, i2s_writer);
    audio_element_process_init(i2s_writer);

    ESP_LOGI(TAG, "[3.4] Link elements together audio_forge-->i2s_stream-->[codec_chip]");
    audio_pipeline_link(pipeline_mix, (const char *[]) {"audio_forge"}, 1);

    ESP_LOGI(TAG, "[4.0] Create Fatfs stream to read input data");
    fatfs_stream_cfg_t fatfs_cfg = FATFS_STREAM_CFG_DEFAULT();
    fatfs_cfg.type = AUDIO_STREAM_READER;

    ESP_LOGI(TAG, "[4.1] Create wav decoder to decode wav file");
    wav_decoder_cfg_t wav_cfg = DEFAULT_WAV_DECODER_CONFIG();

    ESP_LOGI(TAG, "[4.2] Create raw stream of base wav to write data");
    raw_stream_cfg_t raw_cfg = RAW_STREAM_CFG_DEFAULT();
    raw_cfg.type = AUDIO_STREAM_WRITER;

    ESP_LOGI(TAG, "[5.0] Set up  event listener");
    audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
    audio_event_iface_handle_t evt = audio_event_iface_init(&evt_cfg);

    const char *file_names[NUMBER_SOURCE_FILE] = {"/sdcard/music.wav", "/sdcard/8_bit_sound.wav"};
    for (int i = 0; i < NUMBER_SOURCE_FILE; i++) {
        pipeline[i] = audio_pipeline_init(&pipeline_cfg);
        mem_assert(pipeline[i]);
        fats_rd_el[i] = fatfs_stream_init(&fatfs_cfg);
        audio_element_set_uri(fats_rd_el[i], file_names[i]);
        wav_decoder[i] = wav_decoder_init(&wav_cfg);
        el_raw_write[i] = raw_stream_init(&raw_cfg);
        audio_pipeline_register(pipeline[i], fats_rd_el[i], "file");
        audio_pipeline_register(pipeline[i], wav_decoder[i], "wav");
        audio_pipeline_register(pipeline[i], el_raw_write[i], "raw");

        const char *link_tag[3] = {"file", "wav", "raw"};
        audio_pipeline_link(pipeline[i], &link_tag[0], 3);
        ringbuf_handle_t rb = audio_element_get_input_ringbuf(el_raw_write[i]);
        audio_element_set_multi_input_ringbuf(audio_forge, rb, i);
        audio_pipeline_set_listener(pipeline[i], evt);
    }
    audio_pipeline_set_listener(pipeline_mix, evt);
    ESP_LOGI(TAG, "[5.1] Listening event from peripherals");
    audio_event_iface_set_listener(esp_periph_set_get_event_iface(set), evt);

    audio_pipeline_run(pipeline[0]);
    audio_pipeline_run(pipeline_mix);

    // Configure UART
    const uart_config_t uart_config = {
        .baud_rate = 9600, // Set to 9600 as per your requirement
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);

    ESP_LOGI(TAG, "Starting UART event task");
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 10, NULL);

    while (1) {
        audio_event_iface_msg_t msg;
        esp_err_t ret = audio_event_iface_listen(evt, &msg, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[ * ] Event interface error : %d", ret);
            continue;
        }

        for (int i = 0; i < NUMBER_SOURCE_FILE; i++) {
            if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && msg.source == (void *)wav_decoder[i]
                && msg.cmd == AEL_MSG_CMD_REPORT_MUSIC_INFO) {
                audio_element_info_t music_info = {0};
                audio_element_getinfo(wav_decoder[i], &music_info);
                ESP_LOGW(TAG, "[ * ] Receive music info from wav decoder, sample_rates=%d, bits=%d, ch=%d",
                         music_info.sample_rates, music_info.bits, music_info.channels);
                audio_forge_src_info_t src_info = {
                    .samplerate = music_info.sample_rates,
                    .channel = music_info.channels
                };
                audio_forge_set_src_info(audio_forge, src_info, i);
            }
        }

        //Stop when the last pipeline element (fatfs_writer in this case) receives stop event 
        if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT
            && msg.source == (void *)audio_forge
            && msg.cmd == AEL_MSG_CMD_REPORT_STATUS
            && (((int)msg.data == AEL_STATUS_STATE_STOPPED)
                || ((int)msg.data == AEL_STATUS_STATE_FINISHED))) {
            break;
        }
    }

    ESP_LOGI(TAG, "[6.0] Stop pipelines");
    for (int i = 0; i < NUMBER_SOURCE_FILE; i++) {
        audio_pipeline_stop(pipeline[i]);
        audio_pipeline_wait_for_stop(pipeline[i]);
        audio_pipeline_terminate(pipeline[i]);
        audio_pipeline_unregister_more(pipeline[i], fats_rd_el[i], wav_decoder[i], el_raw_write[i], NULL);
        audio_pipeline_remove_listener(pipeline[i]);
        //Release resources 
        audio_pipeline_deinit(pipeline[i]);
        audio_element_deinit(fats_rd_el[i]);
        audio_element_deinit(wav_decoder[i]);
        audio_element_deinit(el_raw_write[i]);
    }
    audio_pipeline_stop(pipeline_mix);
    audio_pipeline_wait_for_stop(pipeline_mix);
    audio_pipeline_terminate(pipeline_mix);
    audio_pipeline_unregister_more(pipeline_mix, audio_forge, NULL);
    audio_pipeline_remove_listener(pipeline_mix);

    //Stop all peripherals before removing the listener 
    esp_periph_set_stop_all(set);
    audio_event_iface_remove_listener(esp_periph_set_get_event_iface(set), evt);

    //Make sure audio_pipeline_remove_listener & audio_event_iface_remove_listener are called before destroying event_iface 
    audio_event_iface_destroy(evt);

    //Release resources 
    audio_pipeline_deinit(pipeline_mix);
    audio_element_deinit(audio_forge);
    audio_element_deinit(i2s_writer);
    esp_periph_set_destroy(set);
}

*/




















/* WORKING UART TEST SCRIPT
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"

#define ECHO_TEST_TXD (GPIO_NUM_1) // Correct TXD0 pin on ESP32 LyraT Mini
#define ECHO_TEST_RXD (GPIO_NUM_3) // Correct RXD0 pin on ESP32 LyraT Mini
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

#define ECHO_UART_PORT_NUM      (1)
#define ECHO_UART_BAUD_RATE     (9600)
#define ECHO_TASK_STACK_SIZE    (2048)

static const char *TAG = "UART TEST";

#define BUF_SIZE (1024)

static void echo_task(void *arg)
{
    uart_config_t uart_config = {
        .baud_rate = ECHO_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
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

    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

    while (1) {
        int len = uart_read_bytes(ECHO_UART_PORT_NUM, data, (BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
        uart_write_bytes(ECHO_UART_PORT_NUM, (const char *) data, len);
        if (len) {
            data[len] = '\0';
            ESP_LOGI(TAG, "Recv str: %s", (char *) data);
        }
    }
}

void app_main(void)
{
    xTaskCreate(echo_task, "uart_echo_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
}
*/





/*  - Just incase - This script works for looping, but not for the UART
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "audio_pipeline.h"
#include "audio_event_iface.h"
#include "board.h"
#include "audio_element.h"
#include "i2s_stream.h"
#include "fatfs_stream.h"
#include "wav_decoder.h"
#include "esp_peripherals.h"
#include "periph_sdcard.h"
#include "driver/uart.h"

#define TAG "AUDIO_FORGE_PIPELINE"
#define DEFAULT_SAMPLERATE 44100
#define DEFAULT_CHANNEL 1
#define DEST_SAMPLERATE 44100
#define DEST_CHANNEL 1
#define UART_NUM UART_NUM_2
#define BUF_SIZE (1024)

QueueHandle_t uart0_queue;
audio_pipeline_handle_t music_pipeline;
audio_pipeline_handle_t effect_pipeline;
audio_element_handle_t fats_rd_el_music;
audio_element_handle_t fats_rd_el_effect;

void reset_pipeline(audio_pipeline_handle_t pipeline) {
    audio_pipeline_stop(pipeline);
    audio_pipeline_wait_for_stop(pipeline);
    audio_pipeline_reset_ringbuffer(pipeline);
    audio_pipeline_reset_elements(pipeline);
    audio_pipeline_run(pipeline);
}

void play_music()
{
    ESP_LOGI(TAG, "Playing music.wav");
    audio_element_set_uri(fats_rd_el_music, "/sdcard/music.wav");
    reset_pipeline(music_pipeline);
}

void play_effect()
{
    ESP_LOGI(TAG, "Playing 8_bit_sound.wav");
    audio_element_set_uri(fats_rd_el_effect, "/sdcard/8_bit_sound.wav");
    reset_pipeline(effect_pipeline);
}

void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t *dtmp = (uint8_t *)malloc(BUF_SIZE);
    while (1) {
        if (xQueueReceive(uart0_queue, (void *)&event, (portTickType)portMAX_DELAY)) {
            memset(dtmp, 0, BUF_SIZE);
            switch (event.type) {
                case UART_DATA:
                    uart_read_bytes(UART_NUM, dtmp, event.size, portMAX_DELAY);
                    for (int i = 0; i < event.size; i++) {
                        char command = dtmp[i];
                        if (command == 'V') {
                            ESP_LOGI(TAG, "Vibration detected: %c", command);
                            play_effect();
                        }
                    }
                    break;
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    uart_flush_input(UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    uart_flush_input(UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error");
                    break;
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart frame error");
                    break;
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    free(dtmp);
    vTaskDelete(NULL);
}

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(TAG, ESP_LOG_INFO);

    ESP_LOGI(TAG, "[1.0] Start audio codec chip");
    audio_board_handle_t board_handle = audio_board_init();
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_DECODE, AUDIO_HAL_CTRL_START);

    ESP_LOGI(TAG, "[2.0] Start and wait for SDCARD to mount");
    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);
    audio_board_sdcard_init(set, SD_MODE_1_LINE);

    ESP_LOGI(TAG, "[3.0] Create audio pipeline for music");
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    music_pipeline = audio_pipeline_init(&pipeline_cfg);
    effect_pipeline = audio_pipeline_init(&pipeline_cfg);

    ESP_LOGI(TAG, "[3.1] Create Fatfs stream to read music data");
    fatfs_stream_cfg_t fatfs_cfg = FATFS_STREAM_CFG_DEFAULT();
    fatfs_cfg.type = AUDIO_STREAM_READER;

    fats_rd_el_music = fatfs_stream_init(&fatfs_cfg);
    fats_rd_el_effect = fatfs_stream_init(&fatfs_cfg);

    ESP_LOGI(TAG, "[3.2] Create wav decoder to decode wav file");
    wav_decoder_cfg_t wav_cfg = DEFAULT_WAV_DECODER_CONFIG();
    audio_element_handle_t wav_decoder_music = wav_decoder_init(&wav_cfg);
    audio_element_handle_t wav_decoder_effect = wav_decoder_init(&wav_cfg);

    ESP_LOGI(TAG, "[3.3] Create i2s stream to write data to codec chip");
    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg.type = AUDIO_STREAM_WRITER;
    audio_element_handle_t i2s_writer_music = i2s_stream_init(&i2s_cfg);
    audio_element_handle_t i2s_writer_effect = i2s_stream_init(&i2s_cfg);

    i2s_stream_set_clk(i2s_writer_music, DEST_SAMPLERATE, 16, DEST_CHANNEL);
    i2s_stream_set_clk(i2s_writer_effect, DEST_SAMPLERATE, 16, DEST_CHANNEL);

    ESP_LOGI(TAG, "[3.4] Register elements to music pipeline");
    audio_pipeline_register(music_pipeline, fats_rd_el_music, "file");
    audio_pipeline_register(music_pipeline, wav_decoder_music, "wav");
    audio_pipeline_register(music_pipeline, i2s_writer_music, "i2s");

    ESP_LOGI(TAG, "[3.5] Link elements together [file]-->wav-->i2s");
    audio_pipeline_link(music_pipeline, (const char *[]) {"file", "wav", "i2s"}, 3);

    ESP_LOGI(TAG, "[3.6] Register elements to effect pipeline");
    audio_pipeline_register(effect_pipeline, fats_rd_el_effect, "file");
    audio_pipeline_register(effect_pipeline, wav_decoder_effect, "wav");
    audio_pipeline_register(effect_pipeline, i2s_writer_effect, "i2s");

    ESP_LOGI(TAG, "[3.7] Link elements together [file]-->wav-->i2s");
    audio_pipeline_link(effect_pipeline, (const char *[]) {"file", "wav", "i2s"}, 3);

    ESP_LOGI(TAG, "[4.0] Set up UART");
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_driver_install(UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 10, NULL);

    // Set up event listener for the music pipeline
    audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
    audio_event_iface_handle_t evt = audio_event_iface_init(&evt_cfg);
    audio_pipeline_set_listener(music_pipeline, evt);

    // Play music on startup
    play_music();

    while (1) {
        audio_event_iface_msg_t msg;
        esp_err_t ret = audio_event_iface_listen(evt, &msg, portMAX_DELAY);
        if (ret == ESP_OK) {
            if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && msg.source == (void *)fats_rd_el_music &&
                msg.cmd == AEL_MSG_CMD_REPORT_STATUS && msg.data == (void *)AEL_STATUS_STATE_FINISHED) {
                ESP_LOGI(TAG, "Music finished, restarting");
                play_music();
            }
        }
    }

    audio_event_iface_destroy(evt);
}
*/






