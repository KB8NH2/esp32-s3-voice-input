#include "wifi.h"
#include "mic_speech.h"
#include "vad.h"
#include "stt.h"
#include "conversation_api.h"
#include "piper_tts.h"
#include "audio_driver.h"
#include "lcd_driver.h"
#include "lvgl_driver.h"
#include "bsp_board.h"
#include "tca9555_driver.h"

#include "esp_log.h"
#include "esp_heap_caps.h"
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "lvgl.h"

static const char *TAG = "main";

// LVGL label objects for displaying STT and conversation text
static lv_obj_t *label_stt = NULL;
static lv_obj_t *label_conversation = NULL;

// Global log level management via keys
static const esp_log_level_t s_log_levels[] = {
    ESP_LOG_NONE,
    ESP_LOG_ERROR,
    ESP_LOG_WARN,
    ESP_LOG_INFO,
    ESP_LOG_DEBUG,
    ESP_LOG_VERBOSE,
};
static const int s_log_levels_count = sizeof(s_log_levels) / sizeof(s_log_levels[0]);
static int s_log_level_idx = 3; // default to ESP_LOG_INFO

static void set_global_log_level_idx(int idx)
{
    if (idx < 0) idx = 0;
    if (idx >= s_log_levels_count) idx = s_log_levels_count - 1;
    s_log_level_idx = idx;
    esp_log_level_t lvl = s_log_levels[s_log_level_idx];
    esp_log_level_set("*", lvl);
    esp_log_level_set("wifi", ESP_LOG_WARN); // keep wifi quieter
    static const char *s_log_level_names[] = { "NONE", "ERROR", "WARN", "INFO", "DEBUG", "VERBOSE" };
    const char *name = "UNKNOWN";
    if (s_log_level_idx >= 0 && s_log_level_idx < (int)(sizeof(s_log_level_names) / sizeof(s_log_level_names[0]))) {
        name = s_log_level_names[s_log_level_idx];
    }
    // Use printf so the message is visible even if the new level suppresses ESP_LOG logs
    printf("%s: Global log level set to %d (%s)\n", TAG, (int)lvl, name);
}

// PTT event queue
typedef enum { PTT_EVENT_NONE = 0, PTT_EVENT_VAD_END } ptt_event_t;
static QueueHandle_t s_ptt_event_queue = NULL;
static volatile int s_ptt_listening = 0;
static volatile int s_stay_in_conversation = 0;

// Argument passed to background and sender STT tasks
struct stt_task_arg {
    int16_t *buf;
    size_t samples;
    int owns_buf; // 1 if buf must be freed separately, 0 if buf is embedded in this struct
};

// Preallocated STT buffer pool (use PSRAM when available)
#define STT_POOL_COUNT 3
#define STT_POOL_SAMPLES 48000
static int16_t *s_stt_pool[STT_POOL_COUNT];
static int s_stt_pool_in_use[STT_POOL_COUNT];
static SemaphoreHandle_t s_stt_pool_mutex = NULL;
// Single-worker STT sender queue
static QueueHandle_t s_stt_queue = NULL;
#define STT_QUEUE_LEN 4

static void stt_pool_init(void)
{
    if (!s_stt_pool_mutex) s_stt_pool_mutex = xSemaphoreCreateMutex();
    for (int i = 0; i < STT_POOL_COUNT; ++i) {
        s_stt_pool[i] = NULL;
        s_stt_pool_in_use[i] = 0;
        // prefer PSRAM
        s_stt_pool[i] = heap_caps_malloc(STT_POOL_SAMPLES * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (s_stt_pool[i]) {
            ESP_LOGD(TAG, "stt_pool_init: allocated pool[%d] in SPIRAM", i);
        } else {
            s_stt_pool[i] = heap_caps_malloc(STT_POOL_SAMPLES * sizeof(int16_t), MALLOC_CAP_8BIT);
            if (s_stt_pool[i]) {
                ESP_LOGD(TAG, "stt_pool_init: allocated pool[%d] in internal RAM", i);
            } else {
                ESP_LOGW(TAG, "stt_pool_init: failed to allocate pool[%d] (%u bytes)", i, (unsigned)(STT_POOL_SAMPLES * sizeof(int16_t)));
                // leave NULL; fallback allocations will be used
                break;
            }
        }
    }
}

static int16_t *stt_pool_alloc(void)
{
    int16_t *ret = NULL;
    if (!s_stt_pool_mutex) return NULL;
    if (xSemaphoreTake(s_stt_pool_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        for (int i = 0; i < STT_POOL_COUNT; ++i) {
            if (s_stt_pool[i] && !s_stt_pool_in_use[i]) {
                s_stt_pool_in_use[i] = 1;
                ret = s_stt_pool[i];
                break;
            }
        }
        xSemaphoreGive(s_stt_pool_mutex);
    } else {
        ESP_LOGW(TAG, "stt_pool_alloc: failed to take pool mutex");
    }
    if (ret) {
        ESP_LOGD(TAG, "stt_pool_alloc: allocated pool buffer %p", ret);
    } else {
        ESP_LOGD(TAG, "stt_pool_alloc: pool exhausted or uninitialized");
    }
    return ret;
}

static void stt_pool_free(int16_t *buf)
{
    if (!s_stt_pool_mutex || !buf) return;
    if (xSemaphoreTake(s_stt_pool_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        for (int i = 0; i < STT_POOL_COUNT; ++i) {
            if (s_stt_pool[i] == buf) {
                s_stt_pool_in_use[i] = 0;
                break;
            }
        }
        xSemaphoreGive(s_stt_pool_mutex);
        ESP_LOGD(TAG, "stt_pool_free: freed pool buffer %p", buf);
    }
}

// VAD -> PTT notifier
// forward declarations for background STT/wake-word handling
static void stt_bg_task(void *v);
static int is_wake_word(const char *text);
static void stt_sender_task(void *v);

static void vad_ptt_notify_cb(const int16_t *samples, size_t count)
{
    ESP_LOGD(TAG, "VAD cb: count=%u s_ptt_listening=%d s_stay_in_conversation=%d", (unsigned)count, (int)s_ptt_listening, (int)s_stay_in_conversation);
    // If we're currently in PTT listening mode:
    // - In normal PTT flow, notify the PTT task to stop and send (VAD END).
    // - In continuous conversation mode, keep capture running and directly
    //   spawn an STT sender with a copy of the VAD buffer so we avoid
    //   stop/start races that produce tiny/noisy uploads.
    if (s_ptt_listening) {
        if (s_stay_in_conversation) {
            if (count == 0 || samples == NULL) return;
                if (count == 0 || samples == NULL) return;
                // Try to use a pooled buffer first to avoid large mallocs
                int16_t *pbuf = stt_pool_alloc();
                if (pbuf) {
                    size_t to_copy = count > STT_POOL_SAMPLES ? STT_POOL_SAMPLES : count;
                    memcpy(pbuf, samples, to_copy * sizeof(int16_t));
                    struct stt_task_arg *arg = malloc(sizeof(*arg));
                    if (!arg) {
                        stt_pool_free(pbuf);
                        ESP_LOGW(TAG, "Failed to allocate stt task arg");
                        return;
                    }
                    arg->buf = pbuf;
                    arg->samples = to_copy;
                    arg->owns_buf = 0;
                    // Enqueue for single-worker STT sender
                    if (s_stt_queue && xQueueSend(s_stt_queue, &arg, pdMS_TO_TICKS(10)) == pdTRUE) {
                        ESP_LOGD(TAG, "Enqueued pooled VAD segment %p samples=%u", pbuf, (unsigned)to_copy);
                    } else {
                        ESP_LOGW(TAG, "STT queue full or unavailable; dropping segment");
                        stt_pool_free(pbuf);
                        free(arg);
                    }
                    return;
                }

                // Fallback: allocate one contiguous block for the task arg + VAD samples.
                size_t buf_bytes = count * sizeof(int16_t);
                // Guard against exhausting PSRAM/internal heap when pool is empty.
                size_t ps_largest = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
                size_t heap_free = esp_get_free_heap_size();
                if ((ps_largest > 0 && ps_largest < buf_bytes + 1024) || (ps_largest == 0 && heap_free < buf_bytes + 4096)) {
                    ESP_LOGW(TAG, "Skipping VAD segment: insufficient contiguous RAM for %u bytes (ps_largest=%u free=%u)", (unsigned)buf_bytes, (unsigned)ps_largest, (unsigned)heap_free);
                    return;
                }

                struct stt_task_arg *arg = NULL;
                // Prefer PSRAM if available, fall back to internal RAM.
                arg = heap_caps_malloc(sizeof(*arg) + buf_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
                if (!arg) {
                    arg = heap_caps_malloc(sizeof(*arg) + buf_bytes, MALLOC_CAP_8BIT);
                }
                if (!arg) {
                    size_t free_heap = esp_get_free_heap_size();
                    size_t largest = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
                    ESP_LOGW(TAG, "Failed to allocate stt_task_arg+buf for VAD samples");
                    ESP_LOGW(TAG, "heap free=%u largest_block=%u", (unsigned)free_heap, (unsigned)largest);
                    return;
                }
                arg->owns_buf = 0;
                arg->samples = count;
                arg->buf = (int16_t *)(arg + 1);
                memcpy(arg->buf, samples, buf_bytes);
                ESP_LOGD(TAG, "Allocated stt_task_arg+buf %p (%u bytes)", arg, (unsigned)(sizeof(*arg) + buf_bytes));

                // Enqueue for single-worker STT sender (keeps capture running).
                if (s_stt_queue && xQueueSend(s_stt_queue, &arg, pdMS_TO_TICKS(10)) == pdTRUE) {
                    // queued
                } else {
                    ESP_LOGW(TAG, "STT queue full or unavailable; dropping VAD segment");
                    heap_caps_free(arg);
                }
                return;
        }

        if (s_ptt_event_queue) {
            ptt_event_t ev = PTT_EVENT_VAD_END;
            if (xQueueSend(s_ptt_event_queue, &ev, 0) == pdTRUE) {
                ESP_LOGD(TAG, "VAD cb -> posted PTT_EVENT_VAD_END");
            } else {
                ESP_LOGW(TAG, "VAD cb -> failed to post PTT_EVENT_VAD_END");
            }
        }
        return;
    }

    // Not in PTT mode — spawn background STT task to check for wake-word.
    // Skip if queue has pending items to avoid backlog
    if (count == 0 || samples == NULL) return;
    if (s_stt_queue && uxQueueMessagesWaiting(s_stt_queue) > 0) {
        ESP_LOGD(TAG, "STT queue busy, skipping background segment");
        return;
    }

    // Try pool first for background segments as well
    int16_t *pbuf = stt_pool_alloc();
    if (pbuf) {
        size_t to_copy = count > STT_POOL_SAMPLES ? STT_POOL_SAMPLES : count;
        memcpy(pbuf, samples, to_copy * sizeof(int16_t));
        struct stt_task_arg *arg = malloc(sizeof(*arg));
        if (!arg) {
            stt_pool_free(pbuf);
            return;
        }
        arg->buf = pbuf;
        arg->samples = to_copy;
        arg->owns_buf = 0;
        // Enqueue pooled background segment to single-worker sender
        if (s_stt_queue && xQueueSend(s_stt_queue, &arg, 0) == pdTRUE) {
            ESP_LOGD(TAG, "Enqueued background pooled segment %p samples=%u", pbuf, (unsigned)to_copy);
        } else {
            ESP_LOGD(TAG, "STT queue full; dropping background segment");
            stt_pool_free(pbuf);
            free(arg);
        }
        return;
    }

    // Fallback: heap allocate a copy. Guard against large allocations when PSRAM is fragmented.
    size_t buf_bytes = count * sizeof(int16_t);
    size_t ps_largest = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
    size_t heap_free = esp_get_free_heap_size();
    if ((ps_largest > 0 && ps_largest < buf_bytes + 1024) || (ps_largest == 0 && heap_free < buf_bytes + 4096)) {
        ESP_LOGD(TAG, "Skipping background VAD segment: insufficient contiguous RAM for %u bytes (ps_largest=%u free=%u)", (unsigned)buf_bytes, (unsigned)ps_largest, (unsigned)heap_free);
        return;
    }
    int16_t *buf = heap_caps_malloc(buf_bytes, MALLOC_CAP_8BIT);
    if (!buf) return;
    ESP_LOGD(TAG, "Allocated fallback bg buffer %p bytes=%u", buf, (unsigned)buf_bytes);
    memcpy(buf, samples, count * sizeof(int16_t));

    struct stt_task_arg *arg = malloc(sizeof(*arg));
    if (!arg) {
        free(buf);
        return;
    }
    arg->buf = buf;
    arg->samples = count;
    arg->owns_buf = 1;

    // Enqueue to worker instead of spawning new task
    if (s_stt_queue && xQueueSend(s_stt_queue, &arg, 0) == pdTRUE) {
        ESP_LOGD(TAG, "Enqueued background fallback segment %p samples=%u", buf, (unsigned)count);
    } else {
        ESP_LOGD(TAG, "STT queue full; dropping background fallback segment");
        free(buf);
        free(arg);
    }
}

// Forward declarations
static void on_stt_result(const char *text);
static void on_conversation_reply(const char *reply_text);
static void ptt_task(void *arg);

// Helper to process a single STT task_arg (used by both worker and existing task wrapper)
static void stt_process_arg(struct stt_task_arg *a)
{
    ESP_LOGD(TAG, "stt_task: calling stt_send_wav_multipart samples=%u", (unsigned)a->samples);
    char *resp = stt_send_wav_multipart(a->buf, a->samples);
    if (a->owns_buf) {
        free(a->buf);
    } else {
        stt_pool_free(a->buf);
    }
    if (resp) {
        if (strlen(resp) > 0) {
            ESP_LOGI(TAG, "Whisper returned: \"%s\"", resp);
            if (label_stt) {
                if (lvgl_port_lock(pdMS_TO_TICKS(1000))) {
                    char display_text[128];
                    snprintf(display_text, sizeof(display_text), "STT: %s", resp);
                    lv_label_set_text(label_stt, display_text);
                    lv_obj_invalidate(label_stt);
                    lvgl_port_unlock();
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
            }
            if (is_wake_word(resp)) {
                ESP_LOGI(TAG, "Wake-word detected! Entering listening mode");
                // Start capture and enter LISTENING so VAD events are processed
                mic_start_capture();
                s_ptt_listening = 1;
                if (!s_stay_in_conversation) {
                    led_ring_set_color(0, 64, 0); // dim green
                } else {
                    led_ring_set_color(0, 0, 64); // dim blue
                }
            } else {
                // Only forward non-wake-word background segments when in
                // continuous conversation mode or when we are actively
                // listening (PTT). If this arg originated from a PTT
                // capture (`owns_buf == 1`) always forward.
                ESP_LOGD(TAG, "owns_buf=%d s_stay_in_conv=%d s_ptt_listening=%d", a->owns_buf, s_stay_in_conversation, s_ptt_listening);
                if (a->owns_buf || s_stay_in_conversation || s_ptt_listening) {
                    on_stt_result(resp);
                } else {
                    ESP_LOGD(TAG, "Background STT (not wake-word, not listening): %s", resp);
                }
            }
        }
        free(resp);
    } else {
        ESP_LOGW(TAG, "stt_send_wav_multipart returned NULL");
    }
    free(a);
}

// Legacy wrapper: allows creating a dedicated task if needed
static void stt_sender_task(void *v)
{
    struct stt_task_arg *a = (struct stt_task_arg *)v;
    stt_process_arg(a);
    vTaskDelete(NULL);
}

// Single persistent worker that serially processes STT uploads from a queue
static void stt_sender_worker(void *v)
{
    struct stt_task_arg *a = NULL;
    for (;;) {
        if (xQueueReceive(s_stt_queue, &a, portMAX_DELAY) == pdTRUE) {
            if (a) {
                ESP_LOGD(TAG, "stt_worker: processing arg %p samples=%u owns_buf=%d", a, (unsigned)a->samples, a->owns_buf);
                stt_process_arg(a);
            }
        }
    }
}

// Background STT task for wake-word detection. It sends the captured
// VAD segment to STT and, if the result matches a wake-word, triggers
// the PTT listening flow (start capture / set LED / set flag).
static int is_wake_word(const char *text)
{
    if (!text) return 0;

    size_t len = strlen(text);
    char *clean = malloc(len + 1);
    if (!clean) return 0;

    size_t ci = 0;
    int prev_space = 0;
    for (size_t i = 0; i < len; ++i) {
        unsigned char c = (unsigned char)text[i];
        if (isalnum(c)) {
            clean[ci++] = tolower(c);
            prev_space = 0;
        } else if (isspace(c)) {
            if (!prev_space && ci > 0) {
                clean[ci++] = ' ';
                prev_space = 1;
            }
        } else {
            // punctuation: treat like a separator
            if (!prev_space && ci > 0) {
                clean[ci++] = ' ';
                prev_space = 1;
            }
        }
    }
    // trim trailing space
    if (ci > 0 && clean[ci - 1] == ' ') ci--;
    clean[ci] = '\0';

    const char *wakes[] = { "hello computer", 
        "hey computer", 
        "okay computer", 
        "hey jarvis", 
        "wake up", 
        "echo on", "turn echo on", "turn on echo",
        "echo off", "turn echo off", "turn off echo",
        "debug on", "turn debug on", "turn on debug",
        "debug off", "turn debug off", "turn off debug",
        NULL };
    int found = 0;
    for (int i = 0; wakes[i]; ++i) {
        if (strcmp(clean, wakes[i]) == 0) { found = 1; break; }
    }
    if (found) {
        if ((strcmp(clean, "echo on") == 0) || 
            (strcmp(clean, "turn echo on") == 0) || 
            (strcmp(clean, "turn on echo") == 0)) {
            s_stay_in_conversation = 1;
            led_ring_set_color(0, 0, 64); // dim blue     
            ESP_LOGI(TAG, "Entering continuous conversation mode");
            conversation_send("echo on");
            found = 0; // don't treat as wake-word
        } else if ((strcmp(clean, "echo off") == 0) || 
                   (strcmp(clean, "turn echo off") == 0) || 
                   (strcmp(clean, "turn off echo") == 0)) {  
            s_ptt_listening = 0;
            s_stay_in_conversation = 0;
            led_ring_clear();
            ESP_LOGI(TAG, "Exiting continuous conversation mode");
            conversation_send("echo off");
            found = 0; // don't treat as wake-word
        } else if ((strcmp(clean, "debug on") == 0) || 
                   (strcmp(clean, "turn debug on") == 0) || 
                   (strcmp(clean, "turn on debug") == 0)) {
            set_global_log_level_idx(s_log_level_idx + 1); // DEBUG/VERBOSE
            printf("Debug logging enabled");
            found = 0; // don't treat as wake-word
        } else if ((strcmp(clean, "debug off") == 0) || 
                   (strcmp(clean, "turn debug off") == 0) || 
                   (strcmp(clean, "turn off debug") == 0)) {
            set_global_log_level_idx(s_log_level_idx - 1); // INFO
            printf("Debug logging disabled");
            found = 0; // don't treat as wake-word
        }   
    }
    free(clean);
    return found;
}

static void stt_bg_task(void *v)
{
    struct stt_task_arg *a = (struct stt_task_arg *)v;
    ESP_LOGD(TAG, "stt_bg: calling stt_send_wav_multipart samples=%u", (unsigned)a->samples);
    char *resp = stt_send_wav_multipart(a->buf, a->samples);
    if (a->owns_buf) {
        free(a->buf);
    } else {
        stt_pool_free(a->buf);
    }
    if (resp) {
        if (strlen(resp) > 0) {
            ESP_LOGI(TAG, "Whisper returned: \"%s\"", resp);
            // Update STT label on LCD
            ESP_LOGD(TAG, "Attempting LCD update, label_stt=%p", label_stt);
            if (label_stt) {
                if (lvgl_port_lock(pdMS_TO_TICKS(1000))) {
                    char display_text[128];
                    snprintf(display_text, sizeof(display_text), "STT: %s", resp);
                    lv_label_set_text(label_stt, display_text);
                    lv_obj_invalidate(label_stt);
                    lvgl_port_unlock();
                    // Give LVGL task time to process the update
                    vTaskDelay(pdMS_TO_TICKS(100));
                    ESP_LOGD(TAG, "LCD updated with STT text");
                } else {
                    ESP_LOGW(TAG, "Failed to acquire LVGL lock for STT update");
                }
            } else {
                ESP_LOGW(TAG, "label_stt is NULL, cannot update");
            }
            
            if (is_wake_word(resp)) {
                ESP_LOGD(TAG, "Wake-word detected -> entering LISTEN mode");
                // Update LCD to show wake-word was heard (but don't send to conversation)
                if (label_stt) {
                    if (lvgl_port_lock(pdMS_TO_TICKS(1000))) {
                        char display_text[128];
                        snprintf(display_text, sizeof(display_text), "STT: %s", resp);
                        lv_label_set_text(label_stt, display_text);
                        lv_obj_invalidate(label_stt);
                        lvgl_port_unlock();
                        vTaskDelay(pdMS_TO_TICKS(100));
                    }
                }
                mic_start_capture();
                // Enter LISTENING for the PTT state machine regardless of
                // whether continuous conversation mode is enabled.
                s_ptt_listening = 1;
                if (!s_stay_in_conversation) {
                    led_ring_set_color(0, 64, 0); // dim green
                } else {
                    led_ring_set_color(0, 0, 64); // dim blue
                }
            } else {
                // Not a wake-word: forward to conversation if we're already
                // in continuous conversation mode OR the system is currently
                // in LISTENING mode (user pressed PTT or wake-word started
                // capture). Otherwise just log the background transcription.
                if (s_stay_in_conversation || s_ptt_listening) {
                    on_stt_result(resp);
                } else {
                    ESP_LOGD(TAG, "Background STT (not wake-word): %s", resp);
                }
            }
        }
        free(resp);
    } else {
        ESP_LOGW(TAG, "stt_send_wav_multipart (bg) returned NULL");
    }
    free(a);
    vTaskDelete(NULL);
}

static void ptt_stop_and_send(void)
{
    ESP_LOGD(TAG, "PTT: ptt_stop_and_send invoked");
    mic_stop_capture();
    if (!s_stay_in_conversation) {
        led_ring_clear();
    }
    size_t samples = 0;
    int16_t *buf = mic_take_captured_buffer(&samples);
    ESP_LOGD(TAG, "PTT: captured samples=%u", (unsigned)samples);
    if (samples == 0) {
        ESP_LOGW(TAG, "PTT: mic_take_captured_buffer returned 0 samples");
    }
        if (buf && samples > 0) {
        // Offload STT upload to its own task to avoid stack exhaustion
        ESP_LOGD(TAG, "PTT: spawning stt sender task for %u samples", (unsigned)samples);
        struct stt_task_arg *arg = malloc(sizeof(*arg));
        if (!arg) {
            ESP_LOGE(TAG, "Failed to allocate stt task arg");
            free(buf);
            return;
        }
        arg->buf = buf;
        arg->samples = samples;
        arg->owns_buf = 1;
            if (s_stt_queue && xQueueSend(s_stt_queue, &arg, pdMS_TO_TICKS(10)) == pdTRUE) {
                ESP_LOGD(TAG, "Enqueued captured audio %p samples=%u", arg->buf, (unsigned)arg->samples);
            } else {
                ESP_LOGE(TAG, "STT queue full or unavailable; dropping captured audio");
                if (arg->owns_buf) free(arg->buf); else stt_pool_free(arg->buf);
                free(arg);
            }
    } else {
        ESP_LOGW(TAG, "No audio captured to send");
    }
}

void app_main(void)
{
    ESP_LOGD(TAG, "Booting voice endpoint firmware");

    // 1. Connect Wi-Fi
    wifi_init();
    wifi_wait_connected();

    // 2. Initialize audio (mic + speaker) and I2C bus
    Speech_Init();

    // 2b. Initialize TCA9555 I/O expander driver (must be after I2C bus creation)
    tca9555_driver_init();

    // 3. Initialize LCD display (requires TCA9555 to be initialized first)
    ESP_LOGI(TAG, "Initializing LCD display");
    lcd_driver_init();
    lvgl_driver_init();

    // Create simple UI with two text labels
    lvgl_port_lock(0);
    lv_obj_t *screen = lv_screen_active();
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x000000), 0);

    // STT label (top line) - larger font for visibility
    label_stt = lv_label_create(screen);
    lv_label_set_text(label_stt, "STT: Listening for wake word...");
    lv_obj_set_style_text_color(label_stt, lv_color_hex(0x00FF00), 0);
    lv_obj_set_style_text_font(label_stt, &lv_font_montserrat_14, 0);
    lv_obj_align(label_stt, LV_ALIGN_TOP_LEFT, 5, 20);
    lv_label_set_long_mode(label_stt, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(label_stt, 310);

    // Conversation label (second line) - larger font
    label_conversation = lv_label_create(screen);
    lv_label_set_text(label_conversation, "AI: Listening for wake word...");
    lv_obj_set_style_text_color(label_conversation, lv_color_hex(0x00FFFF), 0);
    lv_obj_set_style_text_font(label_conversation, &lv_font_montserrat_14, 0);
    lv_obj_align(label_conversation, LV_ALIGN_TOP_LEFT, 5, 60);
    lv_label_set_long_mode(label_conversation, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(label_conversation, 310);

    // Force screen update
    lv_refr_now(NULL);
    lvgl_port_unlock();

    ESP_LOGI(TAG, "LCD display initialized with text labels");

    // Initialize STT buffer pool
    stt_pool_init();

    // PSRAM probe: log total PSRAM and largest free block for diagnostics
    size_t ps_total = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    size_t ps_largest = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
    if (ps_total == 0) {
        ESP_LOGW(TAG, "PSRAM not available (total=0)");
    } else {
        ESP_LOGI(TAG, "PSRAM total: %u bytes, largest free block: %u bytes", (unsigned)ps_total, (unsigned)ps_largest);
    }

    // create PTT event queue and register VAD callback to notify PTT task on VAD END
    s_ptt_event_queue = xQueueCreate(4, sizeof(ptt_event_t));
    vad_set_callback(vad_ptt_notify_cb);

    // Create single-worker STT sender queue and worker
    s_stt_queue = xQueueCreate(STT_QUEUE_LEN, sizeof(void *));
    if (s_stt_queue) {
        xTaskCreatePinnedToCore(stt_sender_worker, "stt_worker", 16384, NULL, 5, NULL, 1);
        ESP_LOGD(TAG, "STT sender queue created (len=%d) and worker started", STT_QUEUE_LEN);
    } else {
        ESP_LOGW(TAG, "Failed to create STT sender queue");
    }

    // 5. Start push-to-talk state machine task
    xTaskCreatePinnedToCore(ptt_task, "ptt_task", 4096, NULL, 5, NULL, 1);

    // 6. Initialize Conversation API
    conversation_init("http://192.168.1.154:8000/conversation");

    // 7. Register Conversation API callback
    conversation_set_callback(on_conversation_reply);

    // ensure global log level is applied at startup
    set_global_log_level_idx(s_log_level_idx);

    ESP_LOGI(TAG, "System ready");
}

/********************************************************************
 *  CALLBACK CHAIN
 ********************************************************************/

// Push-to-talk task: polls key3 and captures audio while pressed
static void ptt_task(void *arg)
{
    enum { STATE_IDLE = 0, STATE_LISTENING, STATE_PROCESSING, STATE_PLAYING_TTS } state = STATE_IDLE;

    bool last_pressed = false;
    bool last_pressed1 = false;
    bool last_pressed2 = false;

    // helpers to detect state changes for extra diagnostics
    bool prev_pressed1 = false;
    bool prev_pressed2 = false;

    for (;;) {
        // check for VAD notifications
        if (s_ptt_event_queue) {
            ptt_event_t ev;
            if (xQueueReceive(s_ptt_event_queue, &ev, 0) == pdTRUE) {
                if (ev == PTT_EVENT_VAD_END) {
                    ESP_LOGD(TAG, "PTT: VAD END event received (state=%d), handling if LISTENING", state);
                    if (state == STATE_LISTENING) {
                        ESP_LOGD(TAG, "PTT: VAD END detected, stopping and sending");
                        ptt_stop_and_send();
                        // If not in continuous conversation mode, stop listening
                        // and move to processing. Do NOT restart capture here.
                        if (!s_stay_in_conversation) {
                            s_ptt_listening = 0;
                            state = STATE_PROCESSING;
                        } else {
                            // In continuous mode we want to keep listening; restart capture.
                            ESP_LOGD(TAG, "PTT: staying in conversation, restarting capture");
                            mic_start_capture();
                            led_ring_set_color(0, 0, 64); // dim blue
                        }
                    }
                }
            }
        }
        bool pressed = debounce_key3();
        bool pressed_edge = pressed && !last_pressed; // detect press event
        last_pressed = pressed;

        bool pressed1 = debounce_key1();
        bool pressed_edge1 = pressed1 && !last_pressed1;
        // diagnostic: print on any state change
        if (pressed1 != prev_pressed1) {
            ESP_LOGD(TAG, "key1 state change -> %d", (int)pressed1);
            prev_pressed1 = pressed1;
        }
        last_pressed1 = pressed1;

        bool pressed2 = debounce_key2();
        bool pressed_edge2 = pressed2 && !last_pressed2;
        if (pressed2 != prev_pressed2) {
            ESP_LOGD(TAG, "key2 state change -> %d", (int)pressed2);
            prev_pressed2 = pressed2;
        }
        last_pressed2 = pressed2;

        if (pressed_edge1) {
            // increment log level
            set_global_log_level_idx(s_log_level_idx + 1);
        }
        if (pressed_edge2) {
            // decrement log level
            set_global_log_level_idx(s_log_level_idx - 1);
        }

        // If another context (wake-word) started capture by setting
        // `s_ptt_listening`, bring the state machine into LISTENING so
        // VAD END events are processed correctly.
        if (s_ptt_listening && state == STATE_IDLE) {
            state = STATE_LISTENING;
        }

        switch (state) {
        case STATE_IDLE:
            if (pressed_edge) {
                ESP_LOGD(TAG, "PTT: start listening (toggle)");
                mic_start_capture();
                led_ring_set_color(0, 0, 64); // dim blue
                s_ptt_listening = 1;
                state = STATE_LISTENING;
            }
            break;

        case STATE_LISTENING:
            if (pressed_edge) {
                ESP_LOGD(TAG, "PTT: stop listening (toggle), sending audio");
                // If user explicitly stops listening with key3, also exit
                // continuous conversation mode so we don't re-enter listen.
                s_ptt_listening = 0;
                led_ring_clear();
                if (s_stay_in_conversation) {   
                    ESP_LOGD(TAG, "Exiting continuous conversation mode");
                    s_stay_in_conversation = 0;
                }
                ptt_stop_and_send();
                state = STATE_PROCESSING;
            }
            break;

        case STATE_PROCESSING:
            // For now, transition back to idle. Conversation / TTS callbacks
            // could drive further states.
            state = STATE_IDLE;
            break;

        case STATE_PLAYING_TTS:
            state = STATE_IDLE;
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// Whisper STT → HA Conversation
static void on_stt_result(const char *text)
{
    if (strlen(text) == 0) {
        ESP_LOGD(TAG, "STT: empty result");
        return;
    }
    ESP_LOGD(TAG, "STT: \"%s\"", text);
    
    conversation_send(text);
}

// Conversation → HA TTS
static void on_conversation_reply(const char *reply_text)
{
    ESP_LOGD(TAG, "Conversation Server Reply: %s", reply_text);
    
    // Parse JSON response to extract just the "response" field
    const char *parsed_text = reply_text;
    if (reply_text && strstr(reply_text, "\"response\"")) {
        // Find "response": in the JSON
        const char *start = strstr(reply_text, "\"response\"");
        if (start) {
            // Move past "response" and find the colon
            start = strchr(start + 10, ':');  // +10 to skip past "response"
            if (start) {
                start++; // Move past the colon
                // Skip whitespace
                while (*start == ' ' || *start == '\t' || *start == '\n' || *start == '\r') start++;
                // Now we should be at the opening quote of the value
                if (*start == '"') {
                    start++; // Move past opening quote
                    const char *end = strchr(start, '"');  // Find closing quote
                    if (end) {
                        size_t len = end - start;
                        char *extracted = malloc(len + 1);
                        if (extracted) {
                            memcpy(extracted, start, len);
                            extracted[len] = '\0';
                            parsed_text = extracted;
                        }
                    }
                }
            }
        }
    }
    ESP_LOGI(TAG, "Conversation Reply: %s", parsed_text);
    // Update conversation label on LCD
    if (label_conversation) {
        lvgl_port_lock(0);
        char display_text[128];
        snprintf(display_text, sizeof(display_text), "Reply: %s", parsed_text);
        
        // Replace Unicode smart quotes with ASCII equivalents for display
        // U+2018 (') and U+2019 (') -> ' (0x27)
        // U+201C (") and U+201D (") -> " (0x22)
        for (int i = 0; display_text[i] != '\0'; i++) {
            if ((unsigned char)display_text[i] == 0xE2 && i + 2 < 128) {
                if ((unsigned char)display_text[i+1] == 0x80) {
                    if ((unsigned char)display_text[i+2] == 0x98 || 
                        (unsigned char)display_text[i+2] == 0x99) {
                        // Left/right single quote -> ASCII apostrophe
                        display_text[i] = '\'';
                        memmove(&display_text[i+1], &display_text[i+3], strlen(&display_text[i+3]) + 1);
                    } else if ((unsigned char)display_text[i+2] == 0x9C || 
                               (unsigned char)display_text[i+2] == 0x9D) {
                        // Left/right double quote -> ASCII quote
                        display_text[i] = '"';
                        memmove(&display_text[i+1], &display_text[i+3], strlen(&display_text[i+3]) + 1);
                    }
                }
            }
        }
        
        lv_label_set_text(label_conversation, display_text);
        lv_obj_invalidate(label_conversation);
        lvgl_port_unlock();
        vTaskDelay(pdMS_TO_TICKS(100));
        ESP_LOGD(TAG, "LCD updated with conversation text");
    }
    
    // Free extracted text if it was allocated
    if (parsed_text != reply_text) {
        free((void*)parsed_text);
    }
    
    //piper_tts_synthesize(reply_text);
}
