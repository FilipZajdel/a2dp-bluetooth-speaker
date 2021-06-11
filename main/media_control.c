#include "media_control.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_avrc_api.h"

#define MEDIA_CONTROL_LOGGER_TAG "media_control"
#define MEDIA_BUTTONS_PIN_SEL                                                  \
    ((1ULL << CONFIG_MEDIA_BUTTON_NEXT_PIN) |                                  \
     (1ULL << CONFIG_MEDIA_BUTTON_PLAY_PAUSE) |                                \
     (1ULL << CONFIG_MEDIA_BUTTON_PREV_PIN) |                                  \
     (1ULL << CONFIG_MEDIA_VOLUME_UP_PIN) |                                    \
     (1ULL << CONFIG_MEDIA_VOLUME_DOWN_PIN))

#define MEDIA_CMD_QUEUE_SIZE (1)
#define BUTTON_DEBOUNCE_TIMEOUT_ms (125)
#define MEDIA_CONTROL_TASK_PRIO (configMAX_PRIORITIES - 4)

static xQueueHandle media_commands_queue;

static void media_control_send_command(uint32_t command);

static void IRAM_ATTR buttons_isr(void *gpio_pin)
{
    uint32_t gpio_num = (uint32_t)gpio_pin;

    gpio_intr_disable(gpio_num);

    xQueueSendFromISR(media_commands_queue, &gpio_num, NULL);
}

static void media_handle_pin(uint32_t gpio_pin)
{
    uint32_t command;

    switch (gpio_pin) {
        case CONFIG_MEDIA_BUTTON_NEXT_PIN:
            command = ESP_AVRC_PT_CMD_FORWARD;
            break;
        case CONFIG_MEDIA_BUTTON_PREV_PIN:
            command = ESP_AVRC_PT_CMD_BACKWARD;
            break;
        case CONFIG_MEDIA_VOLUME_UP_PIN:
            ESP_LOGE(MEDIA_CONTROL_LOGGER_TAG, "Volume up unsupported");
            return;
        case CONFIG_MEDIA_VOLUME_DOWN_PIN:
            ESP_LOGE(MEDIA_CONTROL_LOGGER_TAG, "Volume down unsupported");
            return;
        case CONFIG_MEDIA_BUTTON_PLAY_PAUSE:
            command = ESP_AVRC_PT_CMD_PLAY;
            break;
        default:
            ESP_LOGE(MEDIA_CONTROL_LOGGER_TAG, "Unhandled pin %u", gpio_pin);
            return;
    }

    media_control_send_command(command);
}

static void media_control_task(void *arg)
{
    uint32_t gpio_pin = 0;

    while (true) {
        xQueueReceive(media_commands_queue, &gpio_pin, portMAX_DELAY);
        media_handle_pin(gpio_pin);

        vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_TIMEOUT_ms));
        gpio_intr_enable(gpio_pin);
    }
}

void media_buttons_register(void)
{
    gpio_config_t io_conf = {.intr_type = CONFIG_MEDIA_BUTTONS_INTERRUPT_TYPE,
                             .mode      = GPIO_MODE_INPUT,
                             .pin_bit_mask = MEDIA_BUTTONS_PIN_SEL,
                             .pull_down_en =
                                 CONFIG_MEDIA_BUTTONS_PULL_DOWN_ENABLE,
                             .pull_up_en = CONFIG_MEDIA_BUTTONS_PULL_UP_ENABLE};

    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(CONFIG_MEDIA_BUTTON_PLAY_PAUSE, buttons_isr,
                         (void *)CONFIG_MEDIA_BUTTON_PLAY_PAUSE);

    gpio_isr_handler_add(CONFIG_MEDIA_BUTTON_NEXT_PIN, buttons_isr,
                         (void *)CONFIG_MEDIA_BUTTON_NEXT_PIN);

    media_commands_queue = xQueueCreate(MEDIA_CMD_QUEUE_SIZE, sizeof(uint32_t));
    xTaskCreate(media_control_task, "media_ctrl_tsk", 2048, NULL,
                MEDIA_CONTROL_TASK_PRIO, NULL);
}

void media_control_send_command(uint32_t command)
{
    esp_err_t      ret;
    static uint8_t tl = 0;

    ret = esp_avrc_ct_send_passthrough_cmd(tl++ & 0xF, command,
                                           ESP_AVRC_PT_CMD_STATE_PRESSED);

    if (ret != ESP_OK) {
        ESP_LOGE(MEDIA_CONTROL_LOGGER_TAG, "failed to send command");
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(25));
    ret = esp_avrc_ct_send_passthrough_cmd(tl++ & 0xF, command,
                                           ESP_AVRC_PT_CMD_STATE_RELEASED);
    if (ret != ESP_OK) {
        ESP_LOGE(MEDIA_CONTROL_LOGGER_TAG, "failed to send command");
    }
}
