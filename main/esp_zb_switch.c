#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "led_strip.h" // Додано для керування адресним світлодіодом
#include "ha/esp_zigbee_ha_standard.h"
#include "zcl_utility.h"
#include "esp_zb_switch.h"

// Конфігурація вбудованого RGB LED на ESP32-C6
#define LED_PIN             GPIO_NUM_8
#define LED_STRIP_BLINK_MAX 1

static const char *TAG = "ESP_ZB_LIGHT";
static led_strip_handle_t led_strip;

typedef struct light_bulb_device_params_s {
    esp_zb_ieee_addr_t ieee_addr;
    uint8_t  endpoint;
    uint16_t short_addr;
} light_bulb_device_params_t;

static switch_func_pair_t button_func_pair[] = {
    {GPIO_INPUT_IO_TOGGLE_SWITCH, SWITCH_ONOFF_TOGGLE_CONTROL}
};

/**
 * @brief Допоміжна функція для встановлення кольору LED
 */
static void set_led_state(bool on)
{
    if (on) {
        /* Вмикаємо зелений колір (R:0, G:255, B:0) */
        led_strip_set_pixel(led_strip, 0, 0, 255, 0);
    } else {
        /* Вимикаємо (всі кольори в 0) */
        led_strip_clear(led_strip);
    }
    led_strip_refresh(led_strip);
}

/**
 * @brief Обробник подій Zigbee (Action Handler)
 */
static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID: {
        esp_zb_zcl_set_attr_value_message_t *set_attr_msg = (esp_zb_zcl_set_attr_value_message_t *)message;
        
        if (set_attr_msg->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF &&
            set_attr_msg->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID) {
            
            if (set_attr_msg->attribute.data.value) {
                bool light_state = *(bool *)set_attr_msg->attribute.data.value;
                ESP_LOGI(TAG, "Zigbee command: LED is now %s", light_state ? "ON" : "OFF");
                
                /* Викликаємо функцію для адресного LED */
                set_led_state(light_state);
            }
        }
        break;
    }
    default:
        ESP_LOGI(TAG, "Receive Zigbee action callback ID: 0x%x", callback_id);
        break;
    }
    return ret;
}

/**
 * @brief Обробник фізичної кнопки
 */
static void zb_buttons_handler(switch_func_pair_t *button_func_pair)
{
    if (button_func_pair->func == SWITCH_ONOFF_TOGGLE_CONTROL) {
        esp_zb_zcl_attr_t *attr = esp_zb_zcl_get_attribute(HA_ONOFF_SWITCH_ENDPOINT, 
                                                           ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, 
                                                           ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, 
                                                           ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID);
        
        if (attr && attr->data_p) {
            bool current_state = *(bool *)attr->data_p;
            bool new_state = !current_state;
            
            esp_zb_zcl_set_attribute_val(HA_ONOFF_SWITCH_ENDPOINT, 
                                       ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, 
                                       ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, 
                                       ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID, 
                                       &new_state, false);
            
            ESP_LOGI(TAG, "Physical button toggle: %d -> %d", current_state, new_state);
        }
    }
}

static esp_err_t deferred_driver_init(void)
{
    ESP_RETURN_ON_FALSE(switch_driver_init(button_func_pair, PAIR_SIZE(button_func_pair), zb_buttons_handler), 
                        ESP_FAIL, TAG, "Failed to initialize switch driver");
    return ESP_OK;
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p       = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Device started up %s", esp_zb_bdb_is_factory_new() ? "factory-new" : "rejoined");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Joined network successfully! PAN: 0x%04hx, Addr: 0x%04hx", 
                     esp_zb_get_pan_id(), esp_zb_get_short_address());
            deferred_driver_init();
        } else {
            vTaskDelay(pdMS_TO_TICKS(1000));
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        }
        break;
    default:
        break;
    }
}

static void esp_zb_task(void *pvParameters)
{
    /* Ініціалізація адресного світлодіода (led_strip) */
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_PIN,
        .max_leds = 1,
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    
    /* Очистка при старті */
    led_strip_clear(led_strip);
    led_strip_refresh(led_strip);
    ESP_LOGI(TAG, "LED Strip initialized on GPIO %d", LED_PIN);

    esp_zb_cfg_t zb_nwk_cfg = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,
        .install_code_policy = false,
        .nwk_cfg = {
            .zed_cfg = {
                .ed_timeout = 30,
                .keep_alive = 3000
            }
        },
    };
    esp_zb_init(&zb_nwk_cfg);
    
    esp_zb_on_off_light_cfg_t light_cfg = ESP_ZB_DEFAULT_ON_OFF_LIGHT_CONFIG();
    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_on_off_light_ep_create(HA_ONOFF_SWITCH_ENDPOINT, &light_cfg);
    
    zcl_basic_manufacturer_info_t info = {
        .manufacturer_name = ESP_MANUFACTURER_NAME,
        .model_identifier = ESP_MODEL_IDENTIFIER,
    };
    esp_zcl_utility_add_ep_basic_manufacturer_info(esp_zb_ep_list, HA_ONOFF_SWITCH_ENDPOINT, &info);
    
    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_device_register(esp_zb_ep_list);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}

void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}