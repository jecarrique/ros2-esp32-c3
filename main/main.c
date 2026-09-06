/**
 * @file main.c
 * @brief Application entry point: connects to Wi-Fi, brings up the
 *        micro-ROS UDP transport against a ROS 2 Jazzy micro-ROS agent,
 *        and publishes:
 *          - /imu/data   (sensor_msgs/msg/Imu)   from the FXOS8700/FXAS21002
 *          - /button/state (std_msgs/msg/Bool)   from a push button on GPIO5
 */
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "rcl/rcl.h"
#include "rcl/error_handling.h"
#include "rclc/rclc.h"
#include "rclc/executor.h"

#include <rmw_microros/rmw_microros.h>

#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/bool.h>
#include <rosidl_runtime_c/string_functions.h>

#include "fxos8700_fxas21002.h"

/* ---------------------------------------------------------------------- */
/*  GPIO / hardware configuration                                          */
/* ---------------------------------------------------------------------- */
#define I2C_PORT            I2C_NUM_0
#define I2C_SDA_GPIO         7
#define I2C_SCL_GPIO         6
#define I2C_CLK_SPEED_HZ     400000

#define BUTTON_GPIO          GPIO_NUM_5

/* ---------------------------------------------------------------------- */
/*  Publication rate                                                       */
/* ---------------------------------------------------------------------- */
#define PUBLISH_PERIOD_MS    100  /* 10 Hz */

static const char *TAG = "ros2_esp32_c3";

#define RCCHECK(fn) do { \
    rcl_ret_t rc = (fn); \
    if (rc != RCL_RET_OK) { \
        ESP_LOGE(TAG, "Failed status on line %d: %d. Aborting.", __LINE__, (int)rc); \
        vTaskDelete(NULL); \
    } \
} while (0)

#define RCSOFTCHECK(fn) do { \
    rcl_ret_t rc = (fn); \
    if (rc != RCL_RET_OK) { \
        ESP_LOGW(TAG, "Failed status on line %d: %d. Continuing.", __LINE__, (int)rc); \
    } \
} while (0)

/* ---------------------------------------------------------------------- */
/*  Wi-Fi connection handling                                               */
/* ---------------------------------------------------------------------- */
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT   BIT0

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "Wi-Fi disconnected, retrying...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = { 0 };
    /* NOTE: WIFI_SSID/WIFI_PASSWORD come from Kconfig (see main/Kconfig.projbuild).
     * Do not commit real credentials to sdkconfig.defaults; the generated
     * `sdkconfig` file (which may contain real values) is already gitignored. */
    strncpy((char *)wifi_config.sta.ssid, CONFIG_WIFI_SSID, sizeof(wifi_config.sta.ssid) - 1);
    wifi_config.sta.ssid[sizeof(wifi_config.sta.ssid) - 1] = '\0';
    strncpy((char *)wifi_config.sta.password, CONFIG_WIFI_PASSWORD, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.password[sizeof(wifi_config.sta.password) - 1] = '\0';
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connecting to Wi-Fi SSID '%s'...", CONFIG_WIFI_SSID);
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
}

/* ---------------------------------------------------------------------- */
/*  micro-ROS application task                                             */
/* ---------------------------------------------------------------------- */
static void micro_ros_task(void *arg)
{
    /* Configure and initialize the button GPIO with an internal pull-up. */
    gpio_config_t button_cfg = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&button_cfg));

    /* Configure and initialize the IMU over I2C. */
    imu_config_t imu_cfg = {
        .i2c_port = I2C_PORT,
        .sda_gpio = I2C_SDA_GPIO,
        .scl_gpio = I2C_SCL_GPIO,
        .clk_speed_hz = I2C_CLK_SPEED_HZ,
    };
    esp_err_t imu_err = imu_init(&imu_cfg);
    if (imu_err != ESP_OK) {
        ESP_LOGE(TAG, "IMU initialization failed (0x%x). Continuing without IMU data.", imu_err);
    }

    /* Configure the micro-ROS UDP transport against the configured agent. */
    char agent_port_str[8];
    int port_len = snprintf(agent_port_str, sizeof(agent_port_str), "%d", CONFIG_MICRO_ROS_AGENT_PORT);
    if (port_len < 0 || (size_t)port_len >= sizeof(agent_port_str)) {
        ESP_LOGE(TAG, "Invalid micro-ROS agent port configuration");
        vTaskDelete(NULL);
        return;
    }

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    if (rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, agent_port_str, rmw_options) != RMW_RET_OK) {
        ESP_LOGE(TAG, "Failed to set micro-ROS agent UDP address %s:%s",
                 CONFIG_MICRO_ROS_AGENT_IP, agent_port_str);
        vTaskDelete(NULL);
        return;
    }

    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "esp32c3_node", "", &support));

    rcl_publisher_t imu_publisher;
    RCCHECK(rclc_publisher_init_default(
        &imu_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data"));

    rcl_publisher_t button_publisher;
    RCCHECK(rclc_publisher_init_default(
        &button_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "button/state"));

    sensor_msgs__msg__Imu imu_msg;
    sensor_msgs__msg__Imu__init(&imu_msg);
    /* Orientation is not provided by this IMU; mark it as unknown per the
     * sensor_msgs/Imu convention (orientation_covariance[0] = -1). */
    imu_msg.orientation_covariance[0] = -1.0;
    rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "imu_link");

    std_msgs__msg__Bool button_msg;
    std_msgs__msg__Bool__init(&button_msg);

    ESP_LOGI(TAG, "micro-ROS node started. Publishing /imu/data and /button/state at %d ms.",
             PUBLISH_PERIOD_MS);

    const TickType_t period_ticks = pdMS_TO_TICKS(PUBLISH_PERIOD_MS);
    while (1) {
        imu_sample_t sample = { 0 };
        if (imu_read(&sample) == ESP_OK) {
            imu_msg.linear_acceleration.x = sample.accel.x;
            imu_msg.linear_acceleration.y = sample.accel.y;
            imu_msg.linear_acceleration.z = sample.accel.z;
            imu_msg.angular_velocity.x = sample.gyro.x;
            imu_msg.angular_velocity.y = sample.gyro.y;
            imu_msg.angular_velocity.z = sample.gyro.z;
            RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
        }

        /* Button is wired to GND when pressed (active low), VCC at rest. */
        int level = gpio_get_level(BUTTON_GPIO);
        button_msg.data = (level == 0);
        RCSOFTCHECK(rcl_publish(&button_publisher, &button_msg, NULL));

        vTaskDelay(period_ticks);
    }

    /* Unreachable, but kept for completeness/cleanup symmetry. */
    RCSOFTCHECK(rcl_publisher_fini(&imu_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&button_publisher, &node));
    RCSOFTCHECK(rcl_node_fini(&node));
    vTaskDelete(NULL);
}

void app_main(void)
{
    /* Initialize NVS, required by esp_wifi. */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_sta();

    xTaskCreate(micro_ros_task, "micro_ros_task", 8192, NULL, 5, NULL);
}
