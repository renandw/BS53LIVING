#include <stdio.h>
#include <stdlib.h>
#include <espressif/esp_wifi.h>
#include <espressif/esp_sta.h>
#include <espressif/esp_common.h>
#include <esp/uart.h>
#include <esp8266.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <string.h>
#include <event_groups.h>
#include "i2c/i2c.h"
#include "bmp280/bmp280.h"
#include <etstimer.h>
#include <esplibs/libmain.h>
#include "fujitsu_ac_ir.h"

#include <homekit/homekit.h>
#include <homekit/characteristics.h>
#include <wifi_config.h>

#include <button.h>
#include <toggle.h>
#include <ota-api.h>

//BMP280 sensor setup
const uint8_t i2c_bus = 0;
const uint8_t scl_pin = 5;
const uint8_t sda_pin = 4;
#define TEMPERATURE_POLL_PERIOD 10000

// The GPIO pin that is connected to RELAY#1 on the board.
const int relay_gpio_1 = 0;
// The GPIO pin that is connected to RELAY#2 on the board.
const int relay_gpio_2 = 2;
// The GPIO pin that is connected to RELAY#3 on the board.
const int relay_gpio_3 = 16;


//Contact SENSORS PINS
//Sensor PIN
#define SENSOR_PIN 13
#ifndef SENSOR_PIN
#error SENSOR_PIN is not specified
#endif
//Sensor PIN 2
#define SENSOR_PIN_2 16
#ifndef SENSOR_PIN_2
#error SENSOR_PIN_2 is not specified
#endif

// The GPIO pin that is connected to the button1 on the Board.
#define BUTTON_PIN 3
#ifndef BUTTON_PIN
#error BUTTON_PIN is not specified
#endif

//The GPIO connected to RX_IR
#define IR_RX_GPIO 14


#define ALLOWED_FACTORY_RESET_TIME 60000

void lightbulb_on_1_callback(homekit_characteristic_t *_ch, homekit_value_t on, void *context);
void lightbulb_on_2_callback(homekit_characteristic_t *_ch, homekit_value_t on, void *context);
void lightbulb_on_3_callback(homekit_characteristic_t *_ch, homekit_value_t on, void *context);


homekit_characteristic_t name = HOMEKIT_CHARACTERISTIC_(NAME, "LivRain");
homekit_characteristic_t button_event = HOMEKIT_CHARACTERISTIC_(PROGRAMMABLE_SWITCH_EVENT, 0);
homekit_characteristic_t door_open_detected = HOMEKIT_CHARACTERISTIC_(CONTACT_SENSOR_STATE, 0);
homekit_characteristic_t door_open_detected_2 = HOMEKIT_CHARACTERISTIC_(CONTACT_SENSOR_STATE, 0);
homekit_characteristic_t ota_trigger  = API_OTA_TRIGGER;
homekit_characteristic_t manufacturer = HOMEKIT_CHARACTERISTIC_(MANUFACTURER,  "X");
homekit_characteristic_t serial       = HOMEKIT_CHARACTERISTIC_(SERIAL_NUMBER, "1");
homekit_characteristic_t model        = HOMEKIT_CHARACTERISTIC_(MODEL,         "Z");
homekit_characteristic_t revision     = HOMEKIT_CHARACTERISTIC_(FIRMWARE_REVISION,  "0.0.0");
homekit_characteristic_t rain_event = HOMEKIT_CHARACTERISTIC_(LEAK_DETECTED, 0);
homekit_characteristic_t occupancy_detected = HOMEKIT_CHARACTERISTIC_(OCCUPANCY_DETECTED, 0);
void relay_write_1(bool on) {
        gpio_write(relay_gpio_1, on ? 0 : 1);
}

void relay_write_2(bool on) {
        gpio_write(relay_gpio_2, on ? 0 : 1);
}

void relay_write_3(bool on) {
        gpio_write(relay_gpio_3, on ? 0 : 1);
}

void thermostat_on_callback(homekit_characteristic_t *_ch, homekit_value_t on, void *context);

void reset_configuration_task() {
        printf("Resetting Wifi");
        wifi_config_reset();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        printf("Resetting HomeKit Config\n");
        homekit_server_reset();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        printf("Restarting\n");
        sdk_system_restart();
        vTaskDelete(NULL);
}

void reset_configuration() {
        if (xTaskGetTickCountFromISR() < ALLOWED_FACTORY_RESET_TIME / portTICK_PERIOD_MS) {
                xTaskCreate(reset_configuration_task, "Reset configuration", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
        } else {
                printf("Factory reset not allowed after %ims since boot. Repower device and try again\n", ALLOWED_FACTORY_RESET_TIME);
        }
}

//IR and BMP SENSOR Section %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#define MIN(a, b) (((a) <= (b)) ? (a) : (b))
#define MAX(a, b) (((a) >= (b)) ? (a) : (b))
fujitsu_ac_state_t ac_state;

EventGroupHandle_t sync_flags;
#define SYNC_FLAGS_UPDATE (1 << 0)

void update_state();


void on_update(homekit_characteristic_t *ch, homekit_value_t value, void *context) {
        update_state();
}


homekit_characteristic_t current_humidity = HOMEKIT_CHARACTERISTIC_(CURRENT_RELATIVE_HUMIDITY, 0);
homekit_characteristic_t current_temperature = HOMEKIT_CHARACTERISTIC_(CURRENT_TEMPERATURE, 0);
homekit_characteristic_t target_temperature  = HOMEKIT_CHARACTERISTIC_(
        TARGET_TEMPERATURE, 22,
        .min_value = (float[]) {AC_MIN_TEMPERATURE},
        .max_value = (float[]) {AC_MAX_TEMPERATURE},
        .callback=HOMEKIT_CHARACTERISTIC_CALLBACK(on_update),
        );
homekit_characteristic_t units = HOMEKIT_CHARACTERISTIC_(TEMPERATURE_DISPLAY_UNITS, 0);
homekit_characteristic_t current_state = HOMEKIT_CHARACTERISTIC_(CURRENT_HEATING_COOLING_STATE, 0);
homekit_characteristic_t target_state = HOMEKIT_CHARACTERISTIC_(
        TARGET_HEATING_COOLING_STATE, 0,
        .callback=HOMEKIT_CHARACTERISTIC_CALLBACK(on_update)
        );

uint8_t fan = 0;

void fan_active_set(homekit_value_t value) {
        // fan = value.bool_value;
        update_state();
}

homekit_characteristic_t fan_active = HOMEKIT_CHARACTERISTIC_(ACTIVE, 0, .setter=fan_active_set);
homekit_characteristic_t fan_rotation_speed = HOMEKIT_CHARACTERISTIC_(
        ROTATION_SPEED, 0,
        .callback=HOMEKIT_CHARACTERISTIC_CALLBACK(on_update)
        );
homekit_characteristic_t fan_swing_mode = HOMEKIT_CHARACTERISTIC_(
        SWING_MODE, 0,
        .callback=HOMEKIT_CHARACTERISTIC_CALLBACK(on_update)
        );


void update_state() {
        if ((xEventGroupGetBits(sync_flags) & SYNC_FLAGS_UPDATE) == 0)
                return;

        homekit_value_t new_current_state,
                        new_fan_active = HOMEKIT_UINT8(1),
                        new_rotation_speed = fan_rotation_speed.value;

        fujitsu_ac_state_t new_ac_state = ac_state;
        new_ac_state.command = ac_cmd_turn_on;

        switch (target_state.value.int_value) {
        case HOMEKIT_TARGET_HEATING_COOLING_STATE_HEAT:
                new_ac_state.mode = ac_mode_heat;
                new_current_state = HOMEKIT_UINT8(HOMEKIT_CURRENT_HEATING_COOLING_STATE_HEAT);
                break;

        case HOMEKIT_TARGET_HEATING_COOLING_STATE_COOL:
                new_ac_state.mode = ac_mode_cool;
                new_current_state = HOMEKIT_UINT8(HOMEKIT_CURRENT_HEATING_COOLING_STATE_COOL);
                break;

        case HOMEKIT_TARGET_HEATING_COOLING_STATE_AUTO:
                new_ac_state.mode = ac_mode_auto;
                if (current_temperature.value.int_value < target_temperature.value.int_value) {
                        new_current_state = HOMEKIT_UINT8(HOMEKIT_CURRENT_HEATING_COOLING_STATE_HEAT);
                } else {
                        new_current_state = HOMEKIT_UINT8(HOMEKIT_CURRENT_HEATING_COOLING_STATE_COOL);
                }
                break;

        case HOMEKIT_TARGET_HEATING_COOLING_STATE_OFF:
                if (fan) {
                        new_ac_state.mode = ac_mode_fan;
                        new_current_state = HOMEKIT_UINT8(HOMEKIT_CURRENT_HEATING_COOLING_STATE_OFF);
                        break;
                }

        default:
                new_ac_state.mode = ac_mode_auto;
                new_ac_state.command = ac_cmd_turn_off;
                new_current_state = HOMEKIT_UINT8(HOMEKIT_CURRENT_HEATING_COOLING_STATE_OFF);
                new_fan_active = HOMEKIT_UINT8(0);
        }

        if (ac_state.command == ac_cmd_turn_off &&
            new_ac_state.command != ac_cmd_turn_off &&
            new_ac_state.mode != ac_mode_fan)
        {
                new_ac_state.fan = ac_fan_auto;
                new_rotation_speed = HOMEKIT_FLOAT(100);
        } else {
                uint8_t rotation_speed = (uint8_t)fan_rotation_speed.value.float_value;
                if (rotation_speed > 99) {
                        new_ac_state.fan = ac_fan_auto;
                } else if (rotation_speed > 75) {
                        new_ac_state.fan = ac_fan_high;
                } else if (rotation_speed > 45) {
                        new_ac_state.fan = ac_fan_med;
                } else if (rotation_speed > 15) {
                        new_ac_state.fan = ac_fan_low;
                } else {
                        new_ac_state.fan = ac_fan_low;
                }
        }

        new_ac_state.temperature = MIN(AC_MAX_TEMPERATURE, MAX(AC_MIN_TEMPERATURE, target_temperature.value.float_value));
        new_ac_state.swing = fan_swing_mode.value.int_value ? ac_swing_vert : ac_swing_off;

        int result = fujitsu_ac_ir_send(&new_ac_state);
        if (result < 0) {
                printf("Fujitsu command send failed (code %d)\n", result);
                return;
        }

        ac_state = new_ac_state;

        if (!homekit_value_equal(&new_current_state, &current_state.value)) {
                current_state.value = new_current_state;
                homekit_characteristic_notify(&current_state, current_state.value);
        }

        if (!homekit_value_equal(&new_fan_active, &fan_active.value)) {
                fan_active.value = new_fan_active;
                homekit_characteristic_notify(&fan_active, fan_active.value);
        }

        if (!homekit_value_equal(&new_rotation_speed, &fan_rotation_speed.value)) {
                fan_rotation_speed.value = new_rotation_speed;
                homekit_characteristic_notify(&fan_rotation_speed, fan_rotation_speed.value);
        }
}


#ifdef MODE_FORCED
static void bmp280_task_forced(void *pvParameters)
{
        bmp280_params_t params;
        float pressure, temperature, humidity;

        bmp280_init_default_params(&params);
        params.mode = BMP280_MODE_FORCED;

        bmp280_t bmp280_dev;
        bmp280_dev.i2c_dev.bus = i2c_bus;
        bmp280_dev.i2c_dev.addr = BMP280_I2C_ADDRESS_0;

        while (1) {
                while (!bmp280_init(&bmp280_dev, &params)) {
                        printf("BMP280 initialization failed\n");
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                }

                bool bme280p = bmp280_dev.id == BME280_CHIP_ID;
                printf("BMP280: found %s\n", bme280p ? "BME280" : "BMP280");

                while(1) {
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                        if (!bmp280_force_measurement(&bmp280_dev)) {
                                printf("Failed initiating measurement\n");
                                break;
                        }
                        // wait for measurement to complete
                        while (bmp280_is_measuring(&bmp280_dev)) {};

                        if (!bmp280_read_float(&bmp280_dev, &temperature, &pressure, &humidity)) {
                                printf("Temperature/pressure reading failed\n");
                                break;
                        }
                        printf("Pressure: %.2f Pa, Temperature: %.2f C", pressure, temperature);
                        if (bme280p)
                                printf(", Humidity: %.2f\n", humidity);
                        else
                                printf("\n");

                        current_temperature.value = HOMEKIT_FLOAT(temperature);
                        current_humidity.value = HOMEKIT_FLOAT(humidity);

                        homekit_characteristic_notify(&current_temperature, current_temperature.value);
                        homekit_characteristic_notify(&current_humidity, current_humidity.value);
                        if (target_state.value.int_value == HOMEKIT_TARGET_HEATING_COOLING_STATE_AUTO) {
                                homekit_value_t new_current_state;
                                if (current_temperature.value.int_value < target_temperature.value.int_value) {
                                        new_current_state = HOMEKIT_UINT8(HOMEKIT_CURRENT_HEATING_COOLING_STATE_HEAT);
                                } else {
                                        new_current_state = HOMEKIT_UINT8(HOMEKIT_CURRENT_HEATING_COOLING_STATE_COOL);
                                }
                                if (!homekit_value_equal(&new_current_state, &current_state.value)) {
                                        current_state.value = new_current_state;
                                        homekit_characteristic_notify(&current_state, current_state.value);
                                }
                        }
                }
        }
}
#else
static void bmp280_task_normal(void *pvParameters)
{
        bmp280_params_t params;
        float pressure, temperature, humidity;

        bmp280_init_default_params(&params);

        bmp280_t bmp280_dev;
        bmp280_dev.i2c_dev.bus = i2c_bus;
        bmp280_dev.i2c_dev.addr = BMP280_I2C_ADDRESS_0;

        while (1) {
                while (!bmp280_init(&bmp280_dev, &params)) {
                        printf("BMP280 initialization failed\n");
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                }

                bool bme280p = bmp280_dev.id == BME280_CHIP_ID;
                printf("BMP280: found %s\n", bme280p ? "BME280" : "BMP280");

                while(1) {
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                        if (!bmp280_read_float(&bmp280_dev, &temperature, &pressure, &humidity)) {
                                printf("Temperature/pressure reading failed\n");
                                break;
                        }
                        printf("Pressure: %.2f Pa, Temperature: %.2f C", pressure, temperature);
                        if (bme280p)
                                printf(", Humidity: %.2f\n", humidity);
                        else
                                printf("\n");

                        current_temperature.value = HOMEKIT_FLOAT(temperature);
                        current_humidity.value = HOMEKIT_FLOAT(humidity);
                        vTaskDelay(TEMPERATURE_POLL_PERIOD / portTICK_PERIOD_MS);

                        homekit_characteristic_notify(&current_temperature, current_temperature.value);
                        homekit_characteristic_notify(&current_humidity, current_humidity.value);
                        if (target_state.value.int_value == HOMEKIT_TARGET_HEATING_COOLING_STATE_AUTO) {
                                homekit_value_t new_current_state;
                                if (current_temperature.value.int_value < target_temperature.value.int_value) {
                                        new_current_state = HOMEKIT_UINT8(HOMEKIT_CURRENT_HEATING_COOLING_STATE_HEAT);
                                } else {
                                        new_current_state = HOMEKIT_UINT8(HOMEKIT_CURRENT_HEATING_COOLING_STATE_COOL);
                                }
                                if (!homekit_value_equal(&new_current_state, &current_state.value)) {
                                        current_state.value = new_current_state;
                                        homekit_characteristic_notify(&current_state, current_state.value);
                                }
                        }
                }

        }
}
#endif

void temperature_sensor_init() {

        i2c_init(i2c_bus, scl_pin, sda_pin, I2C_FREQ_400K);

#ifdef MODE_FORCED
        xTaskCreate(bmp280_task_forced, "bmp280_task", 512, NULL, 2, NULL);
#else
        xTaskCreate(bmp280_task_normal, "bmp280_task", 512, NULL, 2, NULL);
#endif

}


void ir_rx_task(void *_args) {
        printf("Running IR task\n");
        ir_decoder_t *decoder = fujitsu_ac_ir_make_decoder();

        fujitsu_ac_state_t state;
        while (true) {
                int size = ir_recv(decoder, 0, &state, sizeof(state));
                if (size < 0) {
                        printf("Bit decoding failed\n");
                        continue;
                }

                xEventGroupClearBits(sync_flags, SYNC_FLAGS_UPDATE);

                ac_state = state;

                homekit_value_t new_target_state, new_fan_active;
                if (state.command == ac_cmd_turn_off) {
                        fan = 0;
                        new_target_state = HOMEKIT_UINT8(0);
                        new_fan_active = HOMEKIT_UINT8(0);
                } else if (state.command == ac_cmd_turn_on) {
                        fan = 0;
                        switch (state.mode) {
                        case ac_mode_heat:
                                new_target_state = HOMEKIT_UINT8(1);
                                break;
                        case ac_mode_cool:
                                new_target_state = HOMEKIT_UINT8(2);
                                break;
                        case ac_mode_auto:
                                new_target_state = HOMEKIT_UINT8(3);
                                break;
                        case ac_mode_dry:
                        case ac_mode_fan:
                                new_target_state = HOMEKIT_UINT8(0);
                                break;
                        }

                        new_fan_active = HOMEKIT_UINT8(1);

                        homekit_value_t new_target_temperature = HOMEKIT_FLOAT(state.temperature);
                        homekit_value_t new_fan_rotation_speed;
                        switch (state.fan) {
                        case ac_fan_auto:
                        case ac_fan_high:
                                new_fan_rotation_speed = HOMEKIT_FLOAT(100);
                                break;
                        case ac_fan_med:
                                new_fan_rotation_speed = HOMEKIT_FLOAT(75);
                                break;
                        case ac_fan_low:
                                new_fan_rotation_speed = HOMEKIT_FLOAT(45);
                                break;
                        }
                        homekit_value_t new_fan_swing_mode = HOMEKIT_UINT8((state.swing == ac_swing_off) ? 0 : 1);

                        if (!homekit_value_equal(&new_target_temperature, &target_temperature.value)) {
                                target_temperature.value = new_target_temperature;
                                homekit_characteristic_notify(&target_temperature, target_temperature.value);
                        }

                        if (!homekit_value_equal(&new_fan_rotation_speed, &fan_rotation_speed.value)) {
                                fan_rotation_speed.value = new_fan_rotation_speed;
                                homekit_characteristic_notify(&fan_rotation_speed, fan_rotation_speed.value);
                        }

                        if (!homekit_value_equal(&new_fan_swing_mode, &fan_swing_mode.value)) {
                                fan_swing_mode.value = new_fan_swing_mode;
                                homekit_characteristic_notify(&fan_swing_mode, fan_swing_mode.value);
                        }
                }

                if (!homekit_value_equal(&new_target_state, &target_state.value)) {
                        target_state.value = new_target_state;
                        homekit_characteristic_notify(&target_state, target_state.value);
                }

                if (!homekit_value_equal(&new_fan_active, &fan_active.value)) {
                        fan_active.value = new_fan_active;
                        homekit_characteristic_notify(&fan_active, fan_active.value);
                }

                xEventGroupSetBits(sync_flags, SYNC_FLAGS_UPDATE);
        }

        decoder->free(decoder);

        vTaskDelete(NULL);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

homekit_characteristic_t lightbulb_on_1 = HOMEKIT_CHARACTERISTIC_(
        ON, false, .callback=HOMEKIT_CHARACTERISTIC_CALLBACK(lightbulb_on_1_callback)
        );

homekit_characteristic_t lightbulb_on_2 = HOMEKIT_CHARACTERISTIC_(
        ON, false, .callback=HOMEKIT_CHARACTERISTIC_CALLBACK(lightbulb_on_2_callback)
        );

homekit_characteristic_t lightbulb_on_3 = HOMEKIT_CHARACTERISTIC_(
        ON, false, .callback=HOMEKIT_CHARACTERISTIC_CALLBACK(lightbulb_on_3_callback)
        );




void gpio_init() {
        gpio_enable(relay_gpio_1, GPIO_OUTPUT);
        relay_write_1(lightbulb_on_1.value.bool_value);

        gpio_enable(relay_gpio_2, GPIO_OUTPUT);
        relay_write_2(lightbulb_on_2.value.bool_value);

        gpio_enable(relay_gpio_3, GPIO_OUTPUT);
        relay_write_3(lightbulb_on_3.value.bool_value);

        gpio_enable(SENSOR_PIN, GPIO_INPUT);
        gpio_enable(SENSOR_PIN_2, GPIO_INPUT);
}

void lightbulb_on_1_callback(homekit_characteristic_t *_ch, homekit_value_t on, void *context) {
        relay_write_1(lightbulb_on_1.value.bool_value);
}

void lightbulb_on_2_callback(homekit_characteristic_t *_ch, homekit_value_t on, void *context) {
        relay_write_2(lightbulb_on_2.value.bool_value);
}

void lightbulb_on_3_callback(homekit_characteristic_t *_ch, homekit_value_t on, void *context) {
        relay_write_3(lightbulb_on_3.value.bool_value);
}


void toggle_callback_1(bool high, void *context) {
        printf("toggle is %s\n", high ? "high" : "low");
        lightbulb_on_1.value.bool_value = !lightbulb_on_1.value.bool_value;
        relay_write_1(lightbulb_on_1.value.bool_value);
        homekit_characteristic_notify(&lightbulb_on_1, lightbulb_on_1.value);
}

void toggle_callback_2(bool high, void *context) {
        printf("toggle is %s\n", high ? "high" : "low");
        lightbulb_on_2.value.bool_value = !lightbulb_on_2.value.bool_value;
        relay_write_2(lightbulb_on_2.value.bool_value);
        homekit_characteristic_notify(&lightbulb_on_2, lightbulb_on_2.value);
}

void toggle_callback_3(bool high, void *context) {
        printf("toggle is %s\n", high ? "high" : "low");
        lightbulb_on_3.value.bool_value = !lightbulb_on_3.value.bool_value;
        relay_write_3(lightbulb_on_3.value.bool_value);
        homekit_characteristic_notify(&lightbulb_on_3, lightbulb_on_3.value);
        reset_configuration();
}



void light_identify(homekit_value_t _value) {
        printf("Light identify\n");
}

void door_identify(homekit_value_t _value) {
        printf("Occupancy identify\n");
}

void button_identify(homekit_value_t _value) {
        printf("Skimmer Control identify\n");
}

void thermostat_identify(homekit_value_t _value) {
        printf("Skimmer Control identify\n");
}

//button
void button_callback(button_event_t event, void *context) {
        switch (event) {
        case button_event_single_press:
                printf("single press\n");
                homekit_characteristic_notify(&button_event, HOMEKIT_UINT8(0));
                break;
        case button_event_double_press:
                printf("double press\n");
                homekit_characteristic_notify(&button_event, HOMEKIT_UINT8(1));
                break;
        case button_event_long_press:
                printf("long press\n");
                homekit_characteristic_notify(&button_event, HOMEKIT_UINT8(2));
                break;
        default:
                printf("unknown button event: %d\n", event);
        }
}

// contact sensor
void sensor_callback(bool high, void *context) {
        door_open_detected.value = HOMEKIT_UINT8(high ? 1 : 0);
        homekit_characteristic_notify(&door_open_detected, door_open_detected.value);
}

void sensor_callback_2(bool high, void *context) {
        door_open_detected_2.value = HOMEKIT_UINT8(high ? 1 : 0);
        homekit_characteristic_notify(&door_open_detected_2, door_open_detected_2.value);
}

homekit_accessory_t *accessories[] = {
        HOMEKIT_ACCESSORY(.id=1, .category=homekit_accessory_category_switch, .services=(homekit_service_t*[]){
                HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]){
                        HOMEKIT_CHARACTERISTIC(IDENTIFY, light_identify),
                        &name,
                        &manufacturer,
                        &serial,
                        &model,
                        &revision,
                        NULL
                }),

                HOMEKIT_SERVICE(LIGHTBULB, .primary=true, .characteristics=(homekit_characteristic_t*[]){
                        HOMEKIT_CHARACTERISTIC(NAME, "Lâmpada"),
                        &lightbulb_on_1,
                        &ota_trigger,
                        NULL
                }),
                NULL,
        }),

        HOMEKIT_ACCESSORY(.id=2, .category=homekit_accessory_category_switch, .services=(homekit_service_t*[]){
                HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]){
                        HOMEKIT_CHARACTERISTIC(IDENTIFY, light_identify),
                        HOMEKIT_CHARACTERISTIC(NAME, "Spots"),
                        &manufacturer,
                        &serial,
                        &model,
                        &revision,
                        NULL
                }),

                HOMEKIT_SERVICE(LIGHTBULB, .characteristics=(homekit_characteristic_t*[]){
                        HOMEKIT_CHARACTERISTIC(NAME, "Spots"),
                        &lightbulb_on_2,
                        NULL
                }),
                NULL,
        }),

        HOMEKIT_ACCESSORY(.id=3, .category=homekit_accessory_category_switch, .services=(homekit_service_t*[]){
                HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]){
                        HOMEKIT_CHARACTERISTIC(IDENTIFY, light_identify),
                        HOMEKIT_CHARACTERISTIC(NAME, "Lâmpadas"),
                        &manufacturer,
                        &serial,
                        &model,
                        &revision,
                        NULL
                }),
                HOMEKIT_SERVICE(LIGHTBULB, .characteristics=(homekit_characteristic_t*[]){
                        HOMEKIT_CHARACTERISTIC(NAME, "Lâmpadas"),
                        &lightbulb_on_3,
                        NULL
                }),
                NULL,
        }),

        HOMEKIT_ACCESSORY(.id=4, .category=homekit_accessory_category_thermostat, .services=(homekit_service_t*[]){
                HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]){
                        HOMEKIT_CHARACTERISTIC(NAME, "Termostato"),
                        &manufacturer,
                        &serial,
                        &model,
                        &revision,
                        HOMEKIT_CHARACTERISTIC(IDENTIFY, thermostat_identify),
                        NULL
                }),
                HOMEKIT_SERVICE(THERMOSTAT, .characteristics=(homekit_characteristic_t*[]) {
                        HOMEKIT_CHARACTERISTIC(NAME, "Termostato"),
                        &current_humidity,
                        &current_temperature,
                        &target_temperature,
                        &current_state,
                        &target_state,
                        &units,
                        NULL
                }),
                HOMEKIT_SERVICE(FAN2, .characteristics=(homekit_characteristic_t*[]) {
                        HOMEKIT_CHARACTERISTIC(NAME, "Fan"),
                        &fan_active,
                        &fan_rotation_speed,
                        &fan_swing_mode,
                        NULL
                }),
                NULL
        }),
        HOMEKIT_ACCESSORY(.id=5, .category=homekit_accessory_category_sensor, .services=(homekit_service_t*[]) {
                HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]) {
                        HOMEKIT_CHARACTERISTIC(NAME, "Contact Sensor"),
                        &manufacturer,
                        &serial,
                        &model,
                        &revision,
                        HOMEKIT_CHARACTERISTIC(IDENTIFY, door_identify),
                        NULL
                }),
                HOMEKIT_SERVICE(CONTACT_SENSOR, .characteristics=(homekit_characteristic_t*[]) {
                        HOMEKIT_CHARACTERISTIC(NAME, "Contact Sensor"),
                        &door_open_detected,
                        NULL
                }),
                NULL
        }),
        HOMEKIT_ACCESSORY(.id=6, .category=homekit_accessory_category_sensor, .services=(homekit_service_t*[]) {
                HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]) {
                        HOMEKIT_CHARACTERISTIC(NAME, "Contact Sensor2"),
                        &manufacturer,
                        &serial,
                        &model,
                        &revision,
                        HOMEKIT_CHARACTERISTIC(IDENTIFY, door_identify),
                        NULL
                }),
                HOMEKIT_SERVICE(CONTACT_SENSOR, .characteristics=(homekit_characteristic_t*[]) {
                        HOMEKIT_CHARACTERISTIC(NAME, "Contact Sensor2"),
                        &door_open_detected_2,
                        NULL
                }),
                NULL
        }),
        HOMEKIT_ACCESSORY(.id=7, .category=homekit_accessory_category_programmable_switch, .services=(homekit_service_t*[]) {
                HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]) {
                        HOMEKIT_CHARACTERISTIC(NAME, "Botão Inteligente"),
                        &manufacturer,
                        &serial,
                        &model,
                        &revision,
                        HOMEKIT_CHARACTERISTIC(IDENTIFY, button_identify),
                        NULL
                }),
                HOMEKIT_SERVICE(STATELESS_PROGRAMMABLE_SWITCH, .characteristics=(homekit_characteristic_t*[]) {
                        HOMEKIT_CHARACTERISTIC(NAME, "Botão Inteligente"),
                        &button_event,
                        NULL
                }),
                NULL
        }),
        NULL
};

homekit_server_config_t config = {
        .accessories = accessories,
        .password = "111-11-111" //ci=9
};

void on_wifi_ready() {
        homekit_server_init(&config);
}

bool initialized = false;

void init() {
        sync_flags = xEventGroupCreate();
        xEventGroupSetBits(sync_flags, SYNC_FLAGS_UPDATE);

        ac_state.command = ac_cmd_turn_off;
        ac_state.temperature = 22;
        ac_state.mode = ac_mode_auto;

        ac_state.fan = ac_fan_auto;
        ac_state.swing = ac_swing_off;

        fujitsu_ac_ir_tx_init(fujitsu_ac_model_ARRAH2E);
        ir_rx_init(IR_RX_GPIO, 300);
        update_state();

        xTaskCreate(ir_rx_task, "IR receiver", 1024, NULL, 2, NULL);

        initialized = true;
}

void on_homekit_event(homekit_event_t event) {
        if (event == HOMEKIT_EVENT_PAIRING_ADDED) {
                if (!initialized) {
                        init();
                }
        } else if (event == HOMEKIT_EVENT_PAIRING_REMOVED) {
                if (!homekit_is_paired()) {
                        printf("Restarting\n");
                        sdk_system_restart();
                }
        }
}

void create_accessory_name() {
        uint8_t macaddr[6];
        sdk_wifi_get_macaddr(STATION_IF, macaddr);

        int name_len = snprintf(NULL, 0, "elgin AC-%02X%02X%02X",
                                macaddr[3], macaddr[4], macaddr[5]);
        char *name_value = malloc(name_len+1);
        snprintf(name_value, name_len+1, "elgin AC-%02X%02X%02X",
                 macaddr[3], macaddr[4], macaddr[5]);

        name.value = HOMEKIT_STRING(name_value);
}

void user_init(void) {
        uart_set_baud(0, 115200);
        temperature_sensor_init();
        gpio_init();
        create_accessory_name();

        wifi_config_init("elgin-ac", NULL, on_wifi_ready);

        button_config_t config = BUTTON_CONFIG(
                button_active_low,
                .long_press_time = 3000,
                .max_repeat_presses = 3,
                );


        if (button_create(BUTTON_PIN, config, button_callback, NULL)) {
                printf("Failed to initialize button\n");
        }

        if (toggle_create(SENSOR_PIN, sensor_callback, NULL)) {
                printf("Failed to initialize sensor\n");
        }
        if (toggle_create(SENSOR_PIN_2, sensor_callback_2, NULL)) {
                printf("Failed to initialize sensor\n");
        }

        if (homekit_is_paired()) {
                init();
        }
}
