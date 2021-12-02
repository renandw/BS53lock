#include <stdio.h>
#include <espressif/esp_wifi.h>
#include <espressif/esp_sta.h>
#include <espressif/esp_common.h>
#include <esp/uart.h>
#include <esp8266.h>
#include <FreeRTOS.h>
#include <task.h>
#include <etstimer.h>
#include <esplibs/libmain.h>

#include <homekit/homekit.h>
#include <homekit/characteristics.h>
#include <wifi_config.h>

#include <button.h>


//Período à espera da senha:
const int password_period = 5;
// The GPIO pin that is connected to a relay
const int relay_gpio = 12;
// The GPIO pin that is connected to a LED
const int led_gpio = 2;
// The GPIO pin that is connected to a button
const int password_gpio = 4;

#define BUTTON_PIN 0
#ifndef BUTTON_PIN
#error BUTTON_PIN is not specified
#endif

#define TOGGLE_PIN 12
#ifndef TOGGLE_PIN
#error TOGGLE_PIN is not specified
#endif

uint8_t toggle_press_count = 0;

// Timeout in seconds to open lock for
const int unlock_period = 5;  // 5 seconds
// Which signal to send to relay to open the lock (0 or 1)
const int relay_open_signal = 1;

ETSTimer password_timer, toggle_press_timer;

void lock_lock();
void lock_unlock();



void relay_write(int value) {
    gpio_write(relay_gpio, value ? 1 : 0);
}

void led_write(bool on) {
    gpio_write(led_gpio, on ? 0 : 1);
}

void password_write(bool on) {
    gpio_write(password_gpio, on ? 0 : 1);
}

void reset_configuration_task() {
    //Flash the LED first before we start the reset
    for (int i=0; i<3; i++) {
        led_write(true);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        led_write(false);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    printf("Resetting Wifi Config\n");
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
    printf("Resetting configuration\n");
    xTaskCreate(reset_configuration_task, "Reset configuration", 256, NULL, 2, NULL);
}

void gpio_init() {
    gpio_enable(led_gpio, GPIO_OUTPUT);
    led_write(false);

    gpio_enable(relay_gpio, GPIO_OUTPUT);
    relay_write(!relay_open_signal);

    gpio_enable(password_gpio, GPIO_OUTPUT);
    password_write(false);
}

void password_turnoff(){
    printf("Tempo acabou\n");
    sdk_os_timer_disarm(&password_timer);
    password_write(false);
}
void password_turnon(){
    printf("Tempo iniciou\n");
    password_write(true);
    sdk_os_timer_arm(&password_timer, password_period * 1000, 0);
}

void button_callback(button_event_t event, void* context) {
    switch (event) {
        case button_event_single_press:
            printf("apagando o led\n");
            password_turnon();
            break;
        case button_event_long_press:
            printf("acendendo o led\n");
            password_turnoff();
            break;
        default:
            printf("Unknown button event: %d\n", event);
    }
}

void lock_identify_task(void *_args) {
    // We identify the Sonoff by Flashing it's LED.
    for (int i=0; i<3; i++) {
        for (int j=0; j<2; j++) {
            led_write(true);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            led_write(false);
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }

        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
    led_write(false);
    vTaskDelete(NULL);
}

void lock_identify(homekit_value_t _value) {
    printf("Lock identify\n");
    xTaskCreate(lock_identify_task, "Lock identify", 128, NULL, 2, NULL);
}


typedef enum {
    lock_state_unsecured = 0,
    lock_state_secured = 1,
    lock_state_jammed = 2,
    lock_state_unknown = 3,
} lock_state_t;



homekit_characteristic_t name = HOMEKIT_CHARACTERISTIC_(NAME, "Lock");

homekit_characteristic_t lock_current_state = HOMEKIT_CHARACTERISTIC_(
    LOCK_CURRENT_STATE,
    lock_state_unknown,
);

void lock_target_state_setter(homekit_value_t value);

homekit_characteristic_t lock_target_state = HOMEKIT_CHARACTERISTIC_(
    LOCK_TARGET_STATE,
    lock_state_secured,
    .setter=lock_target_state_setter,
);

void lock_target_state_setter(homekit_value_t value) {
    lock_target_state.value = value;

    if (value.int_value == 0) {
        lock_unlock();
    } else {
        lock_lock();
    }
}

void lock_control_point(homekit_value_t value) {
    // Nothing to do here
}


void lock_lock() {
    relay_write(!relay_open_signal);
    led_write(false);
    if (lock_current_state.value.int_value != lock_state_secured) {
        lock_current_state.value = HOMEKIT_UINT8(lock_state_secured);
        homekit_characteristic_notify(&lock_current_state, lock_current_state.value);
    }
}

void lock_timeout() {
    if (lock_target_state.value.int_value != lock_state_secured) {
        lock_target_state.value = HOMEKIT_UINT8(lock_state_secured);
        homekit_characteristic_notify(&lock_target_state, lock_target_state.value);
    }

    lock_lock();
}

void lock_init() {
    lock_current_state.value = HOMEKIT_UINT8(lock_state_secured);
    homekit_characteristic_notify(&lock_current_state, lock_current_state.value);
}

void lock_unlock() {
    relay_write(relay_open_signal);
    led_write(true);
    lock_current_state.value = HOMEKIT_UINT8(lock_state_unsecured);
    homekit_characteristic_notify(&lock_current_state, lock_current_state.value);
}

//Função que conta os "apertos" de interruptores
void toggle_callback(bool high, void *context) {
        toggle_press_count++;
        printf("mais1\n");
        sdk_os_timer_arm(&toggle_press_timer, 3100, 0);
}

void toggle_action() {
        if (toggle_press_count > 10) {
                printf("pronto pra ativar\n");
                //aqui deve vir o código para ativar a fechadura;
                printf("Unlocking\n");
                relay_write(true);
                led_write(true);
                if (lock_target_state.value.int_value != lock_state_unsecured) {
                        lock_target_state.value = HOMEKIT_UINT8(lock_state_unsecured);
                        homekit_characteristic_notify(&lock_target_state, lock_target_state.value);
                        lock_current_state.value = HOMEKIT_UINT8(lock_state_unsecured);
                        homekit_characteristic_notify(&lock_current_state, lock_current_state.value);
                }}
                toggle_press_count=0;
                printf("zerou contagem\n");

}


homekit_accessory_t *accessories[] = {
    HOMEKIT_ACCESSORY(.id=1, .category=homekit_accessory_category_door_lock, .services=(homekit_service_t*[]){
        HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]){
            &name,
            HOMEKIT_CHARACTERISTIC(MANUFACTURER, "HaPK"),
            HOMEKIT_CHARACTERISTIC(SERIAL_NUMBER, "1"),
            HOMEKIT_CHARACTERISTIC(MODEL, "Basic"),
            HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "0.1"),
            HOMEKIT_CHARACTERISTIC(IDENTIFY, lock_identify),
            NULL
        }),
        HOMEKIT_SERVICE(LOCK_MECHANISM, .primary=true, .characteristics=(homekit_characteristic_t*[]){
            HOMEKIT_CHARACTERISTIC(NAME, "Lock"),
            &lock_current_state,
            &lock_target_state,
            NULL
        }),
        HOMEKIT_SERVICE(LOCK_MANAGEMENT, .characteristics=(homekit_characteristic_t*[]){
            HOMEKIT_CHARACTERISTIC(LOCK_CONTROL_POINT,
                .setter=lock_control_point
            ),
            HOMEKIT_CHARACTERISTIC(VERSION, "1"),
            NULL
        }),
        NULL
    }),
    NULL
};

homekit_server_config_t config = {
    .accessories = accessories,
    .password = "111-11-111"
};

void on_wifi_ready() {
    homekit_server_init(&config);
}

void create_accessory_name() {
    uint8_t macaddr[6];
    sdk_wifi_get_macaddr(STATION_IF, macaddr);

    int name_len = snprintf(NULL, 0, "Lock-%02X%02X%02X",
                            macaddr[3], macaddr[4], macaddr[5]);
    char *name_value = malloc(name_len+1);
    snprintf(name_value, name_len+1, "Lock-%02X%02X%02X",
             macaddr[3], macaddr[4], macaddr[5]);

    name.value = HOMEKIT_STRING(name_value);
}

void user_init(void) {
    uart_set_baud(0, 115200);

    create_accessory_name();

    wifi_config_init("lock", NULL, on_wifi_ready);
    gpio_init();
    lock_init();
    sdk_os_timer_setfn(&password_timer, password_turnoff, NULL);
    sdk_os_timer_setfn(&toggle_press_timer, toggle_action, NULL);

    button_config_t button_config = BUTTON_CONFIG(
            button_active_low,
            .max_repeat_presses=2,
            .long_press_time=3000,
            );
    if (button_create(BUTTON_PIN, button_config, button_callback, NULL)) {
            printf("Failed to initialize button\n");
    }
    if (toggle_create(TOGGLE_PIN, toggle_callback, NULL)) {
        printf("Failed to initialize sensor\n");
    }
}
