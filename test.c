/*
   Acessório para controlar uma fechadura magnética.
   Features: botão programável; sensor de contato
   Maiores possibilidades para automação do acessório.
 */

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

#include "ota-api.h"

const int relay_gpio = 13;
const int password_gpio = 5;
const int led_gpio = 2;

uint8_t toggle_press_count = 0;

//BOTÃO DE CONTROLE DENTRO DE CASA
#define BUTTON_PIN 0
#ifndef BUTTON_PIN
#error BUTTON_PIN is not specified
#endif

//CONTROLE DO PORTÃO QUANDO O BOTÃO "BURRO" ESTIVER PRESSIONADO
#define BUTTON_PIN_2 4
#ifndef BUTTON_PIN_2
#error BUTTON_PIN_2 is not specified
#endif

//TOGGLE QUANDO BOTAO "BURRO" PRESSIONADO E BOTAO DE CONTROLE EXTERNO TIVER SIDO ATIVADO
#define TOGGLE_PIN 12
#ifndef TOGGLE_PIN
#error TOGGLE_PIN is not specified
#endif

//SENSOR DE CONTATO DA PORTA
#define SENSOR_PIN 16
#ifndef SENSOR_PIN
#error SENSOR_PIN is not specified
#endif

//PERÍODO EM QUE FICARÁ DISPONIVEL ABRIR COM O TOGGLE UMA VEZ JÁ ATIVADO COM BOTÃO
const int password_period = 5;

//ETSTIMER tempo em que ficará disponivel para toggle e tempo que esperará novo clique do toggle
ETSTimer password_timer, toggle_press_timer;

//Declarando as variáveis, pois serão usadas a seguir
void lock_lock();
void lock_unlock();
void password_unlock();

// Define qual status do PIN que ativa o relê/led
void relay_write(int value) {
        gpio_write(relay_gpio, value ? 1 : 0);
}

void password_write(int value) {
        gpio_write(password_gpio, value ? 0 : 1);
}

void led_write(bool on) {
        gpio_write(led_gpio, on ? 0 : 1);
}

//Função para resetar configurações
void reset_configuration_task() {
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

//Seção que ativa os pinos para serem usados como INPUT ou OUTPUT
void gpio_init() {
        gpio_enable(led_gpio, GPIO_OUTPUT);
        led_write(false);

        gpio_enable(relay_gpio, GPIO_OUTPUT);
        relay_write(false);

        gpio_enable(password_gpio, GPIO_OUTPUT);
        password_write(false);

        gpio_enable(BUTTON_PIN, GPIO_INPUT);
        gpio_enable(BUTTON_PIN_2, GPIO_INPUT);
        gpio_enable(SENSOR_PIN, GPIO_INPUT);
        gpio_enable(TOGGLE_PIN, GPIO_INPUT);
}

//Função que identifica o acessório (caso tenha outros iguais ao que se deseja adicionar ao HOMEKIT)
void lock_identify_task(void *_args) {
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

void door_identify(homekit_value_t _value) {
        printf("door identify\n");
}

//Variáveis para serem usadas como status da porta
typedef enum {
        lock_state_unsecured = 0,
        lock_state_secured = 1,
        lock_state_jammed = 2,
        lock_state_unknown = 3,
} lock_state_t;

//Características necessárias ao HOMEKIT
homekit_characteristic_t name = HOMEKIT_CHARACTERISTIC_(NAME, "Lock");
homekit_characteristic_t ota_trigger  = API_OTA_TRIGGER;
homekit_characteristic_t manufacturer = HOMEKIT_CHARACTERISTIC_(MANUFACTURER,  "X");
homekit_characteristic_t serial       = HOMEKIT_CHARACTERISTIC_(SERIAL_NUMBER, "1");
homekit_characteristic_t model        = HOMEKIT_CHARACTERISTIC_(MODEL,         "Z");
homekit_characteristic_t revision     = HOMEKIT_CHARACTERISTIC_(FIRMWARE_REVISION,  "0.0.0");


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
        homekit_characteristic_notify(&lock_target_state, value);
}

void lock_control_point(homekit_value_t value) {
        // Nothing to do here
}

//Função que tranca a porta e notifica o Homekit que foi feito mudançã no estado da fechadura
void lock_lock() {
        relay_write(false);
        led_write(false);
        lock_current_state.value = HOMEKIT_UINT8(lock_state_secured);
        homekit_characteristic_notify(&lock_current_state, lock_current_state.value);

}

//FUNÇÃO PARA DEIXAR O TOGGLE ATIVO ESPERANDO 10 CLIQUES - ATIVA O TOGGLE COM GND;
void password_callback() {
        printf("Tempo iniciou.\n");
        sdk_os_timer_disarm(&password_timer);
        printf("Tempo findou.\n");
        password_write(false);
}

//Função que para ser iniciada assim que o servidor do Homekit inicia
void lock_init() {
        lock_current_state.value = HOMEKIT_UINT8(lock_state_secured);
        homekit_characteristic_notify(&lock_current_state, lock_current_state.value);
}

//Função que trava a fechadura e notifica o Homekit da mudança do estado da fechadura
void lock_unlock() {
        relay_write(true);
        led_write(true);
        lock_current_state.value = HOMEKIT_UINT8(lock_state_unsecured);
        homekit_characteristic_notify(&lock_current_state, lock_current_state.value);
}


//Função que define o comportamento do botão com 1, 2, 3 cliques ou clique longo
void button_callback(button_event_t event, void* context) {
        switch (event) {
        case button_event_single_press:
                printf("Unlocking\n");
                relay_write(true);
                led_write(true);
                if (lock_target_state.value.int_value != lock_state_unsecured) {
                        lock_target_state.value = HOMEKIT_UINT8(lock_state_unsecured);
                        homekit_characteristic_notify(&lock_target_state, lock_target_state.value);
                        lock_current_state.value = HOMEKIT_UINT8(lock_state_unsecured);
                        homekit_characteristic_notify(&lock_current_state, lock_current_state.value);
                }
                break;
        case button_event_double_press:
                printf("Locking\n");
                relay_write(false);
                led_write(false);
                if (lock_target_state.value.int_value != lock_state_secured) {
                        lock_target_state.value = HOMEKIT_UINT8(lock_state_secured);
                        homekit_characteristic_notify(&lock_target_state, lock_target_state.value);
                        lock_current_state.value = HOMEKIT_UINT8(lock_state_secured);
                        homekit_characteristic_notify(&lock_current_state, lock_current_state.value);
                }
                break;
        case button_event_long_press:
                reset_configuration();
                break;
        default:
                printf("Unknown button event: %d\n", event);
        }
}

//Função que define o comportamento do botão com 1, 2, 3 cliques ou clique longo
void button_2_callback(button_event_t event, void* context) {
        switch (event) {
        case button_event_single_press:
                printf("Button_pin_2 single press\n");
                led_write(false);
                printf("é pra o led fazer algo\n");
                break;
        case button_event_double_press:
                printf("Button_pin_2 double press\n");
                led_write(true);
                sdk_os_timer_arm(&password_timer, password_period * 1000, 0);
                password_callback();
                break;
        case button_event_long_press:
                printf("Button_pin_2 long press\n");

                break;
        default:
                printf("Unknown button event: %d\n", event);
        }
}
//Função que conta os "apertos" de interruptores
void toggle_callback(bool high, void *context) {
        toggle_press_count++;
        printf("mais1\n");
        sdk_os_timer_arm(&toggle_press_timer, 3100, 0);
}

//Função que quando chega a 10 "apertos" abre a fechadura após outros passos estarem completos
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
/*
   fim da parte da Fechadura
 */

//início do Sensor de Contato
homekit_characteristic_t door_open_detected = HOMEKIT_CHARACTERISTIC_(CONTACT_SENSOR_STATE, 0);

void sensor_callback(bool high, void *context) {
        door_open_detected.value = HOMEKIT_UINT8(high ? 1 : 0);
        homekit_characteristic_notify(&door_open_detected, door_open_detected.value);
}
/*
   fim da parte do sensor de contato -  outra parte no início do código
 */

//declaração global dos acessórios
homekit_accessory_t *accessories[] = {
        HOMEKIT_ACCESSORY(.id=1, .category=homekit_accessory_category_door_lock, .services=(homekit_service_t*[]){
                HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]){
                        &name,
                        &manufacturer,
                        &serial,
                        &model,
                        &revision,
                        HOMEKIT_CHARACTERISTIC(IDENTIFY, lock_identify),
                        NULL
                }),
                HOMEKIT_SERVICE(LOCK_MECHANISM, .primary=true, .characteristics=(homekit_characteristic_t*[]){
                        HOMEKIT_CHARACTERISTIC(NAME, "Lock"),
                        &lock_current_state,
                        &lock_target_state,
                        &ota_trigger,
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
        HOMEKIT_ACCESSORY(.id=2, .category=homekit_accessory_category_sensor, .services=(homekit_service_t*[]) {
                HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]) {
                        &name,
                        &manufacturer,
                        &serial,
                        &model,
                        &revision,
                        HOMEKIT_CHARACTERISTIC(IDENTIFY, door_identify),
                        NULL
                }),
                HOMEKIT_SERVICE(CONTACT_SENSOR, .primary=false, .characteristics=(homekit_characteristic_t*[]) {
                        HOMEKIT_CHARACTERISTIC(NAME, "Kontakt"),
                        &door_open_detected,
                        NULL
                }),
                NULL
        }),
        NULL
};

homekit_server_config_t config = {
        .accessories = accessories,
        .password = "736-24-212",
        .setupId="7EN2", //ci=8
};

void on_wifi_ready() {
}

void create_accessory_name() {
        uint8_t macaddr[6];
        sdk_wifi_get_macaddr(STATION_IF, macaddr);

        int name_len = snprintf(NULL, 0, "Fechadura-%02X%02X%02X",
                                macaddr[3], macaddr[4], macaddr[5]);
        char *name_value = malloc(name_len+1);
        snprintf(name_value, name_len+1, "Fechadura-%02X%02X%02X",
                 macaddr[3], macaddr[4], macaddr[5]);

        name.value = HOMEKIT_STRING(name_value);

        char *serial_value = malloc(13);
        snprintf(serial_value, 13, "%02X%02X%02X%02X%02X%02X", macaddr[0], macaddr[1], macaddr[2], macaddr[3], macaddr[4], macaddr[5]);
        serial.value = HOMEKIT_STRING(serial_value);
}

void user_init(void) {
        uart_set_baud(0, 115200);
        create_accessory_name();
        gpio_init();
        lock_init();
        sdk_os_timer_setfn(&toggle_press_timer,toggle_action, NULL);
        sdk_os_timer_setfn(&password_timer, password_callback, NULL);


        int c_hash=ota_read_sysparam(&manufacturer.value.string_value,&serial.value.string_value,
                                     &model.value.string_value,&revision.value.string_value);
        c_hash=1; revision.value.string_value="0.0.1"; //cheat line
        //config.accessories[0]->config_number=c_hash;

        button_config_t button_config = BUTTON_CONFIG(
                button_active_low,
                .max_repeat_presses=2,
                .long_press_time=3000,
                );

        if (toggle_create(SENSOR_PIN, sensor_callback, NULL)) {
                printf("Failed to initialize sensor\n");
        }
        if (toggle_create(TOGGLE_PIN, toggle_callback, NULL)) {
                printf("Failed to initialize sensor\n");
        }
        if (button_create(BUTTON_PIN, button_config, button_callback, NULL)) {
                printf("Failed to initialize button\n");
        }
        if (button_create(BUTTON_PIN_2, button_config, button_2_callback, NULL)) {
                printf("Failed to initialize button\n");
        }

        homekit_server_init(&config);
}
