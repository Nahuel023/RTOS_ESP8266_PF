#include "esp_common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "stdio.h"
#include "gpio.h"
#include "pwm.h"
#include <string.h>
#include <lwip/sockets.h>
#include <lwip/netdb.h>

#define SSID "ANM2"
#define PASSWORD "anm157523"

#define GPIO_TRIG 2 
#define GPIO_ECHO 15 

#define PWM_CHANNELS   2
#define PWM_PERIOD     10000 // 10ms, lo que equivale a una frecuencia de 100 Hz

#define GPIO_MOTOR_RIGHT_FWD  0
#define GPIO_MOTOR_RIGHT_BCK  4
#define GPIO_MOTOR_LEFT_FWD   12
#define GPIO_MOTOR_LEFT_BCK   13
#define GPIO_ENB_RIGHT 14
#define GPIO_ENA_LEFT 5

#define MIN_OBSTACLE_DISTANCE 30 // cm

uint32 duty[PWM_CHANNELS] = {0, 0};
uint32_t distance;
bool obstacle_avoidance = false;  // Variable para controlar la tarea de evitar obstáculos

/******************************************************************************
 * FunctionName : user_rf_cal_sector_set
 * Description  : SDK just reversed 4 sectors, used for rf init data and paramters.
 *                We add this function to force users to set rf cal sector, since
 *                we don't know which sector is free in user's application.
 *                sector map for last several sectors : ABCCC
 *                A : rf cal
 *                B : rf init data
 *                C : sdk parameters
 * Parameters   : none
 * Returns      : rf cal sector
*******************************************************************************/
uint32 user_rf_cal_sector_set(void)
{
    flash_size_map size_map = system_get_flash_size_map();
    uint32 rf_cal_sec = 0;
    switch (size_map) {
        case FLASH_SIZE_4M_MAP_256_256:
            rf_cal_sec = 128 - 5;
            break;

        case FLASH_SIZE_8M_MAP_512_512:
            rf_cal_sec = 256 - 5;
            break;

        case FLASH_SIZE_16M_MAP_512_512:
        case FLASH_SIZE_16M_MAP_1024_1024:
            rf_cal_sec = 512 - 5;
            break;

        case FLASH_SIZE_32M_MAP_512_512:
        case FLASH_SIZE_32M_MAP_1024_1024:
            rf_cal_sec = 1024 - 5;
            break;

        default:
            rf_cal_sec = 0;
            break;
    }

    return rf_cal_sec;
}

void gpio_init(void)
{
    // Configura el GPIO16 como salida
    gpio16_output_conf();

    //HC-SR04-------------------------------------------
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_GPIO2);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, FUNC_GPIO15);

    //MOTOR DERECHO-------------------------------------
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, FUNC_GPIO12);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_GPIO13);

    //MOTOR IZQUIERDO-----------------------------------
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO4_U, FUNC_GPIO4);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0);

    uint32 io_info[PWM_CHANNELS][3] = {
        {PERIPHS_IO_MUX_MTMS_U, FUNC_GPIO14, GPIO_ENB_RIGHT}, // Este pin debe ser habilitado para PWM
        {PERIPHS_IO_MUX_GPIO5_U, FUNC_GPIO5, GPIO_ENA_LEFT}, // Este pin debe ser habilitado para PWM
    };

    // Configurar el PWM
    pwm_init(PWM_PERIOD, duty, PWM_CHANNELS, io_info);
}

void MotorControl(int8_t setMotorRight, int8_t setMotorLeft){
    uint32 auxSetMotor;

    auxSetMotor = (abs(setMotorRight)*1023)/100;
    if(setMotorRight >= 0){
        //SET MOTOR DERECHO ADELANTE
        pwm_set_duty(auxSetMotor,0);
        GPIO_OUTPUT_SET(GPIO_MOTOR_RIGHT_FWD, 1);
        GPIO_OUTPUT_SET(GPIO_MOTOR_RIGHT_BCK, 0);
    }else{
        //SET MOTOR DERECHO REVERSA
        pwm_set_duty(auxSetMotor,0);
        GPIO_OUTPUT_SET(GPIO_MOTOR_RIGHT_FWD, 0);
        GPIO_OUTPUT_SET(GPIO_MOTOR_RIGHT_BCK, 1);
    }

    auxSetMotor = (abs(setMotorLeft)*1023)/100;
    if(setMotorLeft >= 0){
        //SET MOTOR IZQUIERDO ADELANTE
        pwm_set_duty(auxSetMotor,1);
        GPIO_OUTPUT_SET(GPIO_MOTOR_LEFT_FWD, 1);
        GPIO_OUTPUT_SET(GPIO_MOTOR_LEFT_BCK, 0);
    }else{
        //SET MOTOR IZQUIERDO REVERSA
        pwm_set_duty(auxSetMotor,1);
        GPIO_OUTPUT_SET(GPIO_MOTOR_LEFT_FWD, 0);
        GPIO_OUTPUT_SET(GPIO_MOTOR_LEFT_BCK, 1);
    }
}

void task_HC_SR04(void* ignore)
{
    while(true) {
        // Inicia el pulso
        GPIO_OUTPUT_SET(GPIO_TRIG, 0); 
        vTaskDelay(2 / portTICK_RATE_MS);
        GPIO_OUTPUT_SET(GPIO_TRIG, 1); 
        vTaskDelay(10 / portTICK_RATE_MS);
        GPIO_OUTPUT_SET(GPIO_TRIG, 0);

        // Mide el tiempo que tarda el pulso en regresar
        while (GPIO_INPUT_GET(GPIO_ECHO) == 0);
        uint32_t start = system_get_time();
        while (GPIO_INPUT_GET(GPIO_ECHO) == 1);
        uint32_t end = system_get_time();

        // Calcula la distancia
        distance = (end - start) / 58.2; // Velocidad del sonido en cm/us

        // Imprime la distancia
        printf("Distancia: %d cm\n", distance);

        vTaskDelay(300 / portTICK_RATE_MS);
    }

    vTaskDelete(NULL);
}

void task_blink(void* ignore)
{
    while(true) {
        gpio16_output_set(0);
        vTaskDelay(500/portTICK_RATE_MS);
        gpio16_output_set(1);
        vTaskDelay(500/portTICK_RATE_MS);
    }

    vTaskDelete(NULL);
}

void task_AvoidObstacle(void* ignore) {
    while (true) {
        if (!obstacle_avoidance) {
            vTaskDelay(100 / portTICK_RATE_MS);
            continue;
        }

        // Leer distancia de la variable global (actualizado por task_HC_SR04)
        uint32_t measuredDistance = distance;

        // Comprueba si hay un obstáculo delante
        if (measuredDistance < MIN_OBSTACLE_DISTANCE) {
            // Gira a la derecha hasta que no haya obstáculos.
            while (measuredDistance < MIN_OBSTACLE_DISTANCE) {
                MotorControl(30, -30); // Ajuste la velocidad según sea necesario (positiva para girar a la derecha)
                pwm_start();
                vTaskDelay(100 / portTICK_RATE_MS); // Breve retraso entre controles de distancia
                measuredDistance = distance; // Leer distancia actualizada
            }
            // Breve pausa después de girar
            vTaskDelay(50 / portTICK_RATE_MS);
        }

        // Avanzar
        MotorControl(30, 30);
        pwm_start();
        vTaskDelay(10 / portTICK_RATE_MS); // Ajustar el retraso para la velocidad
    }
    vTaskDelete(NULL);
}

/*
void handle_client(int client_socket) {
    char buffer[1024];
    int len = read(client_socket, buffer, sizeof(buffer) - 1);
    buffer[len] = '\0';

    if (strstr(buffer, "GET / ") != NULL) {
        char response[2048];
        snprintf(response, sizeof(response),
                 "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n"
                 "<!DOCTYPE html><html><head><style>"
                 "body { font-family: Arial; text-align: center; background-color: #f0f0f0; }"
                 "h1 { color: #333; }"
                 ".control-buttons { display: flex; flex-direction: column; align-items: center; gap: 10px; }"
                 ".direction-buttons { display: flex; flex-direction: row; gap: 10px; }"
                 "button { background-color: #4CAF50; border: none; color: white; padding: 15px; text-align: center; text-decoration: none; display: inline-block; font-size: 24px; margin: 4px 2px; cursor: pointer; border-radius: 50%; }"
                 "</style></head><body>"
                 "<h1>Control del Robot</h1>"
                 "<div class=\"control-buttons\">"
                 "<button onclick=\"fetch('/start')\">Encender</button>"
                 "<button onclick=\"fetch('/stop')\">Apagar</button>"
                 "<button onclick=\"fetch('/avoid')\">Esquivar Objetos</button>"
                 "<div class=\"direction-buttons\">"
                 "<button onmousedown=\"fetch('/forward')\" onmouseup=\"fetch('/stop')\">&#9650;</button>"
                 "</div>"
                 "<div class=\"direction-buttons\">"
                 "<button onmousedown=\"fetch('/left')\" onmouseup=\"fetch('/stop')\">&#9664;</button>"
                 "<button onmousedown=\"fetch('/backward')\" onmouseup=\"fetch('/stop')\">&#9660;</button>"
                 "<button onmousedown=\"fetch('/right')\" onmouseup=\"fetch('/stop')\">&#9654;</button>"
                 "</div>"
                 "<p>Distancia medida: <span id=\"distance\">%d cm</span></p>"
                 "<script>"
                 "setInterval(() => { fetch('/distance').then(response => response.text()).then(data => { document.getElementById('distance').innerText = data; }); }, 1000);"
                 "</script></body></html>", distance);
        write(client_socket, response, strlen(response));
    } else if (strstr(buffer, "GET /forward") != NULL) {
        MotorControl(30, 30);
        pwm_start();
        const char *response = "HTTP/1.1 200 OK\r\n\r\nOK";
        write(client_socket, response, strlen(response));
    } else if (strstr(buffer, "GET /backward") != NULL) {
        MotorControl(-30, -30);
        pwm_start();
        const char *response = "HTTP/1.1 200 OK\r\n\r\nOK";
        write(client_socket, response, strlen(response));
    } else if (strstr(buffer, "GET /left") != NULL) {
        MotorControl(-30, 30);
        pwm_start();
        const char *response = "HTTP/1.1 200 OK\r\n\r\nOK";
        write(client_socket, response, strlen(response));
    } else if (strstr(buffer, "GET /right") != NULL) {
        MotorControl(30, -30);
        pwm_start();
        const char *response = "HTTP/1.1 200 OK\r\n\r\nOK";
        write(client_socket, response, strlen(response));
    } else if (strstr(buffer, "GET /start") != NULL) {
        MotorControl(30, 30);
        pwm_start();
        const char *response = "HTTP/1.1 200 OK\r\n\r\nRobot encendido";
        write(client_socket, response, strlen(response));
    } else if (strstr(buffer, "GET /stop") != NULL) {
        MotorControl(0, 0);
        pwm_start();
        const char *response = "HTTP/1.1 200 OK\r\n\r\nRobot apagado";
        write(client_socket, response, strlen(response));
    } else if (strstr(buffer, "GET /avoid") != NULL) {
        obstacle_avoidance = !obstacle_avoidance;  // Toggle obstacle avoidance
        const char *response = obstacle_avoidance ? "HTTP/1.1 200 OK\r\n\r\nEsquivar objetos activado"
                                                  : "HTTP/1.1 200 OK\r\n\r\nEsquivar objetos desactivado";
        write(client_socket, response, strlen(response));
    } else if (strstr(buffer, "GET /distance") != NULL) {
        char response[64];
        snprintf(response, sizeof(response), "HTTP/1.1 200 OK\r\n\r\n%d cm", distance);
        write(client_socket, response, strlen(response));
    }

    close(client_socket);
}*/

void handle_client(int client_socket) {
    char buffer[1024];
    int len = read(client_socket, buffer, sizeof(buffer) - 1);
    buffer[len] = '\0';

    if (strstr(buffer, "GET / ") != NULL) {
        char response[2048];
        snprintf(response, sizeof(response),
                 "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n"
                 "<!DOCTYPE html><html><head><style>"
                 "body { font-family: Arial; text-align: center; background-color: #f0f0f0; }"
                 "h1 { color: #333; }"
                 ".control-buttons { display: flex; flex-direction: column; align-items: center; gap: 10px; }"
                 ".direction-buttons { display: flex; flex-direction: row; gap: 10px; }"
                 "button { background-color: #4CAF50; border: 2px solid #333; color: white; padding: 15px; text-align: center; text-decoration: none; display: inline-block; font-size: 24px; margin: 4px 2px; cursor: pointer; border-radius: 10px; }"
                 "#avoidButton { background-color: %s; }"
                 ".distance-display { display: inline-block; padding: 10px; border: 2px solid #333; border-radius: 10px; background-color: #fff; }"
                 "</style></head><body>"
                 "<h1>Control del Robot</h1>"
                 "<div class=\"control-buttons\">"
                 "<button id=\"avoidButton\" onclick=\"toggleAvoidance()\">Esquivar Objetos</button>"
                 "<div class=\"direction-buttons\">"
                 "<button onmousedown=\"fetch('/forward')\" onmouseup=\"fetch('/stop')\">&#9650;</button>"
                 "</div>"
                 "<div class=\"direction-buttons\">"
                 "<button onmousedown=\"fetch('/left')\" onmouseup=\"fetch('/stop')\">&#9664;</button>"
                 "<button onmousedown=\"fetch('/backward')\" onmouseup=\"fetch('/stop')\">&#9660;</button>"
                 "<button onmousedown=\"fetch('/right')\" onmouseup=\"fetch('/stop')\">&#9654;</button>"
                 "</div>"
                 "<p class=\"distance-display\">Distancia medida: <span id=\"distance\">%d cm</span></p>"
                 "<script>"
                 "let obstacleAvoidance = %s;"
                 "function toggleAvoidance() {"
                 "    obstacleAvoidance = !obstacleAvoidance;"
                 "    fetch('/avoid');"
                 "    document.getElementById('avoidButton').style.backgroundColor = obstacleAvoidance ? 'red' : 'green';"
                 "}"
                 "setInterval(() => { fetch('/distance').then(response => response.text()).then(data => { document.getElementById('distance').innerText = data; }); }, 1000);"
                 "</script></body></html>",
                 obstacle_avoidance ? "red" : "green", distance, obstacle_avoidance ? "true" : "false");
        write(client_socket, response, strlen(response));
    } else if (strstr(buffer, "GET /forward") != NULL) {
        MotorControl(30, 30);
        pwm_start();
        const char *response = "HTTP/1.1 200 OK\r\n\r\nOK";
        write(client_socket, response, strlen(response));
    } else if (strstr(buffer, "GET /backward") != NULL) {
        MotorControl(-30, -30);
        pwm_start();
        const char *response = "HTTP/1.1 200 OK\r\n\r\nOK";
        write(client_socket, response, strlen(response));
    } else if (strstr(buffer, "GET /left") != NULL) {
        MotorControl(-30, 30);
        pwm_start();
        const char *response = "HTTP/1.1 200 OK\r\n\r\nOK";
        write(client_socket, response, strlen(response));
    } else if (strstr(buffer, "GET /right") != NULL) {
        MotorControl(30, -30);
        pwm_start();
        const char *response = "HTTP/1.1 200 OK\r\n\r\nOK";
        write(client_socket, response, strlen(response));
    } else if (strstr(buffer, "GET /stop") != NULL) {
        MotorControl(0, 0);
        pwm_start();
        const char *response = "HTTP/1.1 200 OK\r\n\r\nOK";
        write(client_socket, response, strlen(response));
    } else if (strstr(buffer, "GET /avoid") != NULL) {
        obstacle_avoidance = !obstacle_avoidance;  // Toggle obstacle avoidance
        const char *response = obstacle_avoidance ? "HTTP/1.1 200 OK\r\n\r\nEsquivar objetos activado"
                                                  : "HTTP/1.1 200 OK\r\n\r\nEsquivar objetos desactivado";
        write(client_socket, response, strlen(response));
    } else if (strstr(buffer, "GET /distance") != NULL) {
        char response[64];
        snprintf(response, sizeof(response), "HTTP/1.1 200 OK\r\n\r\n%d cm", distance);
        write(client_socket, response, strlen(response));
    }

    close(client_socket);
}



void task_http_server(void* ignore) {
    int server_socket = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(80);
    server_addr.sin_addr.s_addr = INADDR_ANY;

    bind(server_socket, (struct sockaddr*)&server_addr, sizeof(server_addr));
    listen(server_socket, 5);

    while (true) {
        struct sockaddr_in client_addr;
        socklen_t client_addr_len = sizeof(client_addr);
        int client_socket = accept(server_socket, (struct sockaddr*)&client_addr, &client_addr_len);

        if (client_socket >= 0) {
            handle_client(client_socket);
        }
    }
}

void wifi_event_handler(System_Event_t *event) {
    if (event == NULL) {
        return;
    }

    switch (event->event_id) {
        case EVENT_STAMODE_GOT_IP:
            printf("Connected to WiFi, IP: " IPSTR "\n", IP2STR(&event->event_info.got_ip.ip));
            break;
        case EVENT_STAMODE_DISCONNECTED:
            printf("Disconnected from WiFi\n");
            break;
        default:
            break;
    }
}

void wifi_init() {
    struct station_config stationConf;

    wifi_set_opmode(STATION_MODE);
    strncpy(stationConf.ssid, SSID, sizeof(stationConf.ssid));
    strncpy(stationConf.password, PASSWORD, sizeof(stationConf.password));

    wifi_station_set_config(&stationConf);
    wifi_set_event_handler_cb(wifi_event_handler);
    wifi_station_connect();
}

/******************************************************************************
 * FunctionName : user_init
 * Description  : entry of user application, init user function here
 * Parameters   : none
 * Returns      : none
*******************************************************************************/
void user_init(void)
{
    // Llama a la función de inicialización de GPIO antes de crear las tareas
    gpio_init();
    // Crea las tareas
    uart_div_modify(0, UART_CLK_FREQ / 115200);

    wifi_init();

    xTaskCreate(task_blink, "blink", 256, NULL, 2, NULL);
    xTaskCreate(task_HC_SR04, "HC-SR04", 1024, NULL, 2, NULL);
    xTaskCreate(task_AvoidObstacle, "AvoidObstacle", 1024, NULL, 2, NULL);
    xTaskCreate(task_http_server, "HTTP Server", 2048, NULL, 2, NULL);
}
