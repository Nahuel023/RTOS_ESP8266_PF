#include "esp_common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "stdio.h"
#include "gpio.h"
#include "hw_timer.h"
#include "pwm.h"

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

    // Configura el GPIO8 como E/S general -- SW
    //PIN_FUNC_SELECT(PERIPHS_IO_MUX_SD_DATA1_U, FUNC_GPIO8);

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
    // Leer distancia de la variable global (actualizado por task_HC_SR04)
    uint32_t measuredDistance = distance;

    //Comprueba si hay un obstáculo delante
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

    // Avanzar (suponiendo que tenga una función de movimiento hacia adelante)
    MotorControl(30, 30);
    pwm_start();
    vTaskDelay(10 / portTICK_RATE_MS); //Ajustar el retraso para la velocidad
  }
  vTaskDelete(NULL);
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
    xTaskCreate(&task_blink, "startup", 100, NULL, 1, NULL);
    xTaskCreate(&task_HC_SR04, "HC_SR04", 1024, NULL, 1, NULL);
    xTaskCreate(&task_AvoidObstacle, "avoid_obstacle", 2048, NULL, 1, NULL);
    
}

