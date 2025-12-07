/*
 * @file main/main.c
 * @brief Main application entry point for MPU-9250 rocket control system example
 * @author Mathias Kulola
 * @data 2024/06/20
 */

#include <stdio.h>
#include <inttypes.h>
#include "mpu9250_example.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"



void app_main(void) {
    example_main();
}