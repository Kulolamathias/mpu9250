/**
 * @file main/mpu9250_example.h
 * @brief Example usage of MPU-9250 driver for rocket control system
 * @author Mathias Kulola
 * @date 2024/06/20
 */

#ifndef MPU9250_EXAMPLE_H
#define MPU9250_EXAMPLE_H

#include "mpu9250.h"

void example_main(void);
void mpu9250_example_task(void* pvParameters);

#endif /* MPU9250_EXAMPLE_H */
