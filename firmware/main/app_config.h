#ifndef CONFIG_H
#define CONFIG_H

#include "driver/gpio.h"

/* --------------- Command executor -------------- */

#define APP_EXECUTOR_TASK_PRIORITY   (10)
#define APP_EXECUTOR_TASK_STACK_SIZE (4096)

#define APP_TX_BUFFER_TIMEOUT_MS     (1000)

/* All command arguments are assumed to be in big-endian format. */

#define APP_CMD_ECHO_CODE           (0x01u)         /* [0 - 127] Anything  */
#define APP_CMD_ACK_CODE(code)      (0xFF - (code)) /* [0 - 1] Status code */

#define APP_CMD_STEP_ARM_CODE       (0x02u) /* [0] Stepper ID                           */
#define APP_CMD_STEP_DISARM_CODE    (0x03u) /* [0] Stepper ID                           */
#define APP_CMD_STEP_START_CODE     (0x04u) /* [0] Stepper ID, [1] Stepping direction   */
#define APP_CMD_STEP_STOP_CODE      (0x05u) /* [0] Stepper ID                           */
#define APP_CMD_STEP_SET_FREQ_CODE  (0x06u) /* [0] Stepper ID, [1 - 2] Frequency in Hz  */
#define APP_CMD_STEP_GET_POS_CODE   (0x07u) /* [0] Stepper ID                           */

#define APP_CMD_SENSE_GET_POS_CODE  (0x08u) /* [0] Sensor ID                            */
#define APP_CMD_SENSE_SET_ZERO_CODE (0x09u) /* [0] Sensor ID, [1 - 2] Zero angle        */
#define APP_CMD_SENSE_GET_PWR_CODE  (0x0Au) /* [0] Sensor ID                            */
#define APP_CMD_SENSE_GET_CURR_CODE (0x0Bu) /* [0] Sensor ID                            */
#define APP_CMD_SENSE_GET_VOLT_CODE (0x0Cu) /* [0] Sensor ID                            */

/* ------------------- Stepper ------------------ */

#define APP_STEPPER_COUNT                   (2) 
#define APP_STEPPER_DEFAULT_FREQUENCY_HZ    (1000) 

#define APP_STEPPER_A_EN_PIN    (GPIO_NUM_4)
#define APP_STEPPER_A_DIR_PIN   (GPIO_NUM_5)
#define APP_STEPPER_A_STEP_PIN  (GPIO_NUM_6)
#define APP_STEPPER_B_EN_PIN    (GPIO_NUM_7)
#define APP_STEPPER_B_DIR_PIN   (GPIO_NUM_15)
#define APP_STEPPER_B_STEP_PIN  (GPIO_NUM_16)

/* ------------------- I2C bus ------------------ */

#define APP_I2C_BUS_A_INTERRUPT_PRIORITY    (1)
#define APP_I2C_BUS_A_SDA_PIN               (GPIO_NUM_18)
#define APP_I2C_BUS_A_SCL_PIN               (GPIO_NUM_17)

#define APP_I2C_BUS_B_INTERRUPT_PRIORITY    (1)
#define APP_I2C_BUS_B_SDA_PIN               (GPIO_NUM_13)
#define APP_I2C_BUS_B_SCL_PIN               (GPIO_NUM_12)

/* Joined search macros */
#define APP_I2C_SCAN_TIMEOUT_MS                 (100)
#define APP_I2C_SCAN_MAX_FOUND_DEVICES          (10)
#define APP_I2C_SCAN_FOUND_DEVICES_ARRAY_SIZE   (APP_I2C_SCAN_MAX_FOUND_DEVICES)

/* ------------------- AS5600 ------------------- */

#define APP_AS5600_COUNT (2)

/* ------------------- INA226 ------------------- */

#define APP_INA226_COUNT            (2)
#define APP_INA226_A_I2C_ADDRESS    (0x44) 
#define APP_INA226_B_I2C_ADDRESS    (0x44)
#define APP_INA226_A_ALERT_GPIO_PIN (GPIO_NUM_8)
#define APP_INA226_B_ALERT_GPIO_PIN (GPIO_NUM_14)

#endif /* CONFIG_H */