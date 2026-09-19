#ifndef CONFIG_H
#define CONFIG_H

#include "driver/gpio.h"

/* --------------- Command executor -------------- */

#define APP_EXECUTOR_TASK_PRIORITY   (10)
#define APP_EXECUTOR_TASK_STACK_SIZE (4096)

/* All command arguments are assumed to be in big-endian format. */

#define APP_CMD_ECHO_CODE           (0x01u)         /* [0 - 127] Anything  */
#define APP_CMD_ACK_CODE(code)      (0xFF - (code)) /* [0 - 1] Status code */

#define APP_CMD_STEP_ARM_CODE       (0x02u) /* [0] Stepper ID                           */
#define APP_CMD_STEP_DISARM_CODE    (0x03u) /* [0] Stepper ID                           */
#define APP_CMD_STEP_START_CODE     (0x04u) /* [0] Stepper ID, [1] Stepping direction   */
#define APP_CMD_STEP_STOP_CODE      (0x05u) /* [0] Stepper ID                           */
#define APP_CMD_STEP_SET_FREQ_CODE  (0x06u) /* [0] Stepper ID, [1 - 2] Frequency in Hz  */
#define APP_CMD_STEP_GET_POS_CODE   (0x07u) /* [0] Stepper ID                           */

/* ------------------- Stepper ------------------ */

#define APP_STEPPER_DEFAULT_FREQUENCY_HZ  (1000) 

#define APP_STEPPER_A_ID    (0)
#define APP_STEPPER_B_ID    (1)
#define APP_STEPPER_COUNT   (2)

#define APP_STEPPER_EN_PIN_A    (GPIO_NUM_4)
#define APP_STEPPER_DIR_PIN_A   (GPIO_NUM_5)
#define APP_STEPPER_STEP_PIN_A  (GPIO_NUM_6)
#define APP_STEPPER_EN_PIN_B    (GPIO_NUM_7)
#define APP_STEPPER_DIR_PIN_B   (GPIO_NUM_15)
#define APP_STEPPER_STEP_PIN_B  (GPIO_NUM_16)

#endif /* CONFIG_H */