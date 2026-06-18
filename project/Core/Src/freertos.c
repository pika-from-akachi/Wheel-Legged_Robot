/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    freertos.c
  * @brief   FreeRTOS task implementations for wheel-legged robot
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <math.h>
#include "icm42688.h"
#include "nrf24l01_rx.h"
#include "freertos_tasks.h"
#include "motor_driver.h"
#include "el05_motor.h"
#include "lqr_control.h"
#include "robot_model.h"
#include "esp32_com.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* ===== Task handles ===== */
osThreadId_t taskHandle_IMU = NULL;
osThreadId_t taskHandle_Balance = NULL;
osThreadId_t taskHandle_Motor = NULL;
osThreadId_t taskHandle_Remote = NULL;
osThreadId_t taskHandle_Monitor = NULL;
osThreadId_t taskHandle_Debug = NULL;

/* ===== Queue handles ===== */
osMessageQueueId_t queue_IMUData = NULL;
osMessageQueueId_t queue_RemoteData = NULL;
osMessageQueueId_t queue_MotorCmd = NULL;

/* ===== Mutex handles ===== */
osMutexId_t mutex_CAN = NULL;
osMutexId_t mutex_SPI1 = NULL;
osMutexId_t mutex_SPI3 = NULL;

/* ===== Semaphore handles ===== */
osSemaphoreId_t sem_IMU_Ready = NULL;
osSemaphoreId_t sem_Remote_Ready = NULL;

/* ===== Status variables ===== */
volatile uint8_t g_system_status = 0;
volatile uint32_t g_imu_update_count = 0;
volatile uint32_t g_remote_update_count = 0;

/* ===== Double buffer for ISR→Task IMU data transfer ===== */
static ICM42688_RawData_t g_imu_raw_buf[2];
static volatile uint32_t g_imu_raw_active_idx = 0;

/* ===== Unique to this file ===== */
osThreadId_t taskHandle_EL05_Motor;       /* EL05 4电机顺序控制 + 轮毂电机1/2 */
osThreadId_t taskHandle_M0601C_Motor;     /* M0601C wheel motor task */
osThreadId_t taskHandle_CAN;              /* CAN communication task */

osMessageQueueId_t queue_EL05_MotorCmd;   /* EL05 motor command queue */
osMessageQueueId_t queue_M0601C_MotorCmd; /* M0601C motor command queue */
osMessageQueueId_t queue_CAN_TX;          /* CAN TX message queue */
osMessageQueueId_t queue_CAN_RX;          /* CAN RX message queue */

osMutexId_t mutex_UART1;                  /* Protect UART1 (M0601C) */

osSemaphoreId_t sem_CAN_RX;               /* CAN RX complete */

volatile uint32_t g_el05_motor_update_count = 0;
volatile uint32_t g_m0601c_motor_update_count = 0;
volatile uint32_t g_can_tx_count = 0;
volatile uint32_t g_can_rx_count = 0;

/* External motor handles (array of 4 joint motors, IDs 1-4) */
extern EL05_MotorHandle_t g_el05_motors[4];

/* ===== LQR Balance variables ===== */
LQR_Controller_t g_lqr_controller;
LQR_TuningParams_t g_lqr_tuning;
RobotState_t g_robot_state;
volatile uint8_t g_balance_enabled = 0;
volatile float g_accel_offset = -0.015f;  /* IMU校准: 0.00589 rad */
volatile float g_gyro_bias = 0.0f;   /* 陀螺仪零偏(rad/s) */
volatile float debug_lqr_body_angle = 0.0f;

/* ===== 数据日志缓冲区 (2500条, 每5次控制循环写一条) ===== */
#define LOG_SIZE 2500
#define LOG_DECIMATE 5
typedef struct {
    uint32_t tick;
    float    ay; float az; float gx;
    float    angle; float rate;
    float    torque;
} LogEntry_t;
volatile LogEntry_t g_log[LOG_SIZE];
volatile uint32_t g_log_idx = 0;
volatile float debug_lqr_wheel_torque = 0.0f;
volatile float debug_lqr_current_raw = 0.0f;
volatile uint8_t debug_lqr_enabled = 0;
volatile int16_t g_motor_test_cur = 0;
volatile uint8_t g_calibrate_zero = 0;  /* 设1标定当前姿态为平衡零点 */   /* 非0时覆盖LQR输出,直接设电流 */
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* Unique tasks (only defined in this file) */
void Task_EL05_Motor(void *argument);
//void Task_M0601C_Motor(void *argument);  // disabled for motor test
//void Task_CAN(void *argument);            // disabled for motor test
/* Other tasks (Task_IMU, Task_Remote, etc.) are declared in freertos_tasks.h */
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  mutex_CAN = osMutexNew(NULL);
  mutex_SPI1 = osMutexNew(NULL);
  mutex_SPI3 = osMutexNew(NULL);
  mutex_UART1 = osMutexNew(NULL);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  sem_IMU_Ready = osSemaphoreNew(1, 0, NULL);
  sem_Remote_Ready = osSemaphoreNew(1, 0, NULL);
  sem_CAN_RX = osSemaphoreNew(1, 0, NULL);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  queue_IMUData = osMessageQueueNew(10, sizeof(IMU_Data_t), NULL);
  queue_RemoteData = osMessageQueueNew(5, sizeof(RemoteData_t), NULL);
  queue_EL05_MotorCmd = osMessageQueueNew(10, sizeof(MotorCmd_t), NULL);
  queue_M0601C_MotorCmd = osMessageQueueNew(10, sizeof(MotorCmd_t), NULL);
  queue_CAN_TX = osMessageQueueNew(20, sizeof(CAN_TxHeaderTypeDef), NULL);
  queue_CAN_RX = osMessageQueueNew(20, sizeof(CAN_RxHeaderTypeDef), NULL);
  queue_MotorCmd = osMessageQueueNew(10, sizeof(MotorCmd_t), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  /* defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes); */

  /* USER CODE BEGIN RTOS_THREADS */
  /* IMU data processing task - ICM-42688-P (1kHz, triggered by TIM2 ISR) */
  {
    const osThreadAttr_t attr = {
      .name = "IMU_Task",
      .stack_size = 1024,
      .priority = (osPriority_t) osPriorityAboveNormal,
    };
    taskHandle_IMU = osThreadNew(Task_IMU, NULL, &attr);
  }

  /* Remote controller task - NRF24L01+ (100Hz) */
  {
    const osThreadAttr_t attr = {
      .name = "Remote_Task",
      .stack_size = 2048,
      .priority = (osPriority_t) osPriorityAboveNormal1,
    };
    taskHandle_Remote = osThreadNew(Task_Remote, NULL, &attr);
  }

  /* EL05 joint motor task (100Hz) */
  {
    const osThreadAttr_t attr = {
      .name = "EL05_Motor_Task",
      .stack_size = 2048,
      .priority = (osPriority_t) osPriorityAboveNormal,
    };
    taskHandle_EL05_Motor = osThreadNew(Task_EL05_Motor, NULL, &attr);
  }

#if 0  // DISABLED for motor test
  /* M0601C wheel motor task (50Hz) - reads remote joystick → drives wheels */
  {
    const osThreadAttr_t attr = {
      .name = "M0601C_Motor_Task",
      .stack_size = 2048,
      .priority = (osPriority_t) osPriorityAboveNormal,
    };
    taskHandle_M0601C_Motor = osThreadNew(Task_M0601C_Motor, NULL, &attr);
  }

  /* CAN communication task (500Hz) */
  {
    const osThreadAttr_t attr = {
      .name = "CAN_Task",
      .stack_size = 2048,
      .priority = (osPriority_t) osPriorityAboveNormal2,
    };
    taskHandle_CAN = osThreadNew(Task_CAN, NULL, &attr);
  }
#endif

  /* Balance control task */
  {
    const osThreadAttr_t attr = {
      .name = "Balance_Task",
      .stack_size = 4096,
      .priority = (osPriority_t) osPriorityAboveNormal2,
    };
    taskHandle_Balance = osThreadNew(Task_Balance, NULL, &attr);
  }

  /* System monitor task (10Hz) */
  {
    const osThreadAttr_t attr = {
      .name = "Monitor_Task",
      .stack_size = 1024,
      .priority = (osPriority_t) osPriorityBelowNormal,
    };
    taskHandle_Monitor = osThreadNew(Task_Monitor, NULL, &attr);
  }

  /* Debug output task (1Hz) */
  {
    const osThreadAttr_t attr = {
      .name = "Debug_Task",
      .stack_size = 1024,
      .priority = (osPriority_t) osPriorityLow,
    };
    taskHandle_Debug = osThreadNew(Task_Debug, NULL, &attr);
  }
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  (void)argument;
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* ============================================================================
 *                          EL05 MOTOR DEBUG VARIABLES
 * ============================================================================
 */
volatile float    debug_m1_pos_target   = 0.0f;
volatile float    debug_m1_kp           = 0.0f;
volatile float    debug_m1_kd           = 0.0f;
volatile uint8_t  debug_m1_state        = 0;
volatile uint8_t  debug_m1_is_online    = 0;
volatile float    debug_m1_fb_pos       = 0.0f;
volatile float    debug_m1_fb_vel       = 0.0f;
volatile float    debug_m1_fb_trq       = 0.0f;
volatile float    debug_m1_fb_temp      = 0.0f;
volatile uint8_t  debug_m1_fb_fault     = 0;
volatile uint32_t debug_m1_phase_elapsed = 0;
volatile uint32_t debug_m1_loop_count   = 0;
volatile uint32_t debug_m1_can_status   = 0;
volatile float    debug_targets[4]      = {0};     // 4个电机当前目标
volatile uint8_t  debug_active_idx      = 0;       // 当前缓动电机索引
volatile uint8_t  debug_enable_flags[4] = {0};     // 使能返回值
volatile uint32_t debug_can_tx_ok[4]   = {0};     // MIT发送成功计数
volatile uint32_t debug_can_tx_fail[4] = {0};     // MIT发送失败计数
volatile uint8_t  debug_task_phase      = 0;
volatile uint8_t  debug_wheel_cmd_status = 0;  /* 0=未发,1=成功,2=失败 */
volatile uint8_t  debug_right_wheel_status = 0; /* 右轮毂(右腿,ID=2)指令状态 */
volatile uint8_t  debug_uart2_init_ok = 0;
volatile uint8_t dr0,dr1,dr2,dr3,dr4,dr5,dr6,dr7,dr8,dr9;  /* 反馈帧10字节 */  /* µVision Watch用 */
volatile float    debug_m3_fb_pos       = 0.0f;    // 电机3位置反馈
volatile float    debug_m3_fb_fault     = 0.0f;    // 电机3报错
volatile uint8_t  debug_m3_is_online    = 0;       // 电机3在线
volatile float    debug_action1_fb_pos       = 0.0f;    // 电机4位置反馈
volatile float    debug_action1_fb_fault     = 0.0f;    // 电机4报错
volatile uint8_t  debug_action1_is_online    = 0;       // 电机4在线

/* CAN RX调试计数器 (定义在 Motor_Drivers/EL05_MOTOR_DRIVE/Core/Src/el05_motor.c) */
extern volatile uint32_t g_dbg_cb_fired;
extern volatile uint32_t g_dbg_rx_msg_ok;
extern volatile uint32_t g_dbg_rx_ext;
extern volatile uint32_t g_dbg_rx_mode2;
extern volatile uint32_t g_dbg_rx_matched;
extern volatile uint32_t g_dbg_rx_ide;
extern volatile uint32_t g_dbg_rx_mode;
extern volatile uint32_t g_dbg_rx_mid;

/* 编译的 el05_motor.o 用 motor1 接收CAN反馈 */
extern EL05_MotorHandle_t motor1;

/* ============================================================================
 *                          动作组0: 最小腿高 (归零位置)
 *                           Action Group 0: Min Leg Height (Homing)
 * ============================================================================
 * 所有4个关节电机回到机械零点, 即最小腿高位置
 * ============================================================================ */

/* 动作组0: 各电机归零位置 (rad), 2π修正使电机从正确方向到达零点 */
static const float g_action0[4] = {
    0.0f,      /* M1/idx0: CCW小角度归零 */
    6.2832f,   /* M2/idx1: CW归零(2π) */
    6.2832f,   /* M3/idx2: CW归零(2π) 镜像1 */
    0.0f,      /* M4/idx3: CCW小角度归零 镜像2 */
};

/* ============================================================================
 *                          动作组1: 初始化到最大腿高
 *                           Action Group 1: Init Max Leg Height
 * ============================================================================
 * 在归零基础上转到最大腿高位置 → 保持
 *
 * 电机编号与安装:
 *   左腿: 电机1(髋前) + 电机3(髋后)  — 外侧对侧安装, 方向取反
 *   右腿: 电机2(髋前) + 电机4(髋后)  — 外侧对侧安装, 方向取反
 *
 * 角度约定: 先走动作组0归零, 再走动作组1到最大腿高
 * ============================================================================ */

/* 动作组1: 各电机目标位置 (rad) = 最大腿高 */
static const float g_action1[4] = {
    1.5708f,   /* M1/idx0: +90° */
    4.3633f,   /* M2/idx1: -110° (2π-1.9199) */
    4.7124f,   /* M3/idx2: -90°  (2π-π/2) 镜像1 */
    1.9199f,   /* M4/idx3: +110° 镜像2 */
};

void Task_EL05_Motor(void *argument)
{
    uint32_t tick;
    (void)argument;
    osDelay(200);

    /* Step 1: 4电机依次配置PP模式+限速限力矩 */
    for (int i = 0; i < 4; i++) {
        EL05_MotorHandle_t *m = &g_el05_motors[i];
        osMutexAcquire(mutex_CAN, osWaitForever);
        EL05_WriteParamU8(m, 0x7005, 1);
        osMutexRelease(mutex_CAN);      osDelay(2);
        osMutexAcquire(mutex_CAN, osWaitForever);
        EL05_WriteParam(m, 0x7017, 2.0f);
        osMutexRelease(mutex_CAN);      osDelay(2);
        osMutexAcquire(mutex_CAN, osWaitForever);
        EL05_WriteParam(m, 0x7018, 3.0f);
        osMutexRelease(mutex_CAN);      osDelay(2);
    }

    /* Step 2: 依次Disable清故障->Enable */
    for (int i = 0; i < 4; i++) {
        EL05_MotorHandle_t *m = &g_el05_motors[i];
        osMutexAcquire(mutex_CAN, osWaitForever);
        EL05_Disable(m);
        osMutexRelease(mutex_CAN);      osDelay(50);
        osMutexAcquire(mutex_CAN, osWaitForever);
        EL05_Enable(m);
        osMutexRelease(mutex_CAN);      osDelay(50);
    }
    osDelay(400);

    /* Step 3: 全部归零 */
    for (int i = 0; i < 4; i++) {
        EL05_MotorHandle_t *m = &g_el05_motors[i];
        osMutexAcquire(mutex_CAN, osWaitForever);
        EL05_WriteParam(m, 0x7016, g_action0[i]);
        osMutexRelease(mutex_CAN);
        debug_targets[i] = g_action0[i];
        osDelay(3);
    }
    osDelay(4000);

    /* Step 4: 切换到速度模式, 启动平衡 */
    osMutexAcquire(mutex_UART1, osWaitForever);
    MOTOR_SendModeSwitchCmd(1, MOTOR_CTRL_CURRENT);
    MOTOR_SendModeSwitchCmd(2, MOTOR_CTRL_CURRENT);
    osMutexRelease(mutex_UART1);
    osDelay(20);

    /* IMU零点校准: 等500ms后读50次平均 (accel offset + gyro bias) */
    {
        osDelay(500);
        float sum_a = 0.0f, sum_g = 0.0f;
        for (int i = 0; i < 50; i++) {
            ICM42688_RawData_t r = IMU_GetLatestRawData();
            float ay = (float)r.accel_y * 0.488f / 1000.0f;
            float az = (float)r.accel_z * 0.488f / 1000.0f;
            sum_a += atan2f(ay, az);
            sum_g += (float)r.gyro_x / 16.4f;  /* raw dps */
            osDelay(2);
        }
        g_accel_offset = sum_a / 50.0f;
        g_gyro_bias = (sum_g / 50.0f) * 0.017453f;  /* 平均零偏 → rad/s */
    }

    LQR_Enable(&g_lqr_controller);
    g_balance_enabled = 1;

    /* Step 5: 10Hz循环保持 — 关节电机最小腿高(动作组0) */
    tick = osKernelGetTickCount();
    for (;;) {
        for (int i = 0; i < 4; i++) {
            EL05_MotorHandle_t *m = &g_el05_motors[i];
            osMutexAcquire(mutex_CAN, osWaitForever);
            EL05_WriteParam(m, 0x7016, g_action0[i]);
            osMutexRelease(mutex_CAN);
            debug_targets[i] = g_action0[i];
        }

        debug_m1_fb_pos   = g_el05_motors[0].feedback.position;
        debug_m1_fb_fault = g_el05_motors[0].feedback.fault;
        debug_m3_fb_pos   = g_el05_motors[2].feedback.position;
        debug_m3_fb_fault = g_el05_motors[2].feedback.fault;
        debug_m3_is_online = g_el05_motors[2].is_online;
        debug_action1_fb_pos   = g_el05_motors[3].feedback.position;
        debug_action1_fb_fault = g_el05_motors[3].feedback.fault;
        debug_action1_is_online = g_el05_motors[3].is_online;
        g_el05_motor_update_count++;
        osDelayUntil(tick + 100);
    }
}

/**
  * @brief CAN debug variables
  */
volatile uint32_t g_can_esr = 0;
volatile uint32_t g_can_tsr = 0;

/**
  * @brief Raw CAN RX capture (updated for EVERY received frame)
  */
volatile uint32_t g_can_rx_raw_id = 0;
volatile uint8_t  g_can_rx_raw_ide = 0;
volatile uint8_t  g_can_rx_raw_dlc = 0;
volatile uint8_t  g_can_rx_raw_data0 = 0;

/**
  * @brief Motor feedback capture (updated by CAN RX callback)
  */
volatile int16_t g_motor_fb_pos_int = 0;
volatile int16_t g_motor_fb_vel_int = 0;
volatile int16_t g_motor_fb_trq_int = 0;
volatile uint16_t g_motor_fb_temp_int = 0;
volatile uint8_t  g_motor_fb_fault = 0;
volatile uint8_t  g_motor_fb_id = 0;
volatile uint8_t  g_motor_fb_mode_state = 0;

/* Raw CAN response bytes (for debugging position encoding) */
volatile uint8_t  g_can_rx_raw[8] = {0};

#if 0  // DISABLED for motor test
/**
  * @brief M0601C wheel motor control task (50Hz)
  * @note  Reads remote joystick from queue_RemoteData,
  *        maps left Y-axis to RPM, drives both M0601C wheel motors.
  */
void Task_M0601C_Motor(void *argument)
{
    RemoteData_t rc;
    osStatus_t status;
    uint32_t tick;

    (void)argument;

    /* --- Configurable parameters --- */
    #define WHEEL_RPM_MAX       200      /* Max wheel speed (RPM) */
    #define WHEEL_DEADBAND      8        /* Joystick deadband (±8) */
    #define MOTOR_ID_LEFT       1        /* Left wheel motor ID */
    #define MOTOR_ID_RIGHT      2        /* Right wheel motor ID */

    /* --- Initialize --- */
    MOTOR_StartReceive();

    /*
     * Ensure motors are in speed mode.
     * On first power-up, motors default to speed mode, so this is optional.
     * Uncomment if motors were left in position/current mode:
     *
     *   osMutexAcquire(mutex_UART1, osWaitForever);
     *   MOTOR_SendModeSwitchCmd(MOTOR_ID_LEFT,  MOTOR_CTRL_SPEED);
     *   MOTOR_SendModeSwitchCmd(MOTOR_ID_RIGHT, MOTOR_CTRL_SPEED);
     *   osMutexRelease(mutex_UART1);
     */

    tick = osKernelGetTickCount();

    for (;;)
    {
        /* --- 1. Get latest remote data (non-blocking, 20ms timeout) --- */
        status = osMessageQueueGet(queue_RemoteData, &rc, NULL, 20);

        if (status == osOK) {
            /* --- 2. Map left Y-axis to RPM --- */
            /*
             * rc.left_y: -128 (forward) … 0 (center) … 127 (backward)
             *
             *    forward (negative):  rpm = +max   (wheel spins forward)
             *    backward (positive): rpm = -max   (wheel spins reverse)
             *    center  (±deadband): rpm = 0
             */
            int32_t raw = rc.left_y;
            int16_t rpm_left, rpm_right;

            /* Apply deadband */
            if (raw > -WHEEL_DEADBAND && raw < WHEEL_DEADBAND) {
                rpm_left  = 0;
                rpm_right = 0;
            } else {
                /* Map -128..-9 → +max..+1,  +9..+127 → -1..-max */
                int32_t scaled;
                if (raw < 0) {
                    scaled = (raw + WHEEL_DEADBAND) * WHEEL_RPM_MAX / (128 - WHEEL_DEADBAND);
                    rpm_left  = (int16_t)(-scaled);   /* forward */
                    rpm_right = (int16_t)(-scaled);
                } else {
                    scaled = (raw - WHEEL_DEADBAND) * WHEEL_RPM_MAX / (127 - WHEEL_DEADBAND);
                    rpm_left  = (int16_t)(-scaled);   /* backward */
                    rpm_right = (int16_t)(-scaled);
                }
            }

            /* --- 3. Send speed commands to both wheel motors --- */
            osMutexAcquire(mutex_UART1, osWaitForever);
            MOTOR_SetSpeed(MOTOR_ID_LEFT,  rpm_left);
            MOTOR_SetSpeed(MOTOR_ID_RIGHT, rpm_right);
            osMutexRelease(mutex_UART1);

        } else {
            /* --- 4. No remote data — safety stop --- */
            osMutexAcquire(mutex_UART1, osWaitForever);
            MOTOR_Stop(MOTOR_ID_LEFT);
            MOTOR_Stop(MOTOR_ID_RIGHT);
            osMutexRelease(mutex_UART1);
        }

        g_m0601c_motor_update_count++;
        osDelayUntil(tick + 20);  /* 50Hz */
        tick += 20;
    }
}
#endif /* DISABLED for motor test */

#if 0  // DISABLED for motor test
/**
  * @brief CAN communication management task (500Hz)
  * @note  Handles CAN TX/RX for EL05 motors
  */
void Task_CAN(void *argument)
{
    (void)argument;

    uint32_t tick = osKernelGetTickCount();
    for (;;)
    {
        /* Sample CAN error/status registers for debugging */
        if (hcan1.Instance) {
            g_can_esr = hcan1.Instance->ESR;   /* Error Status Register */
            g_can_tsr = hcan1.Instance->TSR;   /* Transmit Status Register */
        }

        g_can_tx_count++;
        g_can_rx_count++;
        osDelayUntil(tick + 2);
        tick += 2;
    }
}
#endif /* DISABLED for motor test */

/**
  * @brief Stack overflow hook function
  * @note  Called when stack overflow is detected
  */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;

    /* Stack overflow detected - enter infinite loop for debugging */
    taskDISABLE_INTERRUPTS();
    for (;;);
}

/* ============================================================================
 *                          IMU ISR GLUE CODE
 * ============================================================================ */

/**
  * @brief Start TIM2 for precise 1 kHz IMU read interrupts
  */
void IMU_StartTimerInterrupt(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();

    TIM2->PSC = 84 - 1;
    TIM2->ARR = 1000 - 1;
    TIM2->DIER |= TIM_DIER_UIE;

    HAL_NVIC_SetPriority(TIM2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

    TIM2->CR1 |= TIM_CR1_CEN;
}

/**
  * @brief IMU ISR handler — called from TIM2_IRQHandler
  */
void IMU_ISR_Handler(void)
{
    ICM42688_RawData_t raw;

    if (ICM42688_ReadRawData_FromISR(&raw)) {
        uint32_t idx = g_imu_raw_active_idx;
        g_imu_raw_buf[idx] = raw;
        g_imu_raw_active_idx ^= 1;

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(taskHandle_IMU, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
  * @brief Get the latest raw data from the double buffer
  */
ICM42688_RawData_t IMU_GetLatestRawData(void)
{
    return g_imu_raw_buf[g_imu_raw_active_idx ^ 1];
}

/* ============================================================================
 *                          TASK IMPLEMENTATIONS
 * ============================================================================ */

/**
  * @brief IMU data processing task
  */
void Task_IMU(void *argument)
{
    IMU_Data_t imuData;

    (void)argument;

    if (!ICM42688_Init()) {
        g_system_status |= 0x01;
        vTaskSuspend(NULL);
    }

    IMU_StartTimerInterrupt();

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        ICM42688_RawData_t raw = IMU_GetLatestRawData();
        ICM42688_ProcessRawData(&raw);

        imuData.accel_x_g = g_imu_accel_x_filtered;
        imuData.accel_y_g = g_imu_accel_y_filtered;
        imuData.accel_z_g = g_imu_accel_z_filtered;
        imuData.gyro_x_dps = g_imu_gyro_x_filtered;
        imuData.gyro_y_dps = g_imu_gyro_y_filtered;
        imuData.gyro_z_dps = g_imu_gyro_z_filtered;
        imuData.temperature_c = g_imu_temperature_c;
        imuData.timestamp = osKernelGetTickCount();

        osMessageQueuePut(queue_IMUData, &imuData, 0, 0);
        osSemaphoreRelease(sem_IMU_Ready);

        g_imu_update_count++;
    }
}

/**
  * @brief Balance control task
  * @note  IMU mounting: Y forward, X right, Z up (标准安装)
  *        前倾时: body_angle>0, u[1]=-Kx → K[1][0]<0为正确负反馈
  *        Complementary filter: 95% gyro + 5% accel
  *        Mapping: LEFT=+cur(forward), RIGHT=-cur(forward mirror)
  */
void Task_Balance(void *argument)
{
    uint32_t tick_start;
    (void)argument;

    #define WHEEL_ID_LEFT   1
    #define WHEEL_ID_RIGHT  2
    BalanceState_t balance_state;

    /* Wait for homing to complete */
    while (!g_balance_enabled) {
        osDelay(100);
    }

    /* Current mode init */
    osMutexAcquire(mutex_UART1, osWaitForever);
    MOTOR_SendModeSwitchCmd(WHEEL_ID_LEFT, MOTOR_CTRL_CURRENT);
    MOTOR_SendModeSwitchCmd(WHEEL_ID_RIGHT, MOTOR_CTRL_CURRENT);
    osMutexRelease(mutex_UART1);
    osDelay(50);

    /* g_accel_offset 已硬编码为校准值 */

    /* 陀螺零偏设为0(互补滤波自动收敛,防标定期间运动干扰) */
    g_gyro_bias = 0.0f;

    tick_start = osKernelGetTickCount();

    for (;;) {
        /* Read IMU raw data from ISR double buffer */
        ICM42688_RawData_t raw = IMU_GetLatestRawData();
        /* 标定: g_calibrate_zero=1时记录当前姿态为平衡零点 */
        if (g_calibrate_zero) {
            float ca = atan2f((float)raw.accel_y * 0.488f / 1000.0f, (float)raw.accel_z * 0.488f / 1000.0f);
            g_accel_offset = ca;
            g_calibrate_zero = 0;
        }
        float ax = (float)raw.accel_y * 0.488f / 1000.0f;
        float az = (float)raw.accel_z * 0.488f / 1000.0f;
        float gx = (float)raw.gyro_x / 16.4f;

        /* Z朝上IMU: atan2(ay,az)直立=π, 前倾=π-θ, 后倾=-π+θ.
         *   body_angle_raw = -(diff): 前倾>0, 后倾<0 */
        float accel_angle = atan2f(ax, az);
        float angle_diff = accel_angle - g_accel_offset;
        if (angle_diff > 3.14159f) angle_diff -= 2.0f * 3.14159f;
        else if (angle_diff < -3.14159f) angle_diff += 2.0f * 3.14159f;
        float body_angle_raw = -angle_diff;

        /* Gyro: rad/s, 减零偏. Z朝上:前倾gyro_x<0,取反使与body_angle同号 */
        float gyro_rate = -(gx * 0.017453f - g_gyro_bias);

        /* Complementary filter: 90% gyro + 10% accel */
        static float cf_angle = 0.0f;
        static uint32_t last_t = 0;
        uint32_t now_t = osKernelGetTickCount();
        float dt = (now_t - last_t) * 0.001f;
        if (dt > 0.001f && dt < 0.05f) {
            cf_angle = 0.90f * (cf_angle + gyro_rate * dt) + 0.10f * body_angle_raw;
        }
        /* 首次dt很大: 保持cf_angle初始值(0),不重置到body_angle_raw(运动污染) */
        last_t = now_t;

        /* Gyro LPF for D term */
        static float g_lpf = 0.0f;
        if (g_lpf == 0.0f) g_lpf = gyro_rate;
        g_lpf += (gyro_rate - g_lpf) * 0.1f;

        /* LQR state */
        balance_state.body_angle = cf_angle;
        balance_state.body_rate = gyro_rate;
        /* 轮位估算: 从扭矩积分 */
        /* wheel pos estimation removed */
        
        
        
        
        /* 轮速反馈: 从电机读取RPM,转rad/s,用于阻尼漂移 */
        MotorStatus_t *ms = MOTOR_GetStatus();
        float wheel_spd = (ms && ms->isValid) ? ms->speed * 0.10472f : 0.0f;
        balance_state.wheel_velocity = wheel_spd;
        balance_state.wheel_position = 0.0f;
        balance_state.wheel_position = 0.0f;
        balance_state.wheel_velocity = 0.0f;
        LQR_Update(&g_lqr_controller, &balance_state, dt);

        /* Current mode output + 重心偏后补偿 */
        float torque = g_lqr_controller.u[1];
        if (torque > -0.02f && torque < 0.02f) torque = 0.0f;
        int16_t cur = (int16_t)(torque * 27306.0f);
        if (cur > 32767) cur = 32767; if (cur < -32767) cur = -32767;

        /* 电机方向测试: g_motor_test_cur非0时覆盖输出 */
        if (g_motor_test_cur != 0) {
            cur = g_motor_test_cur;
        }

        osMutexAcquire(mutex_UART1, osWaitForever);
        MOTOR_SetCurrent(WHEEL_ID_LEFT, cur);
        MOTOR_SetCurrent(WHEEL_ID_RIGHT, -cur);
        osMutexRelease(mutex_UART1);

        /* Log */
        { static uint32_t log_cnt = 0;
          if (++log_cnt % LOG_DECIMATE == 0) {
            uint32_t li = g_log_idx++ % LOG_SIZE;
            g_log[li].tick  = uwTick;
            g_log[li].ay    = ax; g_log[li].az = az; g_log[li].gx = gx;
            g_log[li].angle = cf_angle; g_log[li].rate = g_lpf;
            g_log[li].torque= torque;
        } }

        /* Debug */
        debug_lqr_body_angle = cf_angle;
        debug_lqr_wheel_torque = torque;
        debug_lqr_current_raw = (float)cur;
        debug_lqr_enabled = 1;

        osDelayUntil(tick_start + 5);  /* 200Hz */
        tick_start += 5;
    }
}

/**
  * @brief Remote control task (100Hz)
  */
void Task_Remote(void *argument)
{
    RemoteData_t remoteData;
    RemoteControlData_t *rc;
    uint32_t tick_start;

    (void)argument;

    NRF24L01_RX_Init();

    if (!NRF24L01_RX_WaitForPairing()) {
        g_system_status |= 0x02;
    }

    tick_start = osKernelGetTickCount();

    for (;;) {
        if (NRF24L01_RX_ReadData()) {
            rc = NRF24L01_RX_GetData();

            remoteData.right_x = (int16_t)rc->right_joystick_x - 128;
            remoteData.right_y = (int16_t)rc->right_joystick_y - 128;
            remoteData.left_x = (int16_t)rc->left_joystick_x - 128;
            remoteData.left_y = (int16_t)rc->left_joystick_y - 128;
            remoteData.buttons = rc->button_state;
            remoteData.online = NRF24L01_RX_IsOnline();
            remoteData.timestamp = osKernelGetTickCount();

            osMessageQueuePut(queue_RemoteData, &remoteData, 0, 0);
            osSemaphoreRelease(sem_Remote_Ready);
            /* 检测遥控按钮按下,发送到ESP32 */
            {
                static uint8_t prev_btns = 0;
                if (remoteData.buttons != prev_btns && remoteData.buttons != 0) {
                    ESP32_COM_SendRemoteBtn(remoteData.buttons, prev_btns);
                }
                prev_btns = remoteData.buttons;
            }

            g_remote_update_count++;
        }

        osDelayUntil(tick_start + 10);
        tick_start += 10;
    }
}

/**
  * @brief System monitoring task (10Hz)
  */
void Task_Monitor(void *argument)
{
    uint32_t tick_start;

    (void)argument;

    tick_start = osKernelGetTickCount();

    for (;;) {
        if (!NRF24L01_RX_IsOnline()) {
            g_system_status |= 0x04;
        }

        if (g_imu_update_count == 0) {
            g_system_status |= 0x08;
        }

        osDelayUntil(tick_start + 100);
        tick_start += 100;
    }
}

/**
  * @brief Debug output task (1Hz)
  */
void Task_Debug(void *argument)
{
    uint32_t tick_start;

    (void)argument;

    tick_start = osKernelGetTickCount();

    for (;;) {
        osDelayUntil(tick_start + 1000);
        tick_start += 1000;
    }
}

/* USER CODE END Application */

/* FREERTOS_END_OF_FILE */
