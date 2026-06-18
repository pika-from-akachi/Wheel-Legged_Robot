/**
  ******************************************************************************
  * @file    motor_driver.c
  * @brief   M0601C 电机驱动模块实现
  *          基于 UART 串口通信，波特率 115200，数据位 8，停止位 1，无校验
  *          协议帧长度固定为 10 字节，最后 1 字节为 CRC-8/MAXIM 校验
  ******************************************************************************
  */

#include "motor_driver.h"

#include <string.h>

/* ============================================================================ */
/*                             私有变量                                         */
/* ============================================================================ */

/** @brief 接收缓冲区 */
static uint8_t s_rxBuf[MOTOR_FRAME_LEN];

/** @brief 反馈数据结构 */
static MotorFeedback_t s_feedback = {0};

/* ============================================================================ */
/*                        CRC-8/MAXIM 校验表（可选查表法）                      */
/* ============================================================================ */

/**
 * @brief CRC-8/MAXIM 查找表
 * @note  多项式：0x31（x^8 + x^5 + x^4 + 1）
 *        初始值：0x00，输入/输出均反转（RefIn=True, RefOut=True）
 *        与数据手册示例完全一致：
 *        01 64 00 00 00 00 00 00 00 -> CRC = 0x50
 *        01 64 00 1E 00 00 00 00 00 -> CRC = 0x18
 *        01 74 00 00 00 00 00 00 00 -> CRC = 0x04
 */
static const uint8_t s_crc8_maxim_table[256] = {
    0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83,
    0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
    0x9D, 0xC3, 0x21, 0x7F, 0xFC, 0xA2, 0x40, 0x1E,
    0x5F, 0x01, 0xE3, 0xBD, 0x3E, 0x60, 0x82, 0xDC,
    0x23, 0x7D, 0x9F, 0xC1, 0x42, 0x1C, 0xFE, 0xA0,
    0xE1, 0xBF, 0x5D, 0x03, 0x80, 0xDE, 0x3C, 0x62,
    0xBE, 0xE0, 0x02, 0x5C, 0xDF, 0x81, 0x63, 0x3D,
    0x7C, 0x22, 0xC0, 0x9E, 0x1D, 0x43, 0xA1, 0xFF,
    0x46, 0x18, 0xFA, 0xA4, 0x27, 0x79, 0x9B, 0xC5,
    0x84, 0xDA, 0x38, 0x66, 0xE5, 0xBB, 0x59, 0x07,
    0xDB, 0x85, 0x67, 0x39, 0xBA, 0xE4, 0x06, 0x58,
    0x19, 0x47, 0xA5, 0xFB, 0x78, 0x26, 0xC4, 0x9A,
    0x65, 0x3B, 0xD9, 0x87, 0x04, 0x5A, 0xB8, 0xE6,
    0xA7, 0xF9, 0x1B, 0x45, 0xC6, 0x98, 0x7A, 0x24,
    0xF8, 0xA6, 0x44, 0x1A, 0x99, 0xC7, 0x25, 0x7B,
    0x3A, 0x64, 0x86, 0xD8, 0x5B, 0x05, 0xE7, 0xB9,
    0x8C, 0xD2, 0x30, 0x6E, 0xED, 0xB3, 0x51, 0x0F,
    0x4E, 0x10, 0xF2, 0xAC, 0x2F, 0x71, 0x93, 0xCD,
    0x11, 0x4F, 0xAD, 0xF3, 0x70, 0x2E, 0xCC, 0x92,
    0xD3, 0x8D, 0x6F, 0x31, 0xB2, 0xEC, 0x0E, 0x50,
    0xAF, 0xF1, 0x13, 0x4D, 0xCE, 0x90, 0x72, 0x2C,
    0x6D, 0x33, 0xD1, 0x8F, 0x0C, 0x52, 0xB0, 0xEE,
    0x32, 0x6C, 0x8E, 0xD0, 0x53, 0x0D, 0xEF, 0xB1,
    0xF0, 0xAE, 0x4C, 0x12, 0x91, 0xCF, 0x2D, 0x73,
    0xCA, 0x94, 0x76, 0x28, 0xAB, 0xF5, 0x17, 0x49,
    0x08, 0x56, 0xB4, 0xEA, 0x69, 0x37, 0xD5, 0x8B,
    0x57, 0x09, 0xEB, 0xB5, 0x36, 0x68, 0x8A, 0xD4,
    0x95, 0xCB, 0x29, 0x77, 0xF4, 0xAA, 0x48, 0x16,
    0xE9, 0xB7, 0x55, 0x0B, 0x88, 0xD6, 0x34, 0x6A,
    0x2B, 0x75, 0x97, 0xC9, 0x4A, 0x14, 0xF6, 0xA8,
    0x74, 0x2A, 0xC8, 0x96, 0x15, 0x4B, 0xA9, 0xF7,
    0xB6, 0xE8, 0x0A, 0x54, 0xD7, 0x89, 0x6B, 0x35
};

/* ============================================================================ */
/*                             私有辅助函数                                     */
/* ============================================================================ */

/**
 * @brief  THVD1410DR RS485方向控制: 发送模式
 * @note   RE#=1, DE=1 → 驱动器使能, 接收器禁用
 */
static void RS485_TX_Mode(void)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);   /* RE# = 1 */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);   /* DE  = 1 */
}

/**
 * @brief  THVD1410DR RS485方向控制: 接收模式
 * @note   RE#=0, DE=0 → 驱动器禁用, 接收器使能
 */
static void RS485_RX_Mode(void)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET); /* RE# = 0 */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET); /* DE  = 0 */
}

/**
 * @brief  通过 USART2 发送一帧数据 (带RS485方向切换)
 */
static HAL_StatusTypeDef MOTOR_UART_Tx(const uint8_t *pBuf)
{
    HAL_StatusTypeDef status;
    uint32_t tick;

    RS485_TX_Mode();                                          /* 切到发送 */
    for (volatile int i = 0; i < 20; i++);                    /* 短延时等收发器稳定 */

    __HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_TC);             /* 清TC标志 */
    status = HAL_UART_Transmit(&huart2, pBuf, MOTOR_FRAME_LEN, 100U);

    /* 等待TC标志置位（最后一字节已从移位寄存器发出） */
    tick = HAL_GetTick();
    while (!__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC)) {
        if (HAL_GetTick() - tick > 10) break;
    }
    for (volatile int i = 0; i < 5; i++);                     /* 额外保险延时 */
    RS485_RX_Mode();                                          /* 切回接收 */

    return status;
}

/* ============================================================================ */
/*                             公有函数实现                                     */
/* ============================================================================ */

/**
 * @brief  计算 CRC-8/MAXIM
 */
uint8_t MOTOR_CRC8_Calc(const uint8_t *pData, uint8_t len)
{
    uint8_t crc = 0x00U;
    for (uint8_t i = 0U; i < len; i++) {
        crc = s_crc8_maxim_table[crc ^ pData[i]];
    }
    return crc;
}

/**
 * @brief  发送速度/电流/位置驱动指令（数据手册 3.1 节，图 18）
 * @n      帧格式：
 *             DATA[0] = 电机 ID
 *             DATA[1] = 0x64
 *             DATA[2] = 速度/电流/位置给定值高 8 位
 *             DATA[3] = 速度/电流/位置给定值低 8 位
 *             DATA[4] = 0x00
 *             DATA[5] = 0x00
 *             DATA[6] = 加速时间
 *             DATA[7] = 刹车（0xFF 刹车，0x00 不刹车）
 *             DATA[8] = 0x00
 *             DATA[9] = CRC8（校验范围 DATA[0]~DATA[8]）
 */
HAL_StatusTypeDef MOTOR_SendDriveCmd(uint8_t motorId, int16_t value, uint8_t accTime, uint8_t brake)
{
    uint8_t txBuf[MOTOR_FRAME_LEN];

    txBuf[0] = motorId;             /* 电机 ID：1~4 */
    txBuf[1] = 0x64U;               /* 驱动指令固定头 */
    txBuf[2] = (uint8_t)((value >> 8) & 0xFF); /* 给定值高字节 */
    txBuf[3] = (uint8_t)(value & 0xFF);        /* 给定值低字节 */
    txBuf[4] = 0x00U;
    txBuf[5] = 0x00U;
    txBuf[6] = accTime;             /* 加速时间，默认 0 */
    txBuf[7] = brake;               /* 刹车控制字节 */
    txBuf[8] = 0x00U;
    txBuf[9] = MOTOR_CRC8_Calc(txBuf, 9U); /* CRC-8/MAXIM */

    return MOTOR_UART_Tx(txBuf);
}

/**
 * @brief  发送获取其他反馈指令（数据手册 3.3 节，图 21）
 * @n      帧格式：
 *             DATA[0] = 电机 ID
 *             DATA[1] = 0x74
 *             DATA[2]~DATA[8] = 0x00
 *             DATA[9] = CRC8（校验范围 DATA[0]~DATA[8]）
 * @note   电机收到后会返回反馈数据帧，用户需在上位机或串口中断中自行解析
 */
HAL_StatusTypeDef MOTOR_SendFeedbackCmd(uint8_t motorId)
{
    uint8_t txBuf[MOTOR_FRAME_LEN];

    txBuf[0] = motorId;             /* 电机 ID：1~4 */
    txBuf[1] = 0x74U;               /* 获取反馈指令固定头 */
    txBuf[2] = 0x00U;
    txBuf[3] = 0x00U;
    txBuf[4] = 0x00U;
    txBuf[5] = 0x00U;
    txBuf[6] = 0x00U;
    txBuf[7] = 0x00U;
    txBuf[8] = 0x00U;
    txBuf[9] = MOTOR_CRC8_Calc(txBuf, 9U); /* CRC-8/MAXIM */

    return MOTOR_UART_Tx(txBuf);
}

/**
 * @brief  发送模式切换指令（数据手册 3.4 节，图 22）
 * @n      帧格式：
 *             DATA[0] = 电机 ID
 *             DATA[1] = 0xA0
 *             DATA[2]~DATA[8] = 0x00
 *             DATA[9] = 模式值（01=电流环，02=速度环，03=位置环）
 * @note   根据手册示例，该指令没有独立的 CRC8 字节，DATA[9] 直接填写模式值。
 *         若你的固件版本需要 CRC8，请在实际调试时对比返回帧确认。
 */
HAL_StatusTypeDef MOTOR_SendModeSwitchCmd(uint8_t motorId, MotorCtrlMode_t mode)
{
    uint8_t txBuf[MOTOR_FRAME_LEN];

    txBuf[0] = motorId;             /* 电机 ID：1~4 */
    txBuf[1] = 0xA0U;               /* 模式切换指令固定头 */
    txBuf[2] = 0x00U;
    txBuf[3] = 0x00U;
    txBuf[4] = 0x00U;
    txBuf[5] = 0x00U;
    txBuf[6] = 0x00U;
    txBuf[7] = 0x00U;
    txBuf[8] = 0x00U;
    txBuf[9] = (uint8_t)mode;       /* 模式值本身 */

    return MOTOR_UART_Tx(txBuf);
}

/**
 * @brief  发送电机 ID 设置指令（数据手册 3.5 节，图 23）
 * @n      帧格式：
 *             DATA[0] = 0xAA
 *             DATA[1] = 0x55
 *             DATA[2] = 0x53
 *             DATA[3] = 新 ID（1~4）
 *             DATA[4]~DATA[9] = 0x00
 * @warning
 *         1. 设置 ID 时请保证总线上只有一个电机；
 *         2. 每次上电只允许设置一次；
 *         3. 需连续发送 5 次后电机才生效。
 */
HAL_StatusTypeDef MOTOR_SendSetIDCmd(uint8_t newId)
{
    uint8_t txBuf[MOTOR_FRAME_LEN];
    HAL_StatusTypeDef status = HAL_OK;

    txBuf[0] = 0xAAU;
    txBuf[1] = 0x55U;
    txBuf[2] = 0x53U;
    txBuf[3] = newId;
    txBuf[4] = 0x00U;
    txBuf[5] = 0x00U;
    txBuf[6] = 0x00U;
    txBuf[7] = 0x00U;
    txBuf[8] = 0x00U;
    txBuf[9] = 0x00U;

    /* 保持TX连续发5帧, 避免方向切换丢帧 */
    RS485_TX_Mode();
    for (volatile int d = 0; d < 100; d++);

    for (uint8_t i = 0U; i < MOTOR_SET_ID_REPEAT_CNT; i++) {
        __HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_TC);
        status = HAL_UART_Transmit(&huart2, txBuf, MOTOR_FRAME_LEN, 100U);
        if (status != HAL_OK) break;
        uint32_t tick = HAL_GetTick();
        while (!__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC)) {
            if (HAL_GetTick() - tick > 10) break;
        }
        for (volatile int d = 0; d < 5000; d++);
    }
    RS485_RX_Mode();

    return status;
}

/**
 * @brief  控制电机以指定速度运行
 * @note   直接封装 MOTOR_SendDriveCmd，默认加速时间 0，不刹车
 *         例：MOTOR_SetSpeed(1, 30);   // 电机 1 正转 30 RPM
 *             MOTOR_SetSpeed(1, -30);  // 电机 1 反转 30 RPM
 */
HAL_StatusTypeDef MOTOR_SetSpeed(uint8_t motorId, int16_t rpm)
{
    return MOTOR_SendDriveCmd(motorId, rpm, MOTOR_ACC_DEFAULT, MOTOR_BRAKE_DISABLE);
}

/**
 * @brief  电流环控制（需先切换到电流模式）
 * @param  motorId     : 电机 ID 号，范围 1~4
 * @param  current_raw : 电流给定值，-32767~32767
 */
HAL_StatusTypeDef MOTOR_SetCurrent(uint8_t motorId, int16_t current_raw)
{
    return MOTOR_SendDriveCmd(motorId, current_raw, MOTOR_ACC_DEFAULT, MOTOR_BRAKE_DISABLE);
}

/**
 * @brief  电机刹车（仅在速度环模式下有效）
 * @note   发送速度=0、刹车=0xFF 的驱动指令
 */
HAL_StatusTypeDef MOTOR_Brake(uint8_t motorId)
{
    return MOTOR_SendDriveCmd(motorId, 0, MOTOR_ACC_DEFAULT, MOTOR_BRAKE_ENABLE);
}

/**
 * @brief  电机停止（发送 0 速指令，不刹车）
 */
HAL_StatusTypeDef MOTOR_Stop(uint8_t motorId)
{
    return MOTOR_SendDriveCmd(motorId, 0, MOTOR_ACC_DEFAULT, MOTOR_BRAKE_DISABLE);
}

/**
 * @brief  启动中断接收电机反馈数据
 * @retval HAL_StatusTypeDef
 * @note   改用 IT 模式 (非 DMA), 与 M0602C 例程一致
 */
HAL_StatusTypeDef MOTOR_StartReceive(void)
{
    RS485_RX_Mode();  /* 确保在接收模式 */
    return HAL_UART_Receive_IT(&huart2, s_rxBuf, MOTOR_FRAME_LEN);
}

/**
 * @brief  获取最新的电机反馈数据
 * @retval MotorFeedback_t* 反馈数据指针
 */
MotorFeedback_t* MOTOR_GetFeedback(void)
{
    return &s_feedback;
}

/**
 * @brief  打印反馈帧数据（调试用）
 * @param  pFeedback : 反馈数据指针
 */
void MOTOR_PrintFeedback(MotorFeedback_t *pFeedback)
{
    if (pFeedback == NULL || !pFeedback->isValid) {
        return;
    }

    /* 通过LED指示收到数据 */
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, GPIO_PIN_RESET);
}

/**
 * @brief  UART 接收完成回调函数
 * @param  huart : UART 句柄指针
 * @note   此函数由 HAL 库在 DMA 接收完成后自动调用
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        /* 声明外部调试变量 */
        extern uint8_t g_rxCpltCallback;
        extern uint8_t g_crcError;
        extern uint8_t g_rxBufRaw[10];
        extern DebugInfo_t g_debug;  /* 新增：调试结构体 */

        /* 标记回调已触发 */
        g_rxCpltCallback++;
        g_debug.rxCpltCallback++;

        /* 保存原始接收数据 */
        memcpy(g_rxBufRaw, s_rxBuf, MOTOR_FRAME_LEN);
        memcpy(g_debug.rxBufRaw, s_rxBuf, MOTOR_FRAME_LEN);
        g_debug.rxBufValid = 1;

        /* 验证 CRC */
        uint8_t crc_calc = MOTOR_CRC8_Calc(s_rxBuf, 9U);
        uint8_t crc_recv = s_rxBuf[9];

        /* 记录CRC值 */
        g_debug.lastCrcCalc = crc_calc;
        g_debug.lastCrcRecv = crc_recv;

        if (crc_calc == crc_recv) {
            /* CRC 校验通过，保存数据 */
            memcpy(s_feedback.data, s_rxBuf, MOTOR_FRAME_LEN);
            s_feedback.motorId = s_rxBuf[0];
            s_feedback.isValid = 1;

            /* LED闪烁指示接收成功 */
            HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_10);
        } else {
            /* CRC校验失败 */
            g_crcError++;
            g_debug.crcError++;
        }

        /* 重新启动接收 (IT模式) */
        HAL_UART_Receive_IT(&huart2, s_rxBuf, MOTOR_FRAME_LEN);
    }
}

/* ============================================================================
 *                             反馈数据解析
 * ============================================================================ */

/** @brief 解析后的电机状态数据 */
static MotorStatus_t s_motorStatus = {0};

/**
 * @brief  解析电机反馈数据
 * @param  pFeedback : 原始反馈数据指针
 * @param  pStatus   : 解析后的状态数据指针
 * @retval 0=失败，1=成功
 * @note   根据反馈帧格式解析物理量
 *
 *         常见反馈帧格式（需根据实际数据手册调整）：
 *         DATA[0] = 电机ID
 *         DATA[1] = 反馈帧类型标识
 *         DATA[2:3] = 速度（有符号16位）
 *         DATA[4:5] = 电流（有符号16位）
 *         DATA[6:7] = 位置（有符号16位）
 *         DATA[8] = 温度/状态
 *         DATA[9] = CRC
 */
uint8_t MOTOR_ParseFeedback(const MotorFeedback_t *pFeedback, MotorStatus_t *pStatus)
{
    if (pFeedback == NULL || pStatus == NULL || !pFeedback->isValid) {
        return 0;
    }

    const uint8_t *data = pFeedback->data;

    /* 电机ID */
    pStatus->motorId = data[0];

    /*
     * 根据Excel换算公式解析
     * 注意：DATA[2:3] = 电流，DATA[4:5] = 速度
     */

    /* 电流换算：DATA[2:3]
     * 公式：8000 * signed16 / 32767 (mA)
     */
    int16_t currentRaw = (int16_t)((data[2] << 8) | data[3]);
    pStatus->current = (int16_t)((8000L * currentRaw) / 32767);

    /* 速度换算：DATA[4:5]
     * 公式：signed16 (rpm)
     */
    pStatus->speed = (int16_t)((data[4] << 8) | data[5]);

    /* 位置换算：DATA[6:7]
     * 公式：unsigned16
     */
    pStatus->position = (int16_t)((data[6] << 8) | data[7]);

    /* 温度：DATA[8] */
    pStatus->temperature = data[8];

    /* 模式：根据帧类型判断（需要实际验证） */
    pStatus->mode = data[1];  /* 假设DATA[1]是模式 */

    /* 故障码：暂时设为0 */
    pStatus->faultCode = 0;

    pStatus->isValid = 1;

    return 1;
}

/**
 * @brief  获取最新的电机状态（已解析）
 * @retval MotorStatus_t* 状态数据指针
 */
MotorStatus_t* MOTOR_GetStatus(void)
{
    return &s_motorStatus;
}
