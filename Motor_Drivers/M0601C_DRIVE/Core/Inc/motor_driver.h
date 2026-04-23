/**
  ******************************************************************************
  * @file    motor_driver.h
  * @brief   M0601C 电机驱动模块头文件
  *          基于数据手册 UART 通信协议实现
  ******************************************************************************
  */

#ifndef __MOTOR_DRIVER_H
#define __MOTOR_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "usart.h"
#include <stdint.h>

/* ============================================================================ */
/*                              宏定义 / 常量                                   */
/* ============================================================================ */

/** @brief 单帧数据固定长度：10 字节 (DATA[0] ~ DATA[9]) */
#define MOTOR_FRAME_LEN           10U

/** @brief 设置 ID 指令需连续发送次数（电机收到 5 次后才生效） */
#define MOTOR_SET_ID_REPEAT_CNT   5U

/** @brief 模式切换指令中的模式值：电流环 */
#define MOTOR_MODE_CURRENT        0x01U
/** @brief 模式切换指令中的模式值：速度环 */
#define MOTOR_MODE_SPEED          0x02U
/** @brief 模式切换指令中的模式值：位置环 */
#define MOTOR_MODE_POSITION       0x03U

/** @brief 默认加速时间：0 表示加速最快 */
#define MOTOR_ACC_DEFAULT         0x00U

/** @brief 刹车使能值：0xFF（仅在速度环有效） */
#define MOTOR_BRAKE_ENABLE        0xFFU
/** @brief 刹车禁用值：0x00 */
#define MOTOR_BRAKE_DISABLE       0x00U

/* ============================================================================ */
/*                            枚举 / 结构体                                     */
/* ============================================================================ */

/**
 * @brief 电机控制模式枚举
 * @note  对应数据手册 3.4 节模式切换指令
 */
typedef enum {
    MOTOR_CTRL_CURRENT  = MOTOR_MODE_CURRENT,   /*!< 电流环模式 */
    MOTOR_CTRL_SPEED    = MOTOR_MODE_SPEED,     /*!< 速度环模式（电机上电默认） */
    MOTOR_CTRL_POSITION = MOTOR_MODE_POSITION   /*!< 位置环模式 */
} MotorCtrlMode_t;

/**
 * @brief 电机反馈数据结构体
 * @note  用于存储从电机接收的反馈帧数据
 */
typedef struct {
    uint8_t  motorId;        /*!< 电机ID */
    uint8_t  data[10];       /*!< 原始数据帧 */
    uint8_t  isValid;        /*!< 数据有效标志 */
} MotorFeedback_t;

/**
 * @brief 调试信息结构体
 * @note  用于在调试器中实时显示接收状态
 */
typedef struct {
    uint8_t dmaRxStarted;        /*!< DMA接收是否启动：1=成功，0=失败 */
    uint8_t rxCpltCallback;      /*!< 接收完成回调触发次数 */
    uint8_t crcError;            /*!< CRC校验错误计数 */
    uint8_t lastCrcCalc;         /*!< 最后计算的CRC值 */
    uint8_t lastCrcRecv;         /*!< 最后接收的CRC值 */
    uint8_t txStatus;            /*!< 发送状态：0=失败，1=成功 */
    uint8_t reserved[3];         /*!< 保留对齐 */
    uint8_t rxBufRaw[10];        /*!< 原始接收缓冲区 */
    uint8_t rxBufValid;          /*!< 原始缓冲区是否有数据 */
} DebugInfo_t;

/* ============================================================================ */
/*                           函数声明                                          */
/* ============================================================================ */

/**
 * @brief  计算 CRC-8/MAXIM 校验值
 * @param  pData : 待校验数据指针
 * @param  len   : 数据长度（字节）
 * @retval CRC8  校验结果
 * @note   多项式 0x31，初始值 0x00，输入/输出均反转（LSB First）
 */
uint8_t MOTOR_CRC8_Calc(const uint8_t *pData, uint8_t len);

/**
 * @brief  发送速度/电流/位置驱动指令（数据手册 3.1 节）
 * @param  motorId : 电机 ID 号，范围 1~4
 * @param  value   : 驱动给定值（16 位有符号数）
 *                   速度模式下：正值正转(RPM)，负值反转(RPM)
 *                   例：30  -> 0x001E（正转 30 RPM）
 *                       -30 -> 0xFFE2（反转 30 RPM）
 * @param  accTime : 加速时间，默认写 0 加速最快
 * @param  brake   : 刹车标志，MOTOR_BRAKE_ENABLE(0xFF) 刹车，MOTOR_BRAKE_DISABLE(0x00) 不刹车
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef MOTOR_SendDriveCmd(uint8_t motorId, int16_t value, uint8_t accTime, uint8_t brake);

/**
 * @brief  发送获取其他反馈指令（数据手册 3.3 节）
 * @param  motorId : 电机 ID 号，范围 1~4
 * @retval HAL_StatusTypeDef
 * @note   可获取电机温度、当前模式等信息，电机返回的响应帧需另行解析
 */
HAL_StatusTypeDef MOTOR_SendFeedbackCmd(uint8_t motorId);

/**
 * @brief  发送模式切换指令（数据手册 3.4 节）
 * @param  motorId : 电机 ID 号，范围 1~4
 * @param  mode    : 目标模式（电流环 / 速度环 / 位置环）
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef MOTOR_SendModeSwitchCmd(uint8_t motorId, MotorCtrlMode_t mode);

/**
 * @brief  发送电机 ID 设置指令（数据手册 3.5 节）
 * @param  newId : 要设置的新 ID 号，范围 1~4
 * @retval HAL_StatusTypeDef
 * @warning 设置 ID 时必须保证总线上只有一个电机！
 *          每次上电只允许设置一次，需连续发送 5 次电机才生效。
 */
HAL_StatusTypeDef MOTOR_SendSetIDCmd(uint8_t newId);

/**
 * @brief  封装函数：控制电机以指定速度运行
 * @param  motorId : 电机 ID 号，范围 1~4
 * @param  rpm     : 目标转速，单位 RPM，有符号（正数正转，负数反转）
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef MOTOR_SetSpeed(uint8_t motorId, int16_t rpm);

/**
 * @brief  封装函数：电机刹车（速度环下有效）
 * @param  motorId : 电机 ID 号，范围 1~4
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef MOTOR_Brake(uint8_t motorId);

/**
 * @brief  封装函数：电机停止（发送 0 速指令）
 * @param  motorId : 电机 ID 号，范围 1~4
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef MOTOR_Stop(uint8_t motorId);

/**
 * @brief  启动 DMA 接收电机反馈数据
 * @retval HAL_StatusTypeDef
 * @note   接收完成后会触发 HAL_UART_RxCpltCallback 回调
 */
HAL_StatusTypeDef MOTOR_StartReceive(void);

/**
 * @brief  获取最新的电机反馈数据
 * @retval MotorFeedback_t* 反馈数据指针
 */
MotorFeedback_t* MOTOR_GetFeedback(void);

/**
 * @brief  打印反馈帧数据（调试用）
 * @param  pFeedback : 反馈数据指针
 */
void MOTOR_PrintFeedback(MotorFeedback_t *pFeedback);

/**
 * @brief 电机状态数据结构体（解析后）
 * @note  用于存储解析后的物理量，方便调试器查看
 */
typedef struct {
    uint8_t  motorId;        /*!< 电机ID */
    int16_t  speed;          /*!< 当前转速 (RPM)，有符号 */
    int16_t  current;        /*!< 当前电流 (mA)，有符号 */
    int16_t  position;       /*!< 当前位置（根据模式不同含义不同） */
    uint8_t  temperature;    /*!< 电机温度 (°C) */
    uint8_t  mode;           /*!< 当前控制模式：01=电流环，02=速度环，03=位置环 */
    uint8_t  faultCode;      /*!< 故障码 */
    uint8_t  isValid;        /*!< 数据有效标志 */
    uint8_t  reserved[2];    /*!< 保留对齐 */
} MotorStatus_t;

/**
 * @brief  解析电机反馈数据
 * @param  pFeedback : 原始反馈数据指针
 * @param  pStatus   : 解析后的状态数据指针
 * @retval 0=失败，1=成功
 * @note   将原始10字节数据解析成物理量
 */
uint8_t MOTOR_ParseFeedback(const MotorFeedback_t *pFeedback, MotorStatus_t *pStatus);

/**
 * @brief  获取最新的电机状态（已解析）
 * @retval MotorStatus_t* 状态数据指针
 */
MotorStatus_t* MOTOR_GetStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_DRIVER_H */
