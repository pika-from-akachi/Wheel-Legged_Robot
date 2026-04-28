/**
  ******************************************************************************
  * 快速集成代码片段 - 直接复制到你的main.c中使用
  ******************************************************************************
  */

/* 第1步：在main.c顶部添加头文件 */
#include "nrf24l01_rx.h"

/* 第2步：在main.c顶部添加全局变量（用于Keil Watch窗口查看） */
RemoteControlData_t remote_data = {0};          // 遥控器数据结构体
volatile int16_t debug_right_x = 0;             // 右摇杆X (-128~127)
volatile int16_t debug_right_y = 0;             // 右摇杆Y (-128~127)
volatile int16_t debug_left_x = 0;              // 左摇杆X (-128~127)
volatile int16_t debug_left_y = 0;              // 左摇杆Y (-128~127)
volatile uint8_t debug_buttons = 0;             // 按键位掩码
volatile uint8_t pairing_status = 0;            // 对码状态
volatile uint8_t online_status = 0;             // 在线状态
volatile uint32_t packet_count = 0;             // 数据包计数

/* 第3步：在main()函数中，MX_GPIO_Init()和MX_SPI3_Init()之后添加 */
NRF24L01_RX_Init();

// 等待对码（10秒超时）
if (!NRF24L01_RX_WaitForPairing()) {
    pairing_status = 0;  // 对码失败
    // 可以添加错误处理，比如闪烁LED
} else {
    pairing_status = 1;  // 对码成功
}

/* 第4步：在main()的while(1)循环中添加 */
while (1)
{
    // 读取遥控器数据
    if (NRF24L01_RX_ReadData()) {
        RemoteControlData_t *rc = NRF24L01_RX_GetData();
        remote_data = *rc;  // 复制到全局变量

        // 转换摇杆值到中心范围 (-128~127)
        debug_right_x = (int16_t)rc->right_joystick_x - 128;
        debug_right_y = (int16_t)rc->right_joystick_y - 128;
        debug_left_x = (int16_t)rc->left_joystick_x - 128;
        debug_left_y = (int16_t)rc->left_joystick_y - 128;
        debug_buttons = rc->button_state;

        packet_count++;  // 数据包计数

        // 在这里添加你的控制逻辑
        // 例如：电机控制、按键处理等
    }

    // 检查在线状态
    online_status = NRF24L01_RX_IsOnline();

    // 如果遥控器离线，添加安全保护
    if (!online_status) {
        // 停止电机等安全措施
    }

    HAL_Delay(10);  // 100Hz更新率
}

/* 第5步：确保SPI3已正确初始化（STM32CubeMX配置） */
// 在STM32CubeMX中：
// - Connectivity → SPI3 → Mode: Full-Duplex Master
// - Configuration → Parameter Settings:
//   - Clock Polarity: Low
//   - Clock Phase: 1 Edge
//   - Baud Rate: APB1 Peripheral Clock / 16 (或更低)
// - GPIO设置：
//   - PC10: SPI3_SCK
//   - PC11: SPI3_MISO
//   - PC12: SPI3_MOSI
//   - PC8: GPIO_Output (CE)
//   - PC9: GPIO_Output (CSN)

/* 第6步：在Keil调试时，打开Watch窗口添加以下变量 */
// View → Watch Window → Watch 1
// 添加：
//   remote_data
//   debug_right_x
//   debug_right_y
//   debug_left_x
//   debug_left_y
//   debug_buttons
//   pairing_status
//   online_status
//   packet_count

/* 第7步：数值说明 */
// 摇杆值：
//   -128 ~ 0: 向左/前推
//   0: 中心位置
//   0 ~ 127: 向右/后推
//
// 按键值：
//   debug_buttons位掩码：
//   Bit0: KEY1 (0x01)
//   Bit1: KEY2 (0x02)
//   Bit2: KEY3 (0x04)
//   Bit3: KEY4 (0x08)
//   Bit4: LB   (0x10)
//   Bit5: RB   (0x20)
//
// 示例：
//   0x01 = KEY1按下
//   0x02 = KEY2按下
//   0x03 = KEY1和KEY2同时按下

/* 第8步：按键处理示例代码 */
// 在主循环中添加：
if (debug_buttons & 0x01) {
    // KEY1按下
}
if (debug_buttons & 0x02) {
    // KEY2按下
}
if (debug_buttons & 0x04) {
    // KEY3按下
}
if (debug_buttons & 0x08) {
    // KEY4按下
}
if (debug_buttons & 0x10) {
    // LB按下
}
if (debug_buttons & 0x20) {
    // RB按下
}

/* 第9步：电机控制示例代码 */
// 死区处理（忽略小值）
int left_motor = debug_left_y;
int right_motor = debug_right_y;

if (abs(left_motor) < 10) {
    left_motor = 0;  // 死区±10
}
if (abs(right_motor) < 10) {
    right_motor = 0;
}

// 应用到电机
// EL05_VelocityControl(&motor_left, left_motor * 0.1f, 5.0f);
// EL05_VelocityControl(&motor_right, right_motor * 0.1f, 5.0f);