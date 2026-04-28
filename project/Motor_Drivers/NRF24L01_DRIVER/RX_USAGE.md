# NRF24L01+ 接收器驱动使用指南

## 📌 快速开始

### 硬件连接

```
NRF24L01模块    STM32F407
─────────────────────────
VCC    ───►   3.3V (⚠️不要接5V!)
GND    ───►   GND
CE     ───►   PC8
CSN    ───►   PC9
SCK    ───►   PC10 (SPI3_SCK)
MOSI   ───►   PC12 (SPI3_MOSI)
MISO   ───►   PC11 (SPI3_MISO)
IRQ    ───►   PC7 (可选)
```

### STM32CubeMX配置

#### 1. SPI3配置
- **Connectivity → SPI3**
- Mode: Full-Duplex Master
- Clock Polarity: Low
- Clock Phase: 1 Edge
- Prescaler: 16 (10.5MHz)
- 引脚: PC10(SCK), PC11(MISO), PC12(MOSI)

#### 2. GPIO配置
- **PC8 (CE)**: GPIO_Output, High speed
- **PC9 (CSN)**: GPIO_Output, High speed
- **PC7 (IRQ)**: GPIO_Input, Pull-up (可选)

---

## 🚀 代码集成

### 步骤1: 添加头文件

在 `main.c` 开头添加:

```c
#include "nrf24l01_rx.h"
```

### 步骤2: 初始化

在 `main()` 函数中,系统初始化后添加:

```c
// 系统初始化 (CubeMX生成)
HAL_Init();
SystemClock_Config();
MX_GPIO_Init();
MX_SPI3_Init();  // 确保SPI3已初始化

// 初始化NRF24L01接收器
NRF24L01_RX_Init();
```

### 步骤3: 主循环读取数据

在 `while(1)` 主循环中添加:

```c
while (1) {
    // 读取遥控数据
    if (NRF24L01_RX_ReadData()) {
        RemoteControlData_t *rc = NRF24L01_RX_GetData();

        // 使用遥控数据
        int left_speed = rc->left_motor_speed;    // -100 到 100
        int right_speed = rc->right_motor_speed;  // -100 到 100
        uint8_t button = rc->button_state;

        // 控制电机...
    }

    // 检查遥控器是否在线
    if (!NRF24L01_RX_IsOnline()) {
        // 超过500ms未收到数据,停止电机(安全保护)
    }

    HAL_Delay(10); // 100Hz控制频率
}
```

---

## 📊 数据格式

### 接收数据结构

```c
typedef struct {
    int8_t left_motor_speed;    // 左电机速度 (-100 到 100)
    int8_t right_motor_speed;   // 右电机速度 (-100 到 100)
    uint8_t button_state;       // 按键状态 (位掩码)
    uint8_t data_valid;         // 数据有效标志
    uint32_t timestamp;         // 接收时间戳 (ms)
} RemoteControlData_t;
```

### 数据包格式

发射器发送的数据包格式 (16字节):

| 字节 | 内容 | 说明 |
|------|------|------|
| 0 | 0xAA | 帧头 |
| 1 | left_motor_speed | 左电机速度 |
| 2 | right_motor_speed | 右电机速度 |
| 3 | button_state | 按键状态 |
| 4 | 0x55 | 帧尾 |
| 5 | checksum | 校验和(XOR) |

---

## 🎮 应用示例

### 示例1: 基本使用

```c
#include "nrf24l01_rx.h"

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI3_Init();

    NRF24L01_RX_Init();

    while (1) {
        if (NRF24L01_RX_ReadData()) {
            RemoteControlData_t *rc = NRF24L01_RX_GetData();

            // 打印调试信息
            printf("Left: %d, Right: %d, Button: 0x%02X\n",
                   rc->left_motor_speed, rc->right_motor_speed, rc->button_state);
        }
        HAL_Delay(10);
    }
}
```

### 示例2: 控制EL05电机

```c
while (1) {
    if (NRF24L01_RX_ReadData()) {
        RemoteControlData_t *rc = NRF24L01_RX_GetData();

        if (rc->data_valid) {
            // 转换为rad/s
            float left_vel = rc->left_motor_speed * 0.1f;
            float right_vel = rc->right_motor_speed * 0.1f;

            // 控制EL05电机
            EL05_MotorHandle_t motor_left = {.can_id = 0x01};
            EL05_MotorHandle_t motor_right = {.can_id = 0x02};

            EL05_VelocityControl(&motor_left, left_vel, 5.0f);
            EL05_VelocityControl(&motor_right, right_vel, 5.0f);

            // 按键控制
            if (rc->button_state & 0x01) {
                EL05_Enable(&motor_left);
                EL05_Enable(&motor_right);
            }
        }
    }

    // 安全保护: 遥控器离线时停止电机
    if (!NRF24L01_RX_IsOnline()) {
        EL05_Disable(&motor_left);
        EL05_Disable(&motor_right);
    }

    HAL_Delay(10);
}
```

### 示例3: 控制M0601C电机

```c
while (1) {
    if (NRF24L01_RX_ReadData()) {
        RemoteControlData_t *rc = NRF24L01_RX_GetData();

        // 转换为RPM
        int left_rpm = rc->left_motor_speed * 2;
        int right_rpm = rc->right_motor_speed * 2;

        // 控制M0601C电机
        MOTOR_SetSpeed(1, left_rpm);
        MOTOR_SetSpeed(2, right_rpm);
    }

    HAL_Delay(10);
}
```

---

## 🔧 API参考

### 初始化函数

| 函数 | 说明 |
|------|------|
| `void NRF24L01_RX_Init(void)` | 初始化接收器 |
| `void NRF24L01_RX_GPIO_Init(void)` | 初始化GPIO引脚 |

### 数据接收函数

| 函数 | 说明 |
|------|------|
| `uint8_t NRF24L01_RX_ReadData(void)` | 读取并解析数据,返回1表示成功 |
| `RemoteControlData_t* NRF24L01_RX_GetData(void)` | 获取解析后的数据指针 |
| `uint8_t NRF24L01_RX_CheckData(void)` | 检查是否有数据可读 |

### 状态函数

| 函数 | 说明 |
|------|------|
| `uint8_t NRF24L01_RX_IsOnline(void)` | 检查遥控器是否在线(500ms内收到数据) |
| `uint32_t NRF24L01_RX_GetLastReceiveTime(void)` | 获取最后接收时间 |

---

## ⚙️ 配置参数

如果需要修改配置,编辑 `nrf24l01_rx.c` 中的初始化函数:

### RF通道

```c
NRF24L01_RX_WriteRegister(NRF24L01_REG_RF_CH, 70); // 通道70 (2.470GHz)
```

### 接收地址

```c
uint8_t address[5] = {0x34, 0x43, 0x10, 0x10, 0x01}; // 必须与发射器一致
```

### 载荷宽度

```c
nrf_rx_handle.payload_width = 16; // 数据包长度(字节)
```

---

## 🔍 故障排除

| 问题 | 可能原因 | 解决方案 |
|------|----------|----------|
| 无法接收数据 | SPI未初始化 | 检查SPI3配置和时钟 |
| 数据校验失败 | 地址不匹配 | 确认发射器和接收器地址一致 |
| 频繁离线 | 通道干扰 | 更换RF通道 |
| 距离过短 | 功率设置低 | 检查发射器功率设置 |

---

## 📚 相关文档

- [完整驱动文档](README.md)
- [使用示例](Core/Src/nrf24l01_rx_example.c)
- [EL05电机驱动](../EL05_MOTOR_DRIVE/)
- [M0601C电机驱动](../M0601C_DRIVE/)

---

**祝开发顺利! | Happy Coding!**
