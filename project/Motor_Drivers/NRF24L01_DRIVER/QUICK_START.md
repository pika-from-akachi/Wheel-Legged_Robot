# NRF24L01+ 驱动快速集成指南

## 📌 STM32CubeMX 配置步骤

### 1. SPI3 配置

在STM32CubeMX中:

1. 进入 **Connectivity → SPI3**
2. 配置参数:
   - **Mode**: Full-Duplex Master
   - **Hardware NSS Signal**: Disable
   - **Frame Format**: Motorola
   - **Clock Polarity (CPOL)**: Low
   - **Clock Phase (CPHA)**: 1 Edge
   - **Data Size**: 8 Bits
   - **First Bit**: MSB First
   - **Prescaler**: 16 (时钟频率: 10.5MHz)

3. 引脚配置 (自动分配):
   - **PC10**: SPI3_SCK
   - **PC11**: SPI3_MISO
   - **PC12**: SPI3_MOSI

### 2. GPIO 配置

在STM32CubeMX中:

#### CE 引脚 (PC8)
- **GPIO mode**: Output Push Pull
- **GPIO Pull-up/Pull-down**: No pull-up and no pull-down
- **Maximum output speed**: High
- **User Label**: NRF_CE

#### CSN 引脚 (PC9)
- **GPIO mode**: Output Push Pull
- **GPIO Pull-up/Pull-down**: No pull-up and no pull-down
- **Maximum output speed**: High
- **User Label**: NRF_CSN

#### IRQ 引脚 (PC7, 可选)
- **GPIO mode**: External Interrupt Mode with Falling edge trigger detection
- **GPIO Pull-up/Pull-down**: Pull-up
- **User Label**: NRF_IRQ

### 3. NVIC 配置 (如果使用IRQ)

进入 **System Core → NVIC**:
- 勾选 **EXTI line[9:5] interrupts**
- 设置优先级 (建议: Preemption Priority = 5, Sub Priority = 0)

### 4. 时钟配置

确保APB1时钟已启用SPI3:
- **APB1 Peripheral Clocks**: SPI3 已勾选

## 📝 代码集成

### 步骤 1: 添加头文件

在 `main.c` 中添加:

```c
#include "nrf24l01.h"
#include "nrf24l01_hal_config.h"
```

### 步骤 2: 声明全局变量

```c
NRF24L01_Handle_t nrf_handle;
```

### 步骤 3: 初始化 (接收器示例)

在 `main()` 函数中,系统初始化后添加:

```c
// 初始化NRF24L01 GPIO
NRF24L01_GPIO_Init();

// 配置NRF24L01
NRF24L01_Init_t config = {
    .hspi = &hspi3,
    .ce_port = NRF24L01_CE_PORT,
    .ce_pin = NRF24L01_CE_PIN,
    .csn_port = NRF24L01_CSN_PORT,
    .csn_pin = NRF24L01_CSN_PIN,
    .irq_port = NRF24L01_IRQ_PORT,
    .irq_pin = NRF24L01_IRQ_PIN,
    .channel = 70,
    .payload_width = 16,
    .data_rate = NRF24L01_RF_DR_2MBPS,
    .tx_power = NRF24L01_RF_PWR_0DBM,
    .address = {0x34, 0x43, 0x10, 0x10, 0x01},
    .auto_ack = true,
    .dynamic_payload = false
};

NRF24L01_Init(&nrf_handle, &config);
NRF24L01_SetRxMode(&nrf_handle);
```

### 步骤 4: 主循环接收数据

```c
while (1) {
    int8_t left_speed, right_speed;
    uint8_t button;

    if (NRF24L01_ReceiveControlData(&left_speed, &right_speed, &button)) {
        // 处理遥控数据
        // 例如: 控制电机
    }

    HAL_Delay(10);
}
```

## 🔧 常见问题

### 1. 编译错误: 找不到头文件

**解决方案**: 在项目设置中添加头文件路径:
- Keil MDK: Options → C/C++ → Include Paths
- 添加: `Motor_Drivers/NRF24L01_DRIVER/Core/Inc`

### 2. SPI通信失败

**检查项**:
- SPI3时钟是否启用
- GPIO引脚是否正确配置
- SPI模式是否正确 (CPOL=Low, CPHA=1Edge)
- CSN引脚初始状态是否为高电平

### 3. 无法接收数据

**检查项**:
- 发射器和接收器是否使用相同的:
  - RF通道 (channel)
  - 地址 (address)
  - 数据速率 (data_rate)
  - 载荷宽度 (payload_width)
- CE引脚是否正确控制
- 是否已调用 `NRF24L01_SetRxMode()`

### 4. 距离过短

**解决方案**:
- 增加发射功率: `NRF24L01_RF_PWR_0DBM`
- 降低数据速率: `NRF24L01_RF_DR_1MBPS` 或 `NRF24L01_RF_DR_250KBPS`
- 检查天线连接
- 检查电源稳定性 (添加去耦电容)

## 📚 相关文档

- [完整驱动文档](README.md)
- [使用示例](Core/Src/nrf24l01_example.c)
- [硬件配置头文件](Core/Inc/nrf24l01_hal_config.h)

## ⚡ 快速测试

使用示例代码快速测试通信:

```c
// 发射器测试
uint8_t test_data[16] = {0xAA, 0x50, 0x50, 0x01, 0x55, 0xFF};
NRF24L01_Transmit(&nrf_handle, test_data, 16);

// 接收器测试
if (NRF24L01_DataReady(&nrf_handle)) {
    uint8_t rx_data[16];
    uint8_t length;
    NRF24L01_Receive(&nrf_handle, rx_data, &length);
    // 检查 rx_data 内容
}
```

---

**祝开发顺利! | Happy Coding!**
