# NRF遥控器测试准备清单

## ⚠️ 重要：当前项目缺少SPI3配置！

你的项目目前只有CAN1，**没有SPI3**，无法与NRF24L01通信。

## 📋 完整测试步骤

### 第1步：在STM32CubeMX中添加SPI3

1. 打开 `WheelRobot.ioc`
2. 在左侧找到 **Connectivity → SPI3**
3. 点击 **SPI3**，配置如下：
   - **Mode**: Full-Duplex Master
   - **Hardware NSS Signal**: Disable

4. **Configuration → Parameter Settings**:
   ```
   Clock Polarity (CPOL): Low
   Clock Phase (CPHA): 1 Edge
   NSS Signal Type: Software
   Baud Rate: APB1 Peripheral Clock / 16 (约2.6MHz)
   Data Size: 8 Bits
   First Bit: MSB First
   ```

5. **GPIO Settings**（自动分配，确认引脚）:
   ```
   PC10 → SPI3_SCK
   PC11 → SPI3_MISO
   PC12 → SPI3_MOSI
   ```

### 第2步：在STM32CubeMX中添加GPIO（CE和CSN）

1. 点击 **PC8**，设置为：
   - **GPIO_Output**
   - **User Label**: NRF_CE

2. 点击 **PC9**，设置为：
   - **GPIO_Output**
   - **User Label**: NRF_CSN

3. （可选）点击 **PC7**，设置为：
   - **GPIO_Input**
   - **User Label**: NRF_IRQ
   - **GPIO Pull-up/Pull-down**: Pull-up

### 第3步：生成代码

1. 点击 **Project → Generate Code**（或按Ctrl+Shift+G）
2. 这会生成：
   - `Core/Src/spi.c`
   - `Core/Inc/spi.h`
   - 更新 `main.c`

### 第4步：复制NRF驱动文件到项目

将以下文件复制到你的项目：

```
Motor_Drivers/NRF24L01_DRIVER/Core/Inc/nrf24l01_rx.h  →  Core/Inc/
Motor_Drivers/NRF24L01_DRIVER/Core/Src/nrf24l01_rx.c  →  Core/Src/
```

### 第5步：修改main.c

在 `Core/Src/main.c` 中添加以下代码：

#### 5.1 添加头文件（在USER CODE BEGIN Includes）
```c
/* USER CODE BEGIN Includes */
#include "nrf24l01_rx.h"
/* USER CODE END Includes */
```

#### 5.2 添加全局变量（在USER CODE BEGIN PV）
```c
/* USER CODE BEGIN PV */
// 遥控器数据（Keil Watch窗口查看）
RemoteControlData_t remote_data = {0};
volatile int16_t debug_right_x = 0;
volatile int16_t debug_right_y = 0;
volatile int16_t debug_left_x = 0;
volatile int16_t debug_left_y = 0;
volatile uint8_t debug_buttons = 0;
volatile uint8_t pairing_status = 0;
volatile uint8_t online_status = 0;
volatile uint32_t packet_count = 0;
/* USER CODE END PV */
```

#### 5.3 初始化NRF（在USER CODE BEGIN 2）
```c
/* USER CODE BEGIN 2 */
// 初始化NRF24L01接收器
NRF24L01_RX_Init();

// 等待对码（10秒超时）
if (!NRF24L01_RX_WaitForPairing()) {
    pairing_status = 0;  // 对码失败
    // 可以添加LED闪烁指示错误
} else {
    pairing_status = 1;  // 对码成功
}
/* USER CODE END 2 */
```

#### 5.4 主循环读取数据（在USER CODE BEGIN 3）
```c
/* USER CODE BEGIN 3 */
// 读取遥控器数据
if (NRF24L01_RX_ReadData()) {
    RemoteControlData_t *rc = NRF24L01_RX_GetData();
    remote_data = *rc;

    // 转换摇杆值到中心范围 (-128~127)
    debug_right_x = (int16_t)rc->right_joystick_x - 128;
    debug_right_y = (int16_t)rc->right_joystick_y - 128;
    debug_left_x = (int16_t)rc->left_joystick_x - 128;
    debug_left_y = (int16_t)rc->left_joystick_y - 128;
    debug_buttons = rc->button_state;

    packet_count++;

    // 在这里添加你的控制逻辑
    // 例如：电机控制、按键处理等
}

// 检查在线状态
online_status = NRF24L01_RX_IsOnline();

// 安全保护：遥控器离线时停止电机
if (!online_status) {
    // 停止电机
}

HAL_Delay(10);  // 100Hz更新率
/* USER CODE END 3 */
```

### 第6步：在Keil中添加文件

1. 在Keil中，右键点击 **Application/User/Core**
2. 选择 **Add Existing Files to Group**
3. 添加：
   - `Core/Src/nrf24l01_rx.c`

4. 在 **Options for Target → C/C++ → Include Paths** 中添加：
   - `../Core/Inc`

### 第7步：编译项目

1. 点击 **Build**（F7）
2. 检查是否有错误
3. 常见错误解决：
   - 找不到 `hspi3`：确保SPI3已初始化
   - 找不到头文件：检查Include Paths

### 第8步：烧录并测试

1. **连接硬件**：
   ```
   NRF24L01    STM32F407
   ─────────────────────
   VCC    →   3.3V ⚠️
   GND    →   GND
   CE     →   PC8
   CSN    →   PC9
   SCK    →   PC10
   MOSI   →   PC12
   MISO   →   PC11
   IRQ    →   PC7 (可选)
   ```

2. **烧录程序**：点击 **Download**（F8）

3. **启动调试**：
   - 点击 **Debug → Start/Stop Debug Session**（Ctrl+F5）
   - 打开 **View → Watch Window → Watch 1**
   - 添加变量：
     ```
     pairing_status
     online_status
     debug_right_x
     debug_right_y
     debug_left_x
     debug_left_y
     debug_buttons
     packet_count
     ```

4. **运行程序**：点击 **Run**（F5）

5. **操作遥控器**：
   - 打开遥控器电源
   - 观察Watch窗口中的 `pairing_status` 变为 1
   - 操作摇杆，观察数值变化

## ✅ 测试检查清单

在测试前确认：

- [ ] SPI3已在STM32CubeMX中配置
- [ ] GPIO PC8(CE)、PC9(CSN)已配置
- [ ] 代码已生成（spi.c/spi.h）
- [ ] nrf24l01_rx.c已添加到Keil项目
- [ ] Include Paths已设置
- [ ] 编译无错误
- [ ] NRF24L01接线正确（VCC=3.3V）
- [ ] 遥控器电池已安装

## 🔍 预期结果

### 对码成功
```
pairing_status = 1
```

### 遥控器在线
```
online_status = 1
packet_count 持续增长
```

### 摇杆数值
```
debug_right_x: -128 ~ 127 (右摇杆左右)
debug_right_y: -128 ~ 127 (右摇杆前后)
debug_left_x:  -128 ~ 127 (左摇杆左右)
debug_left_y:  -128 ~ 127 (左摇杆前后)
```

### 按键状态
```
debug_buttons: 0x00 ~ 0x3F
Bit0: KEY1
Bit1: KEY2
Bit2: KEY3
Bit3: KEY4
Bit4: LB
Bit5: RB
```

## ❌ 常见问题

### 问题1: 编译错误 - 找不到hspi3
**原因**：SPI3未初始化
**解决**：在STM32CubeMX中配置SPI3并重新生成代码

### 问题2: pairing_status = 0（对码失败）
**原因**：
- NRF24L01接线错误
- 电源电压不正确
- 遥控器未开机

**解决**：
- 检查接线（VCC必须是3.3V，不是5V）
- 用万用表测量电压
- 确认遥控器已开机

### 问题3: online_status = 0（遥控器离线）
**原因**：
- 遥控器已休眠
- 通信距离过远

**解决**：
- 按任意按键唤醒遥控器
- 缩短通信距离（<8米）

### 问题4: 数值不变化
**原因**：
- 数据接收失败
- 校验和错误

**解决**：
- 检查 `packet_count` 是否增长
- 检查 `remote_data.data_valid` 是否为1

## 📌 快速测试代码

如果只想快速测试，可以在main.c中使用最简代码：

```c
/* USER CODE BEGIN Includes */
#include "nrf24l01_rx.h"
/* USER CODE END Includes */

/* USER CODE BEGIN 2 */
NRF24L01_RX_Init();
if (!NRF24L01_RX_WaitForPairing()) {
    while(1); // 对码失败，死循环
}
/* USER CODE END 2 */

/* USER CODE BEGIN 3 */
if (NRF24L01_RX_ReadData()) {
    RemoteControlData_t *rc = NRF24L01_RX_GetData();
    // 在这里设置断点，查看rc的值
}
HAL_Delay(10);
/* USER CODE END 3 */
```

然后在 `NRF24L01_RX_ReadData()` 的return语句前设置断点，查看接收到的数据。

---

## 🎯 总结

**不能直接烧录测试！** 必须先：

1. ✅ 在STM32CubeMX中配置SPI3
2. ✅ 配置GPIO（PC8, PC9）
3. ✅ 重新生成代码
4. ✅ 添加NRF驱动文件到项目
5. ✅ 修改main.c
6. ✅ 编译无错误
7. ✅ 正确接线（VCC=3.3V）
8. ✅ 烧录并调试

完成以上步骤后，才能测试遥控器数据接收！
