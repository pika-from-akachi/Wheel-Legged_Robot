# Keil Debugger 使用指南 - NRF遥控器调试

## 一、在Keil中设置调试变量

### 1. 打开Watch窗口

在Keil调试模式下：
1. 点击菜单 **View → Watch Window → Watch 1** (或按 `Ctrl+Alt+1`)
2. 或者在调试工具栏中点击Watch图标

### 2. 添加变量到Watch窗口

在Watch窗口中，点击空白处输入以下变量名：

#### 基础摇杆数据（推荐）
```
remote_data                  // 完整的遥控器数据结构体
debug_right_x                // 右摇杆X (-128 ~ 127)
debug_right_y                // 右摇杆Y (-128 ~ 127)
debug_left_x                 // 左摇杆X (-128 ~ 127)
debug_left_y                 // 左摇杆Y (-128 ~ 127)
```

#### 按键状态
```
debug_buttons                // 按键位掩码 (0x00-0x3F)
debug_key1                   // KEY1状态 (0=松开, 1=按下)
debug_key2                   // KEY2状态
debug_key3                   // KEY3状态
debug_key4                   // KEY4状态
debug_lb                     // LB状态
debug_rb                     // RB状态
```

#### 系统状态
```
pairing_status               // 对码状态 (0=未对码, 1=已对码)
online_status                // 在线状态 (0=离线, 1=在线)
packet_count                 // 接收数据包计数
rolling_code                 // 滚动码
last_receive_time            // 最后接收时间戳
```

## 二、查看结构体成员

在Watch窗口中，点击 `remote_data` 前面的 `+` 号展开，可以看到：

```
remote_data
  ├─ right_joystick_x        // 原始值 (0-255, 128=中心)
  ├─ right_joystick_y        // 原始值 (0-255, 128=中心)
  ├─ left_joystick_x         // 原始值 (0-255, 128=中心)
  ├─ left_joystick_y         // 原始值 (0-255, 128=中心)
  ├─ button_state            // 按键位掩码
  ├─ rolling_code            // 滚动码
  ├─ data_valid              // 数据有效标志
  └─ timestamp               // 时间戳
```

## 三、数值说明

### 摇杆值

#### 原始值 (right_joystick_x/y, left_joystick_x/y)
- **范围**: 0 ~ 255 (uint8_t)
- **中心**: 128 (摇杆未推动)
- **最小**: 0 (向左/前推到底)
- **最大**: 255 (向右/后推到底)

#### 转换值 (debug_right_x/y, debug_left_x/y)
- **范围**: -128 ~ 127 (int16_t)
- **中心**: 0 (摇杆未推动)
- **负值**: 向左/前推 (-128 = 推到底)
- **正值**: 向右/后推 (127 = 推到底)

**示例**：
```
原始值 = 128  →  转换值 = 0    (中心位置)
原始值 = 64   →  转换值 = -64  (向左/前推一半)
原始值 = 192  →  转换值 = 64   (向右/后推一半)
原始值 = 0    →  转换值 = -128 (向左/前推到底)
原始值 = 255  →  转换值 = 127  (向右/后推到底)
```

### 按键值

#### 位掩码 (debug_buttons)
- **范围**: 0x00 ~ 0x3F (6个按键)
- **格式**: 二进制 Bit5~Bit0 对应 RB, LB, KEY4, KEY3, KEY2, KEY1

**示例**：
```
0x00 = 0b000000  →  所有按键松开
0x01 = 0b000001  →  KEY1按下
0x02 = 0b000010  →  KEY2按下
0x03 = 0b000011  →  KEY1和KEY2同时按下
0x10 = 0b010000  →  LB按下
0x20 = 0b100000  →  RB按下
0x3F = 0b111111  →  所有按键按下
```

#### 单独按键 (debug_key1~6)
- **0**: 按键松开
- **1**: 按键按下

## 四、实时监控步骤

### 1. 编译并下载程序
1. 在Keil中点击 **Build** (F7)
2. 点击 **Debug** → **Start/Stop Debug Session** (Ctrl+F5)
3. 点击 **Run** (F5)

### 2. 打开Watch窗口
按照第一节的步骤打开Watch窗口并添加变量

### 3. 观察对码过程
程序启动后会自动进行对码，观察：
```
pairing_status: 0 → 1  // 对码成功
```

如果 `pairing_status` 保持为 0，说明对码失败，检查：
- 遥控器是否开机
- NRF24L01接线是否正确
- 电源是否正常

### 4. 操作遥控器并观察数据

#### 测试摇杆
1. **右摇杆左右推**：观察 `debug_right_x` 变化
   - 向左推：-128 ~ 0
   - 向右推：0 ~ 127

2. **右摇杆前后推**：观察 `debug_right_y` 变化
   - 向前推：-128 ~ 0
   - 向后推：0 ~ 127

3. **左摇杆左右推**：观察 `debug_left_x` 变化
4. **左摇杆前后推**：观察 `debug_left_y` 变化

#### 测试按键
1. **按下KEY1**：观察 `debug_key1` 变为 1
2. **按下KEY2**：观察 `debug_key2` 变为 1
3. **同时按下多个按键**：观察 `debug_buttons` 的位掩码值

### 5. 检查在线状态
```
online_status: 1  // 遥控器在线
online_status: 0  // 遥控器离线（超过500ms未收到数据）
```

## 五、使用Logic Analyzer查看波形

### 1. 打开Logic Analyzer
点击 **View → Analysis Window → Logic Analyzer**

### 2. 添加信号
在Logic Analyzer窗口中点击 **Setup**，添加以下变量：

```
debug_right_x        // 右摇杆X波形
debug_right_y        // 右摇杆Y波形
debug_left_x         // 左摇杆X波形
debug_left_y         // 左摇杆Y波形
```

### 3. 设置显示范围
- **Min**: -128
- **Max**: 127
- **Display Type**: Analog (模拟波形)

### 4. 运行并观察
点击Run，操作遥控器摇杆，可以看到实时波形变化。

## 六、常见问题排查

### 问题1: pairing_status = 0 (对码失败)

**原因**：
- 遥控器未开机
- NRF24L01接线错误
- 电源电压不正确（应为3.3V）
- SPI配置错误

**解决**：
1. 确认遥控器已开机（指示灯闪烁）
2. 检查NRF24L01接线
3. 用万用表测量VCC是否为3.3V
4. 检查SPI3配置

### 问题2: online_status = 0 (遥控器离线)

**原因**：
- 遥控器已休眠（60秒无操作）
- 遥控器电池电量低
- 通信距离过远（>8米）

**解决**：
1. 按任意按键唤醒遥控器
2. 更换遥控器电池
3. 缩短通信距离

### 问题3: 摇杆值不变化

**原因**：
- 遥控器未连接
- 数据校验失败

**解决**：
1. 检查 `online_status` 是否为 1
2. 检查 `remote_data.data_valid` 是否为 1
3. 检查 `packet_count` 是否在增加

### 问题4: 数值异常

**现象**：摇杆值不在预期范围

**检查**：
```
remote_data.right_joystick_x  // 应为 0-255
debug_right_x                 // 应为 -128~127
```

如果数值异常，可能是：
- 数据包格式错误
- 校验和失败
- NRF24L01配置错误

## 七、调试技巧

### 1. 设置断点
在以下位置设置断点：
```c
// 在 NRF24L01_RX_ReadData() 函数中
if (checksum != nrf_rx_handle.rx_buffer[15]) {
    // 设置断点，检查校验和错误
}

// 在 Update_Debug_Variables() 函数中
debug_right_x = (int16_t)remote_data.right_joystick_x - 128;
// 设置断点，检查数据更新
```

### 2. 单步调试
1. 在关键位置设置断点
2. 运行到断点处
3. 使用 **Step Into** (F11) 或 **Step Over** (F10) 单步执行
4. 观察变量变化

### 3. 查看内存
点击 **View → Memory Window**，输入地址查看原始数据：
```
&remote_data          // 查看remote_data结构体内存
nrf_rx_handle.rx_buffer  // 查看原始接收缓冲区
```

### 4. 使用串口打印（可选）
如果需要更详细的调试信息，可以添加串口打印：

```c
// 在 Update_Debug_Variables() 中添加
printf("RX:%d RY:%d LX:%d LY:%d BTN:0x%02X\n",
       debug_right_x, debug_right_y,
       debug_left_x, debug_left_y,
       debug_buttons);
```

## 八、Watch窗口布局建议

### 推荐布局

**Watch 1** - 主要数据
```
remote_data
debug_right_x
debug_right_y
debug_left_x
debug_left_y
debug_buttons
```

**Watch 2** - 系统状态
```
pairing_status
online_status
packet_count
rolling_code
last_receive_time
```

**Watch 3** - 按键状态
```
debug_key1
debug_key2
debug_key3
debug_key4
debug_lb
debug_rb
```

## 九、数据记录

### 使用Keil的记录功能

1. 在Watch窗口中右键点击变量
2. 选择 **Add to Watch List**
3. 点击 **View → Watch List**
4. 可以导出变量历史记录

### 导出数据到CSV

如果需要分析数据，可以：
1. 使用串口输出数据到PC
2. 使用串口调试助手记录数据
3. 导出为CSV格式进行分析

## 十、性能监控

### 查看更新频率

观察 `packet_count` 的增长速度：
- 正常：每秒增加约100（100Hz更新率）
- 异常：增长过慢或停止

### 查看通信延迟

观察 `last_receive_time`：
- 正常：持续更新
- 异常：停止更新（遥控器离线）

---

## 快速检查清单

在Keil调试时，按以下顺序检查：

- [ ] `pairing_status = 1` (对码成功)
- [ ] `online_status = 1` (遥控器在线)
- [ ] `packet_count` 持续增长 (正常接收数据)
- [ ] `remote_data.data_valid = 1` (数据有效)
- [ ] 摇杆值在预期范围内 (-128 ~ 127)
- [ ] 按键状态正确 (0 或 1)

如果所有项都正常，说明遥控器工作正常！
