/**
  ******************************************************************************
  * 地址测试代码 - 测试两种地址方案
  ******************************************************************************
  */

/* 方案1: LSB first (当前实现) */
void test_address_lsb_first(void)
{
    uint8_t default_address[5] = {0x30, 0x42, 0x46, 0x58, 0x48}; // "0BFXH" reversed
    NRF24L01_RX_SetAddress(default_address);
}

/* 方案2: MSB first */
void test_address_msb_first(void)
{
    uint8_t default_address[5] = {0x48, 0x58, 0x46, 0x42, 0x30}; // "HXFB0" normal
    NRF24L01_RX_SetAddress(default_address);
}

/* 测试流程 */
void test_both_address_schemes(void)
{
    // 测试方案1
    NRF24L01_RX_Init();  // 使用LSB first
    if (NRF24L01_RX_WaitForPairing()) {
        // 方案1成功！地址应该是LSB first
        printf("LSB first address works!\n");
        return;
    }

    // 方案1失败，测试方案2
    NRF24L01_RX_DeInit();
    HAL_Delay(100);

    // 修改为MSB first
    uint8_t default_address[5] = {0x48, 0x58, 0x46, 0x42, 0x30};
    NRF24L01_RX_SetAddress(default_address);

    // 重新初始化
    NRF24L01_RX_Init();
    if (NRF24L01_RX_WaitForPairing()) {
        // 方案2成功！地址应该是MSB first
        printf("MSB first address works!\n");
        return;
    }

    // 两种方案都失败
    printf("Both address schemes failed!\n");
}

/* 在main.c中使用 */
/*
在 USER CODE BEGIN 2 中：

// 测试两种地址方案
test_both_address_schemes();

// 或者直接测试当前方案
NRF24L01_RX_Init();
if (!NRF24L01_RX_WaitForPairing()) {
    pairing_status = 0;
} else {
    pairing_status = 1;
}
*/