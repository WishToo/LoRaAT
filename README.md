# 关于 泽耀Uart LoRa 库

这是一个支持 A39C-T400A22S1a/A39C-T400A22D1a/A39C-T400A30S1a/A39C-T400A30D1a 的 Arduino 库，该模块工作于 UART 模式下，可以通过简单设置来启用该模块正常工作。


# 一个简单示例

```
#include "LoRaAT.h"

// LoRA MO对应引脚
#define LORA_MO_PIN 5     // MO引脚
#define LORA_M1_PIN 17    // M1引脚
#define LORA_SERIAL_RX 18 // 串口RX引脚
#define LORA_SERIAL_TX 19 // 串口TX引脚
void setup()
{
    Serial.begin(9600);

    loraAt.begin(&Serial2, LORA_MO_PIN, LORA_M1_PIN, -1, -1, LORA_SERIAL_RX, LORA_SERIAL_TX);
    loraAt.registerCallback(recvCallback);
}

uint32_t lasttime = 0;
void loop()
{
    if (loraAt.isRun() && time(0) - lasttime > 5)
    {
        lasttime = time(0);
        uint8_t data[] = "HelloWorld";
        loraAt.sendData(data, sizeof(data));
    }
}

void recvCallback(uint8_t *data, uint16_t len)
{
    Serial.print("Recv Data: ");
    loraAt.hex_print(data, len);
}
```
