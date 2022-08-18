#ifndef YUTOO_HEADER_LORAAT__
#define YUTOO_HEADER_LORAAT__
/*
    基于泽耀的LoRa模块 A39C0T400S22D1a/A39C0T400S22S1a/A39C0T400S30D1a/A39C0T400S30S1a AT命令实现
    单片机选用ESP32 WROOM/WOVER 实现
*/
#ifdef ESP32

#endif

// 串口命令等待时长 毫秒
#define LORA_CMD_WAIT_TIME 1000
// 连接有效性检验命令
const uint8_t LORA_CMD_VAILD[] = {0x00, 0x00, 0x01};
// 获取配置命令
const uint8_t LORA_CMD_GET_CONFIG[] = {0x00, 0x04, 0x1E};
// 获取信号强度
const uint8_t LORA_CMD_GET_RSSI[] = {0x00, 0x01, 0x01};
// 恢复默认配置命令
const uint8_t LORA_CMD_RESET_CONFIG[] = {0x80, 0x23, 0x01};
// 写入配置命令
const uint8_t LORA_CMD_WRITE_CONFIG[] = {0x80, 0x04, 0x1E};
// 自定义AES密钥 16位 默认值： 77 77 77 2E 61 73 68 69 6E 69 6E 67 2E 63 6F 6D
const uint8_t LORA_CMD_AES_KEY[] = {0x99, 0x1E, 0x22, 0x60, 0x88, 0x19, 0x22, 0x36, 0x55, 0xB3, 0x77, 0x88, 0xEE, 0x33, 0x28, 0xCD};

typedef void (*LoRaRecvCallback)(uint8_t *, uint16_t);
// 申明串口监视函数
void serialDataRecv(void *);

// LORA配置数据定义
struct LoRaConfigStrc
{
    uint8_t header[3] = {0};       // 协议头
    uint8_t baud[4] = {0};         // 串口波特率
    uint8_t uart = 0x00;           // bit5为停止位[默认为0] 0为1位 1为2位
                                   // Bit4 为帧长度(数据位+校验位)0为8位 1为9位
                                   // Bit2 | Bit1 为校验位 00 为无校验 10 为偶校验 11 为奇校验
    uint8_t RF[2] = {0};           // Bit11|Bit10 | Bit9 | Bit8 | Bit7 | Bit6 |Bit5 为信道编号，默认为 010111(23)信道
                                   // bit4 bit3为功率[默认为11] 11:21dBm  10:17dBm  01:14dBm  00:11dBm
                                   // Bit2|Bit1|Bit0 为空速[默认为010]
                                   //  111 : 62.5K          011 : 9.6K
                                   //  110 : 50K            010 : 4.8K
                                   //  101 : 38.4K          001 : 2.4K
                                   //  100 : 19.2K          000 : 1.2K
    uint8_t work_model[2] = {0};   // 工作模式 [默认为1] 0x0001,透传      0x0002,定点     0x0004, 主从    0x0020, 中继
    uint8_t normal1[3] = {0};      // 保留
    byte masterOrSlave = 0x00;     // 设备为主机还是从机 Bit0 主从模式的主机(0)还是从机(1)
    uint8_t aes_key[16] = {0};     // 16位AES密钥
    uint8_t normal2[14] = {0};     // 保留
    uint8_t waketime = 0;          // 唤醒时间，100ms 为单位
    uint8_t normal3[2] = {0};      // 保留
    uint8_t work_sel[2] = {0};     // 工作选项 Bit0 输出地址 Bit7 唤醒码
    uint8_t localGroupID = 0;      // 本地组号
    uint8_t localGroupAddr = 0;    // 本地地址
    uint8_t targetGroupID = 0;     // 目标组号
    uint8_t targetGroupAddr = 0;   // 目标地址
    uint8_t replyPathAID = 0;      // 中继模式下，路径 A 组号
    uint8_t replyPathAAddr = 0;    // 中继模式下，路径 A 地址
    uint8_t replyPathBID = 0;      // 中继模式下，路径 B 组号
    uint8_t replyPathBAddr = 0;    // 中继模式下，路径 B 地址
    uint8_t replyTempChannel = 0;  // 中继模式下跳转到的临时信道值
    uint8_t replyTempWaitTime = 0; // 中继模式临时信道停留时间
};
// LORA信号强度定义
struct LoRaRSSIStrc
{
    uint8_t header[3] = {0}; // 协议头
    uint8_t normal1 = 0;     //
    int8_t env_rssi = 0;     // 环境信号
    uint8_t normal2 = 0;     //
    int8_t data_rssi = 0;    // 数据信号
};

class LoRaAT
{
private:
    /* data */
    bool _avail_level = LOW;
    uint8_t _m0_pin;
    uint8_t _m1_pin;
    uint8_t _aux_pin;
    uint8_t _power_pin;
    HardwareSerial *_lora_serial;
    bool _run = false;
    LoRaRecvCallback _callback;
    LoRaRSSIStrc _rssi;

    // 字节数组转数字 src 原数组 原数据长度
    uint32_t s2i(uint8_t *src, uint8_t len = 4)
    {
        uint32_t i;
        uint8_t index = 0;
        while (index < len)
        {
            i += src[index] << ((len - index - 1) * 8);
            index++;
        }

        return i;
    };
    // 数字转字节串数组 src 原数据 target 目录数组 len 字节长度
    void i2s(uint32_t src, uint8_t *target, uint8_t len = 4)
    {
        uint8_t index = 0;
        while (index < len)
        {
            target[index] = (src >> ((len - index - 1) * 8)) & 0xFF;
            index++;
        }
    };

public:
    LoRaAT();
    ~LoRaAT();

    bool begin(HardwareSerial *SerialNo, uint8_t M0_pin, uint8_t M1_pin, uint8_t AUX_pin, uint8_t power_pin, uint8_t UART_RX_pin = -1, uint8_t UART_TX_pin = -1);
    // 初始化参数
    bool Init();
    // 读取串口数据
    uint16_t readSerialData(uint8_t *buffer, uint16_t max_len);
    // 获取运行状态
    bool isRun() { return _run; };
    void registerCallback(LoRaRecvCallback c) { _callback = c; };
    // 收到数据处理
    void recvData(uint8_t *buffer, uint16_t len);
    bool sendData(uint8_t *buffer, uint16_t len);
    // 设置有效电平 HIGH 高有效 LOW 低有效
    void setAvailLevel(uint8_t level) { _avail_level = level; };
    // 打印16进制数据
    void hex_print(uint8_t *buffer, uint16_t len)
    {
        for (uint16_t i = 0; i < len; i++)
        {
            Serial.printf("%02X ", buffer[i]);
            if ((i + 1) % 16 == 0)
                Serial.println();
        }
        Serial.println();
    };
};

LoRaAT::LoRaAT(/* args */)
{
}

LoRaAT::~LoRaAT()
{
}
bool LoRaAT::begin(HardwareSerial *SerialNo, uint8_t M0_pin, uint8_t M1_pin, uint8_t AUX_pin, uint8_t power_pin, uint8_t UART_RX_pin, uint8_t UART_TX_pin)
{
    //_rx = UART_RX;
    //_tx = UART_TX;
    _m0_pin = M0_pin;
    _m1_pin = M1_pin;
    _aux_pin = AUX_pin;
    _lora_serial = SerialNo;
    _power_pin = power_pin;
    if (UART_RX_pin == -1)
    {
        _lora_serial->begin(9600);
    }
    else
    {
        _lora_serial->begin(9600, SERIAL_8N1, UART_RX_pin, UART_TX_pin);
    }
    // 配置功能引脚工作模式和默认状态
    pinMode(_m0_pin, OUTPUT);
    pinMode(_m1_pin, OUTPUT);
    if (_aux_pin != -1)
        pinMode(_aux_pin, INPUT);
    if (_power_pin != -1)
    {
        pinMode(_power_pin, OUTPUT);
        digitalWrite(_power_pin, LOW);
    }
    digitalWrite(_m0_pin, _avail_level);
    digitalWrite(_m1_pin, !_avail_level);

    // 初始化参数
    if (!Init())
        return false;

    _run = true;
    // 开启数据监听线程
    TaskHandle_t serialTask; //声明一个TaskHandle_t类型的变量，用于存储将要新建的任务的句柄
    xTaskCreate(
        serialDataRecv, ///任务函数
        "serial",       //带任务名称的字符串
        10000,          //堆栈大小，单位为字节
        NULL,           //作为任务输入传递的参数
        6,              //任务的优先级
        &serialTask);   //任务句柄
    return true;
}

bool LoRaAT::Init()
{
    uint32_t workTime = millis();
    bool vaild = false;
    // 将M0设置为低 进入配置模式
    digitalWrite(_m0_pin, !_avail_level);
    Serial.println("开始获取LoRa配置信息");
    // 发送命令测试
    _lora_serial->write(LORA_CMD_GET_CONFIG, sizeof(LORA_CMD_GET_CONFIG));
    uint8_t buff[200] = {0};
    uint16_t realLen = this->readSerialData(buff, 200);
    if (realLen == 0)
    {
        digitalWrite(_m0_pin, _avail_level);
        return false;
    }
    // 还原配置数据
    uint16_t len = realLen <= sizeof(LoRaConfigStrc) ? realLen : sizeof(LoRaConfigStrc);
    LoRaConfigStrc config;
    memcpy(&config, buff, len);
    // 判断获取数据是否有效
    if (strcmp((const char *)config.header, (const char *)LORA_CMD_GET_CONFIG) != 0)
    {
        digitalWrite(_m0_pin, _avail_level);
        return false;
    }

    // 设置写入协议头
    memcpy(&config.header, LORA_CMD_WRITE_CONFIG, sizeof(LORA_CMD_WRITE_CONFIG));
    //
    uint16_t RF = this->s2i(config.RF, 2);
    // 设置信道
    // 设置空中速率 为4.8K
    RF = RF & ~0x07;
    RF |= 0x02;
    // 设置发送功率 为21dBm
    RF |= 0x18;
    this->i2s(RF, config.RF, 2);
    // 设置工作模式 为透传
    config.work_model[1] = 0x01;
    // 设置工主从模式 为主机
    config.masterOrSlave = 0x00;
    // 设置默认密钥
    memcpy(&config.aes_key, LORA_CMD_AES_KEY, sizeof(LORA_CMD_AES_KEY));
    // 开始更新配置
    memset(buff, 0, sizeof(buff));
    memcpy(buff, &config, sizeof(LoRaConfigStrc));
    Serial.println("发送的配置数据");
    this->hex_print(buff, sizeof(LoRaConfigStrc));
    _lora_serial->write(buff, sizeof(LoRaConfigStrc));
    memset(buff, 0, sizeof(buff));
    realLen = this->readSerialData(buff, 200);
    Serial.println("收到的串口数据");
    this->hex_print(buff, realLen);
    // 判断设置是否成功
    if (strcmp((const char *)buff, (const char *)LORA_CMD_WRITE_CONFIG) != 0)
    {
        digitalWrite(_m0_pin, _avail_level);
        return false;
    }

    digitalWrite(_m0_pin, _avail_level);
    return true;
}
bool LoRaAT::sendData(uint8_t *buffer, uint16_t len)
{
    if (!_run)
        return false;

    _lora_serial->write(buffer, len);

    return true;
}

uint16_t LoRaAT::readSerialData(uint8_t *buffer, uint16_t max_len)
{
    uint64_t waitTime = millis();
    while (!_lora_serial->available() && millis() - waitTime < LORA_CMD_WAIT_TIME)
    {
        delay(1);
    }
    if (!_lora_serial->available())
        return 0;

    uint16_t index = 0;
    while (_lora_serial->available() && index < max_len)
    {
        buffer[index] = _lora_serial->read();
        index++;
    }
    Serial.printf("获取串口数据成功 耗时=%dms 字节长度=", millis() - waitTime);
    Serial.println(index);
    return index;
}
// 收到数据处理
void LoRaAT::recvData(uint8_t *buffer, uint16_t len)
{
    if (len == 7)
    {
        // 长度为7位时判断下是不是信号强度
        LoRaRSSIStrc rssi;
        memcpy(&rssi, buffer, sizeof(LoRaRSSIStrc));
        if (strcmp((const char *)rssi.header, (const char *)LORA_CMD_GET_RSSI) == 0)
        {
            _rssi = rssi;
            Serial.printf("RSSI: env_rssi=%d data_rssi=%d\n", rssi.env_rssi, rssi.data_rssi);
            return;
        }
    }

    if (_callback != NULL)
        _callback(buffer, len);
};

LoRaAT loraAt;

void serialDataRecv(void *param)
{
    uint16_t realLen = 0;
    uint8_t buffer[512] = {0};
    uint32_t waitTime = 0;
    while (loraAt.isRun())
    {
        if ((realLen = loraAt.readSerialData(buffer, 512)) > 0)
        {
            loraAt.recvData(buffer, realLen);
        }

        if (time(0) - waitTime >= 5)
        {
            // 5秒钟获取一次信号
            waitTime = time(0);
            loraAt.sendData((uint8_t *)LORA_CMD_GET_RSSI, sizeof(LORA_CMD_GET_RSSI));
        }
        // vTaskDelay(1);
    }

    vTaskDelete(NULL);
}

#endif