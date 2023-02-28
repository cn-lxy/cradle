#include <Arduino.h>
#include <SparkFunMLX90614.h>
#include <U8g2lib.h>
#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif
#include "PubSubClient.h"
#include "WiFi.h"
#include <aliyun_mqtt.h>
#include <ArduinoJson.h>

/*------------------------------------- WiFi & MQTT ----------------------------------------*/
#define WIFI_SSID "DESKTOP-LXY"                         //wifi名
#define WIFI_PASSWD "12345678"                          //wifi密码
/*------------------------------------------------------------------------------------------*/

/*------------------------------------------ GPIO ------------------------------------------*/
// GPIO of weight sensor [重力传感器]
#define HX711_SCK_PIN  13   // SCK 输出口 ---输出脉冲
#define HX711_DT_PIN   12   // DT 输入口  ---读取数据

// GPIO of alert [蜂鸣器]
#define BUZZER_PIN      4   

// GPIO OLED SDA SCL [显示屏]
#define OLED_SCL_PIN   5
#define OLED_SDA_PIN   18

// GPIO Board LED [板载LED !不用画!]
#define LED_PIN        2

// GPIO Raindrop [雨滴传感器]
#define RAINDROP_PIN   14

// GPIO humidifier [加湿器] 
#define HUMIDIFIER_PIN 27

// GPIO electric blanket [电热毯]
#define ELECTRIC_BLANKET_PIN 26
// GPIO Bottle Heat [奶瓶加热]
#define BOTTLE_HEAT_PIN 25

/*------------------------------------------------------------------------------------------*/

#define WEIGHT_THRESHOLD_VALUE 			 100   // 重量低于该值开启报警
#define HUMIDIFIER_THRESHOLD_VALUE 		 80.0  // 环境湿度低于该值开启加湿器
#define ELECTRON_BLANKET_THRESHOLD_VALUE 25.0  // 环境温度低于该值开启电热毯

/*------------------------------------- 云平台消息相关 ---------------------------------------*/
#define PRODUCT_KEY "a1HqBPF6ttD"                         //产品ID
#define DEVICE_NAME "ESP32"                               //设备名
#define DEVICE_SECRET "37242dd6595dd7e1e5e174d3ae9189e0"  //设备key
#define REGION_ID "cn-shanghai"
// 服务端消息订阅Topic
#define ALINK_TOPIC_PROP_SET "/a1HqBPF6ttD/" DEVICE_NAME "/user/get"
// 属性上报Topic
#define ALINK_TOPIC_PROP_POST "/sys/" PRODUCT_KEY "/" DEVICE_NAME "/thing/event/property/post"
// 设备 `POST` 上传数据要用到一个json字符串, 这个是拼接postJson用到的一个字符串
#define ALINK_METHOD_PROP_POST "thing.event.property.post"
// 这是 `POST` 上传数据使用的模板
#define ALINK_BODY_FORMAT "{\"id\":\"%u\",\"version\":\"1.0.0\",\"method\":\"%s\",\"params\":%s}"
/*------------------------------------------------------------------------------------------*/

/*----------------------------------------- 全局变量 ----------------------------------------*/
// 重力传感器
volatile long weightInit = 0;   // 传感器初始值
#define GAP_VALUE 405    		// 该值需校准 每个传感器都有所不同
// SHTC3
#define SHTC3_ADDRESS 0x70      // 定义SHTC3的I2C器件地址为0x70
// mqtt
unsigned int postMsgId = 0;     // 消息发布计数[Aliyun]
// 数据结构体
typedef struct {
	volatile long   weightTrue;  	  // 真实重量 = 传感器返回重量 - 传感器初始值  
	volatile double bodyTemperature;  // 体温
	volatile double envTemperature;   // 环境温度
	volatile double envHumidity;      // 环境湿度
	volatile int    rainDrop;		  // 雨滴【尿床】
} Data;
Data data;

// 蜂鸣器模式
typedef struct {
	volatile int mode;
} BuzzerMode;
BuzzerMode buzzerMode;

// 奶瓶加热
typedef struct {
	volatile bool bottleHeat = false;
} AliData;
AliData aliData; 

// mutex: senor data
SemaphoreHandle_t xMutexData = NULL;
// mutex: buzzer
SemaphoreHandle_t xMutexBuzzer = NULL;

TickType_t timeout = 1000;

/*----------------------------------------------------------------------------------------*/

/*----------------------------------------- 全局对象 ---------------------------------------*/
// oled显示类构造函数
U8G2_SH1106_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0,  /*SCL*/ OLED_SCL_PIN,  /*SDA*/ OLED_SDA_PIN, /*reset*/ U8X8_PIN_NONE);  
WiFiClient espClient;               // 创建网络连接客户端
PubSubClient mqttClient(espClient); // 通过网络客户端连接创建mqtt连接客户端
IRTherm therm;  					// 红外MLX90614操作对象
/*----------------------------------------------------------------------------------------*/

/*---------------------------------------- 任务声明 -------------------------------------*/
void setupTask(void *ptParams);       // [task]总程序初始化
void sensorGetTask(void *ptParams);	  // [task]获取传感器
void sensorLogTask(void *ptParams);   // [task]传感器数据打印
void displayTask(void *ptParams);	  // [task]OLED显示
void mqttCheck(void *ptParams);		  // [task]mqtt连接检查
void controllerTask(void *ptParams);  // [task]外设控制
void sendMsgTask(void *ptParams);     // [task]向aliyun发送消息
void buzzerTask(void *ptParams);      // [task]蜂鸣器控制
/*--------------------------------------------------------------------------------------*/

/*------------------------------------ 函数声明 -------------------------------------------*/
void pinSetup(void);												// [ESP32]  初始化GPIO
void WifiSetup(void); 											    // [wifi] 	wifi连接
void mqttCallback(char *topic, byte *payload, unsigned int length); // [mqtt] 	收到消息回调
void clientReconnect(void); 									    // [mqtt] 	mqtt客户端重连函数
void mqttCheck(void);  											    // [mqtt] 	MQTT网络检测
void readMLX(void);   											    // [红外] 	 读取mlx温度
uint8_t SHTC3_CRC_CHECK(uint16_t DAT,uint8_t CRC_DAT);  		    // [温湿度]  SHTC3的CRC校验
void readSHTC3(void);     										    // [温湿度]  获取温湿度数据
unsigned long readHX711(void);  								    // [重力] 	 读取重力传感器数据
void getWeight(void); 											    // [重力] 	 获取物体真实重量
void readRainDrop(void);										    // [雨滴]	 读取雨滴传感器数据
void humidifierHandler(void);									    // [加湿器]  控制加湿器 
void buzzerHandler(void);											// [报警]	 蜂鸣器模式控制
void electricBlanketHandler(void);									// [电热毯]  根据温度控制电热毯加热
void bottleHeatingHandler(void);									// [奶瓶]    由微信小程序控制开启或给关闭奶瓶加热
void initDisplayFun(const char* info);								// [显示]	 在初始化阶段显示初始化信息
/*---------------------------------------------------------------------------------------*/
