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
#include "buzzer.h"
#include "main.h"

//! setup
void setup()
{
	xTaskCreate(setupTask, "setup task", 1024 * 4, NULL, 1, NULL);
}

//! loop
void loop() {}

// 👋
void setupTask(void *ptParams)
{
	initDisplayFun("systems initialize...");
	Serial.begin(115200); // 设置串口波特率
	pinSetup();
	u8g2.begin();
	therm.begin();
	therm.setUnit(TEMP_C);
	Wire.begin(); // 初始化为I2C主机 SHTC3

	weightInit = readHX711(); // 获取开机时的重力传感器数据
	// 打印初始化的 重量值：weightInit
	Serial.printf("weightInit Date: %ld\n", weightInit);
	Serial.flush();

	initDisplayFun("Network initialize...");
	WifiSetup();
	if (connectAliyunMQTT(mqttClient, PRODUCT_KEY, DEVICE_NAME, DEVICE_SECRET))
	{
		Serial.println("MQTT服务器连接成功!");
	}
	mqttClient.subscribe(ALINK_TOPIC_PROP_SET); // ! 订阅Topic !!这是关键!!
	mqttClient.setCallback(mqttCallback);		// 绑定收到set主题时的回调(命令下发1回调)

	initDisplayFun("system initialize...");
	// 任务初始化: 都在 `core1` 上创建任务
	xMutexData = xSemaphoreCreateMutex(); // 创建Mutex
	if (xMutexData == NULL)
	{
		Serial.println("No Enough RAM, unable to create `Semaphore.`");
	}
	xMutexBuzzer = xSemaphoreCreateMutex(); // 创建 buzzer mutex
	if (xMutexBuzzer == NULL)
	{
		Serial.println("No Enough RAM, unable to create `Semaphore.`");
	}

	if (xTaskCreatePinnedToCore(sensorGetTask, "sensorGetTask", 1024 * 5, NULL, 1, NULL, 1) == pdPASS)
		Serial.println("sensorGetTask 创建成功");
	if (xTaskCreatePinnedToCore(sensorLogTask, "sensorLogTask", 1024 * 5, NULL, 1, NULL, 1) == pdPASS)
		Serial.println("sensorLogTask 创建成功");
	if (xTaskCreatePinnedToCore(displayTask, "displayTask", 1024 * 5, NULL, 1, NULL, 1) == pdPASS)
		Serial.println("displayTask 创建成功");
	if (xTaskCreatePinnedToCore(mqttCheck, "mqttCheck", 1024 * 5, NULL, 1, NULL, 1) == pdPASS)
		Serial.println("mqttCheck 创建成功");
	if (xTaskCreatePinnedToCore(controllerTask, "controllerTask", 1024 * 5, NULL, 1, NULL, 1) == pdPASS)
		Serial.println("controllerTask 创建成功");

	if (xTaskCreatePinnedToCore(sendMsgTask, "sendMsgTask", 1024 * 5, NULL, 1, NULL, 1) == pdPASS)
		Serial.println("sendMsgTask 创建成功");

	if (xTaskCreatePinnedToCore(buzzerTask, "buzzer music", 1024 * 4, NULL, 1, NULL, 1) == pdPASS)
		Serial.println("buzzerLoop 创建成功");

	// BUG[定时器中有`sprintf`操作就会导致重启，使用task替换该功能的实现] 创建定时器
	// sendMsgTimerHandle = xTimerCreate("sendMsg timer", 2000, pdTRUE, (void *)1, sendMsgTimerCallback);
	// xTimerStart(sendMsgTimerHandle, portMAX_DELAY);

	initDisplayFun("initialize over!");
	vTaskDelay(pdMS_TO_TICKS(1000));
	vTaskDelete(NULL);
}

void sensorGetTask(void *ptParams)
{
	const TickType_t xFrequency = 100; // 采集频率 100 ticks = 100ms
	TickType_t lastSleepTick = xTaskGetTickCount();
	while (true)
	{
		// 拿取钥匙
		if (xSemaphoreTake(xMutexData, timeout) == pdPASS)
		{
			readSHTC3();
			getWeight();
			readRainDrop();
			readMLX();
			// LOG
			// Serial.println("get sensor data.");
			// 释放锁
			xSemaphoreGive(xMutexData);
		}
		vTaskDelayUntil(&lastSleepTick, xFrequency);
	}
}

// 传感器数据打印task
void sensorLogTask(void *ptParams)
{
	const TickType_t xFrequency = 1000;
	TickType_t lastSleepTick = xTaskGetTickCount();
	while (true)
	{
		if (xSemaphoreTake(xMutexData, timeout) == pdPASS)
		{
			Serial.printf("BodyTemperature: %.2f°\n", data.bodyTemperature);
			Serial.printf("Humidity: %.2f%%, Temperature: %.2f\n", data.envTemperature, data.envHumidity);
			Serial.printf("Weight: %ldg\n", data.weightTrue);
			Serial.printf("RainDrop: %d\n", data.rainDrop);
			// 释放锁
			xSemaphoreGive(xMutexData);
		}
		vTaskDelayUntil(&lastSleepTick, xFrequency);
	}
}

// OLED显示task
void displayTask(void *ptParams)
{
	while (true)
	{
		u8g2.clearBuffer();
		u8g2.setFont(u8g2_font_ncenB08_tr);
		u8g2.setColorIndex(2);
		char info1[32];
		char info2[32];
		char info3[32];
		char info4[32];
		char info5[32];
		if (xSemaphoreTake(xMutexData, timeout) == pdPASS)
		{
			sprintf(info1, "Weight: %ldg", data.weightTrue);
			sprintf(info2, "EnvTemp: %.2f", data.envTemperature);
			sprintf(info3, "EnvHum: %.2f%%", data.envHumidity);
			sprintf(info4, "BodyTemp: %.2f", data.bodyTemperature);
			sprintf(info5, "RainDrop: %d", data.rainDrop);

			xSemaphoreGive(xMutexData);
		}
		else
		{
			continue;
		}
		u8g2.drawStr(0, 10, info1);
		u8g2.drawStr(0, 20, info2);
		u8g2.drawStr(0, 30, info3);
		u8g2.drawStr(0, 40, info4);
		u8g2.drawStr(0, 50, info5);
		u8g2.sendBuffer();
	}
}

// mqtt连接检查task，1s检查一次
void mqttCheck(void *ptParams)
{
	const TickType_t xFrequency = 1000; // 1000 ticks = 1000ms
	TickType_t xLastSleepTick = xTaskGetTickCount();
	while (true)
	{
		mqttCheck();	   // 检查mqtt连接
		mqttClient.loop(); // mqtt客户端监听，不会阻塞
		vTaskDelayUntil(&xLastSleepTick, xFrequency);
	}
}

// 硬件控制 [报警,加湿器,尿湿音乐,电热毯,奶瓶加热]
void controllerTask(void *ptParams)
{
	while (true)
	{
		if (xSemaphoreTake(xMutexData, timeout) == pdPASS)
		{
			buzzerHandler();		  // 蜂鸣器
			humidifierHandler();	  // 加湿器
			electricBlanketHandler(); // 电热毯
			bottleHeatingHandler();	  // 奶瓶加热
			xSemaphoreGive(xMutexData);
		}
	}
}

void sendMsgTask(void *ptParams)
{
	const TickType_t xFrequency = 5000;
	TickType_t lastSleepTick = xTaskGetTickCount();
	while (true)
	{
		if (mqttClient.connected())
		{
			// 先拼接出json字符串
			char param[128] = {0};
			char jsonBuf[256] = {0};
			if (xSemaphoreTake(xMutexData, timeout) == pdPASS)
			{
				// 把要上传的数据写在param中
				sprintf(param,
						"{\"BodyTemperature\":%.2f,\"EnvTemperature\":%.2f,\"EnvHumidity\":%.2f,\"RainDrop\":%d}",
						data.bodyTemperature, data.envTemperature, data.envHumidity, data.rainDrop);

				postMsgId += 1;
				sprintf(jsonBuf, ALINK_BODY_FORMAT, postMsgId, ALINK_METHOD_PROP_POST, param);
				xSemaphoreGive(xMutexData);

				// 再从mqtt客户端中发布post消息
				if (mqttClient.publish(ALINK_TOPIC_PROP_POST, jsonBuf))
				{
					Serial.print("Post message to cloud: ");
					Serial.println(jsonBuf);
					// 发送成功板载LED闪烁
					digitalWrite(LED_PIN, HIGH);
					vTaskDelay(pdMS_TO_TICKS(50)); // 50ms
					digitalWrite(LED_PIN, LOW);
				}
				else
				{
					Serial.println("Publish message to cloud failed!");
				}
			}
		}
		vTaskDelayUntil(&lastSleepTick, xFrequency);
	}
}

// FIXED 【修复】多状态蜂鸣器控制
void buzzerTask(void *ptParams)
{
	int channel = 0;	// 通道
	int freq = 2;		// 频率
	int resolution = 8; // 分辨率
	ledcSetup(channel, freq, resolution);
	ledcAttachPin(BUZZER_PIN, channel);

	int length = sizeof(tune) / sizeof(tune[0]); // 计算长度
	int xMode = 0;

	while (true)
	{
		if (xSemaphoreTake(xMutexBuzzer, timeout) == pdPASS)
		{
			xMode = buzzerMode.mode;
			xSemaphoreGive(xMutexBuzzer);
		}
		switch (xMode)
		{
		case 0:
			// Serial.println("buzzer: none");  // log
			ledcWrite(channel, 0);
			break;
		case 1:
			// Serial.println("buzzer: alert"); // log
			ledcWrite(channel, 255);
			vTaskDelay(pdMS_TO_TICKS(1000));
			break;
		case 2:
			// Serial.println("buzzer: music");  // log
			for (int x = 0; x < length; x++)
			{
				if (xSemaphoreTake(xMutexBuzzer, timeout) == pdPASS)
				{
					xMode = buzzerMode.mode;
					xSemaphoreGive(xMutexBuzzer);
					if (xMode != 2)
						break;
				}
				ledcWriteTone(0, tune[x]);
				vTaskDelay(500 * durt[x]); // 这里用来根据节拍调节延时，500这个指数可以自己调整，在该音乐中，我发现用500比较合适。
			}
			// ledcWriteTone(0, 0);
			vTaskDelay(pdMS_TO_TICKS(500));
			break;
		default:
			break;
		}
	}
}

// 初始化GPIO
void pinSetup()
{
	pinMode(HX711_SCK_PIN, OUTPUT); // HX711重力传感器 GPIO初始化
	pinMode(HX711_DT_PIN, INPUT);
	pinMode(BUZZER_PIN, OUTPUT);  // 报警GPIO初始化
	pinMode(LED_PIN, OUTPUT);	  // 板载LED初始化
	pinMode(RAINDROP_PIN, INPUT); // 雨滴传感器GPIO初始化
	pinMode(HUMIDIFIER_PIN, OUTPUT);
	pinMode(ELECTRIC_BLANKET_PIN, OUTPUT);
	pinMode(BOTTLE_HEAT_PIN, OUTPUT);
}

/**
 * @brief wifi连接
 */
void WifiSetup()
{
	delay(10);
	Serial.println("连接WIFI");
	WiFi.begin(WIFI_SSID, WIFI_PASSWD);
	while (!WiFi.isConnected())
	{
		Serial.print(".");
		// vTaskDelay(pdMS_TO_TICKS(500));  // 500ms
		delay(500); // 500ms
	}
	Serial.println("OK");
	Serial.println("Wifi连接成功");
}

/**
 * @brief 收到消息回调
 * @param topic 订阅的topic
 * @param payload 接受的消息负载
 * @param length 消息长度
 */
void mqttCallback(char *topic, byte *payload, unsigned int length)
{
	if (strstr(topic, ALINK_TOPIC_PROP_SET))
	{
		Serial.println("收到下发的命令主题:");
		Serial.println(topic);
		Serial.println("下发的内容是:");
		payload[length] = '\0'; // 为payload添加一个结束附,防止Serial.println()读过了
		Serial.println((char *)payload);

		// 接下来是收到的json字符串的解析
		DynamicJsonDocument doc(150);
		DeserializationError error = deserializeJson(doc, payload);
		if (error)
		{
			Serial.println("parse json failed");
			Serial.println(error.c_str());
			return;
		}
		JsonObject setAlinkMsgObj = doc.as<JsonObject>();
		serializeJsonPretty(setAlinkMsgObj, Serial);
		Serial.println();

		// 回调数据处理
		aliData.bottleHeatStatus = setAlinkMsgObj["params"]["bottleHeat"];		// { "name": "bottleHeat", "type": "bool" }
		aliData.alertStatus = setAlinkMsgObj["params"]["alert"];				// { "name": "alert", "type": "bool" }
		aliData.electricBlankStatus = setAlinkMsgObj["params"]["electriBlank"]; // { "name": "electriBlank", "type": "bool" }
	}
}

/**
 * @brief mqtt客户端重连函数, 如果客户端断线,可以通过此函数重连
 */
void clientReconnect()
{
	while (!mqttClient.connected())
	{ // 再重连客户端
		Serial.println("reconnect MQTT...");
		if (connectAliyunMQTT(mqttClient, PRODUCT_KEY, DEVICE_NAME, DEVICE_SECRET))
		{
			Serial.println("connected");
		}
		else
		{
			Serial.println("failed");
			Serial.println(mqttClient.state());
			Serial.println("try again in 5 sec");
			delay(5000);
		}
	}
}

/**
 * @brief MQTT网络检测
 */
void mqttCheck()
{
	if (!WiFi.isConnected())
	{ // 判断WiFi是否连接
		WifiSetup();
	}
	else
	{ // 如果WIFI连接了,
		if (!mqttClient.connected())
		{ // 再次判断mqtt是否连接成功
			Serial.println("mqtt disconnected!Try reconnect now...");
			Serial.println(mqttClient.state());
			clientReconnect();
		}
	}
}

/**
 * @brief 读取mlx温度
 * @version 2.0.0
 * @date 2022.6.9
 * @author lxy@qq.com
 */
void readMLX()
{
	if (therm.read())
	{ // On success, read() will return 1, on fail 0.
		String s = String(therm.object(), 2);
		data.bodyTemperature = atoi(s.c_str());
	}
}

/**
 * @brief SHTC3的CRC校验
 * @param DAT
 * @param CRC_DAT
 * @return uint8_t
 */
uint8_t SHTC3_CRC_CHECK(uint16_t DAT, uint8_t CRC_DAT)
{
	uint8_t i, t, temp;
	uint8_t CRC_BYTE;
	CRC_BYTE = 0xFF;
	temp = (DAT >> 8) & 0xFF;
	for (t = 0; t < 2; t++)
	{
		CRC_BYTE ^= temp;
		for (i = 0; i < 8; i++)
		{
			if (CRC_BYTE & 0x80)
			{
				CRC_BYTE <<= 1;
				CRC_BYTE ^= 0x31;
			}
			else
			{
				CRC_BYTE <<= 1;
			}
		}
		if (t == 0)
		{
			temp = DAT & 0xFF;
		}
	}
	if (CRC_BYTE == CRC_DAT)
	{
		temp = 1;
	}
	else
	{
		temp = 0;
	}
	return temp;
}

/**
 * @brief 获取温湿度数据
 */
void readSHTC3()
{
	Wire.beginTransmission(SHTC3_ADDRESS); // 根据地址0x70，开始向I2C的从机进行传输。
	Wire.write(byte(0xE0));				   // 发送写入指令
	Wire.endTransmission();				   // 停止向从机传输
	Wire.beginTransmission(SHTC3_ADDRESS);
	Wire.write(byte(0x35)); // 发送唤醒指令的高位部分
	Wire.write(byte(0x17)); // 发送唤醒指令的低位部分
	Wire.endTransmission();
	delayMicroseconds(300); // 延时300微秒
	Wire.beginTransmission(SHTC3_ADDRESS);
	Wire.write(byte(0xE0));
	Wire.endTransmission();
	Wire.beginTransmission(SHTC3_ADDRESS);
	Wire.write(byte(0x7C)); // 发送采集指令的高位部分
	Wire.write(byte(0xA2)); // 发送采集指令的低位部分
	Wire.endTransmission();
	Wire.beginTransmission(SHTC3_ADDRESS);
	Wire.write(byte(0xE1)); // 发送读取指令
	Wire.endTransmission();
	Wire.requestFrom(SHTC3_ADDRESS, 6); // 向从机请求数据
	uint16_t T_temp, RH_temp, T_CRC, RH_CRC;
	if (2 <= Wire.available())
	{
		T_temp = Wire.read();  // 接收温度高位数据
		T_temp = T_temp << 8;  // 左移8位
		T_temp |= Wire.read(); // 左移8位后的温度高位数据与接收到的温度低位数据进行按位或运算
		T_CRC = Wire.read();   // 接收CRC校验码
		if (SHTC3_CRC_CHECK(T_temp, T_CRC))
		{															// 校验数据
			data.envTemperature = float(T_temp) * 175 / 65536 - 45; // 计算出温度
		}
	}
	if (2 <= Wire.available())
	{
		RH_temp = Wire.read();	// 接收湿度高位数据
		RH_temp = RH_temp << 8; // 左移8位
		RH_temp |= Wire.read(); // 左移8位后的湿度高位数据与接收到的湿度低位数据进行按位或运算
		RH_CRC = Wire.read();
		if (SHTC3_CRC_CHECK(RH_temp, RH_CRC))
		{
			data.envHumidity = float(RH_temp) * 100 / 65536;
		}
	}
}

/**
 * @brief 读取重力传感器数据 => 选择芯片工作方式并进行数据读取
 * @author lxy
 * @return 重力传感器原始数据
 */
unsigned long readHX711(void)
{
	unsigned long count = 0; // 储存输出值
	unsigned char i;

	digitalWrite(HX711_DT_PIN, HIGH);
	delayMicroseconds(1); // 延时 1微秒
	digitalWrite(HX711_SCK_PIN, LOW);
	delayMicroseconds(1); // 延时 1微秒

	// 当DT的值为1时，开始ad转换
	while (digitalRead(HX711_DT_PIN))
		;
	// 24个脉冲，对应读取24位数值
	for (i = 0; i < 24; i++)
	{
		// 利用 SCK从0--1 ，发送一次脉冲，读取数值
		digitalWrite(HX711_SCK_PIN, HIGH);

		delayMicroseconds(1);			  // 延时 1微秒
		count = count << 1;				  // 用于移位存储24位二进制数值
		digitalWrite(HX711_SCK_PIN, LOW); // 为下次脉冲做准备
		delayMicroseconds(1);
		if (digitalRead(HX711_DT_PIN)) // 若DT值为1，对应count输出值也为1
			count++;
	}
	digitalWrite(HX711_SCK_PIN, HIGH); // 再来一次上升沿 选择工作方式  128增益

	// 按位异或,不同则为1 => 0^0=0; 1^0=1;
	// 对应二进制 => [1000 0000 0000 0000 0000 0000]; 作用为将最高位取反,其他位保留原值
	count ^= 0x800000;
	delayMicroseconds(1);

	digitalWrite(HX711_SCK_PIN, LOW); // SCK=0；
	delayMicroseconds(1);
	// 返回传感器读取值
	return count;
}

/**
 * @brief 获取物体真实重量
 * @author lxy
 */
void getWeight()
{
	long HX711_Buffer;
	HX711_Buffer = readHX711();									  // 读取传感器输出值
	data.weightTrue = HX711_Buffer;								  // 将传感器的输出值储存
	data.weightTrue -= weightInit;								  // 获取实物的AD采样数值。
	data.weightTrue = (long)((float)data.weightTrue / GAP_VALUE); // AD值转换为重量（g）
																  // Serial.printf("data weight: %d", data.weightTrue);
}

/**
 * @brief 读取雨滴传感器数据
 */
void readRainDrop(void)
{
	data.rainDrop = digitalRead(RAINDROP_PIN);
}

// 加湿器handler
void humidifierHandler(void)
{
	if (data.envHumidity < HUMIDIFIER_THRESHOLD_VALUE)
	{
		digitalWrite(HUMIDIFIER_PIN, HIGH);
	}
	else
	{
		digitalWrite(HUMIDIFIER_PIN, LOW);
	}
}

/**
 * @brief 报警handler
 */
void buzzerHandler(void)
{
	if (xSemaphoreTake(xMutexBuzzer, timeout) == pdPASS)
	{
		// 报警: 在微信小程序报警是关的状态下，重量低于预设值
		if (data.weightTrue < WEIGHT_THRESHOLD_VALUE && !aliData.alertStatus)
		{
			buzzerMode.mode = 1;
			xSemaphoreGive(xMutexBuzzer);
			return;
		}
		else if (data.rainDrop)
		{
			// buzzer music
			buzzerMode.mode = 2;
			xSemaphoreGive(xMutexBuzzer);
			return;
		}
		else
		{
			buzzerMode.mode = 0;
			xSemaphoreGive(xMutexBuzzer);
			return;
		}
	}
}

/**
 * @brief 电热毯加热handler
 */
void electricBlanketHandler(void)
{
	// @deprecated 根据环境温度，ESP32自动控制电热毯
	// if (data.envTemperature < ELECTRON_BLANKET_THRESHOLD_VALUE)
	// {
	// 	digitalWrite(ELECTRIC_BLANKET_PIN, HIGH);
	// }
	// else
	// {
	// 	digitalWrite(ELECTRIC_BLANKET_PIN, LOW);
	// }

	// 微信小程序控制电热毯
	if (aliData.electricBlankStatus)
	{
		digitalWrite(ELECTRIC_BLANKET_PIN, HIGH);
	}
	else
	{
		digitalWrite(ELECTRIC_BLANKET_PIN, LOW);
	}
}

/**
 * @brief [奶瓶] 由微信小程序控制开启或给关闭奶瓶加热
 */
void bottleHeatingHandler(void)
{
	if (aliData.bottleHeatStatus)
	{
		digitalWrite(BOTTLE_HEAT_PIN, HIGH);
	}
	else
	{
		digitalWrite(BOTTLE_HEAT_PIN, LOW);
	}
}

// 初始化显示信息
void initDisplayFun(const char *info)
{
	u8g2.clearBuffer();
	u8g2.setFont(u8g2_font_ncenB08_tr);
	u8g2.setColorIndex(2);
	char buffer[32];
	sprintf(buffer, "%s", info);
	u8g2.drawStr(0, 10, buffer);
	u8g2.sendBuffer();
}