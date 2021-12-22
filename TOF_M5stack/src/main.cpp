#include <M5Stack.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <Ticker.h>
#include <Wire.h>
#include <VL53L0X.h>
#include "string.h"

#define BUTTON          0                                    // (Don't Change for Sonoff) 定义按钮的 gpio
#define LED_GREEN       13                                   // (Don't Change for Sonoff) 定义LED灯的GPIO
#define LED_RED         12
#define LED_BLUE        14
/******************************************* 定义 VL53L0X XSHUT接口  **************************************************/

#define  XSHUT1_PIN 16  // 定义sensor1 的 XSHUT 接口
#define  XSHUT2_PIN 17  // 定义sensor2 的 XSHUT 接口

#define VERSION    "\n\n------------------  VL53L0X v1.00  ------------------"

// 取消注释可输出传感器串口信息
#define PRINT_SERIAL_DEBUG

// Uncomment this line to use long range mode. This
// increases the sensitivity of the sensor and extends its
// potential range, but increases the likelihood of getting
// an inaccurate reading because of reflections from objects
// other than the intended target. It works best in dark
// conditions.

// 取消注释可以增强检测距离，默认1.8米
// #define LONG_RANGE

// Uncomment ONE of these two lines to get
// - higher speed at the cost of lower accuracy OR
// - higher accuracy at the cost of lower speed

// 取消注释可以开启高速模式
#define HIGH_SPEED
// 取消注释可以开启精准模式
 //#define HIGH_ACCURACY

const char* wifi1_ssid = "KingHone_2.4";
const char* wifi1_passwd = "JayKing0912";
const char* wifi2_ssid = "king";
const char* wifi2_passwd = "JayKing0912";
const char* mqtt_user = "mqtt";
const char* mqtt_password = "1qaz2wsx";
int mqtt_port = 1883;
// 使用HIVEMQ 的信息中转服务
const char* mqtt_server = "10.0.0.142";
// 订阅信息主题
const char* TOPIC = "VL53L0X/zhuwei_vl53l0x";
const char* SUB_TOPIC = "VL53L0X/zhuwei_vl53l0x/cmd";
char* Topic_sensor1 = new char[100]();
char* Topic_sensor2 = new char[100]();
char* client_id =  new char[100]();
bool requestRestart = false;                                 // (Do not Change) 控制是否重启
unsigned long run_time;


/******************************************* 定义 传感器变量 **************************************************/
VL53L0X sensor1;                                         // 初始化传感器库
VL53L0X sensor2;                                         // 初始化传感器库
uint16_t last_dist_1;                                            // 初始化距离变量
uint16_t last_dist_2;                                            // 初始化距离变量

// 主循环延迟时间
int delay_time = 0;

// 距离产生多大的变化才发送消息
int change_num = 50;
unsigned long count = 0;                                     // (Do not Change) 用于长按计数

// extern "C" {                                                // 告诉编译器，大括号里面的代码使用C语言编译
//   #include "user_interface.h"                               // 引入 Arduino 网络库 user_interface 应该是 WiFiClient 需要
// }

WiFiClient wifiClient;                                        // 初始化 WiFiClient 对象
PubSubClient client(wifiClient);  // 初始化 PubSubClient 对象
Ticker btn_timer;                                             // 初始化 Ticker 对象

void callback(char* topic, byte* payload, unsigned int length) {                     // 监听mqtt指令
  Serial.println ("callback ...");
  String mqtt_message;
  for (int i=0;i<(int)length;i++){
    Serial.println((char)payload[i]);
    mqtt_message += (char)payload[i];
  }
  Serial.print("mqtt_message:");
  Serial.println(mqtt_message);
  if (mqtt_message.equals(String("reset"))) {
    Serial.println ("Reset ...");
    requestRestart = true;
  }
}

bool sensor_online_chack() {
    int count = 0;
    Serial.println ("I2C scanner. Scanning ...");
    for (byte i = 1; i < 120; i++)
    {
        Wire.beginTransmission (i);
        if (Wire.endTransmission () == 0)
        {
            Serial.print ("Found address: ");
            Serial.print (i, DEC);
            Serial.print (" (0x");
            Serial.print (i, HEX);
            Serial.println (")");
            count++;
            delay (1);
        }
    }
    Serial.print (count);
    if ((count = 2))
    {
        return true;
    }
    else 
    {
        return false;
    }
}

void sensor_setup() {
    pinMode(XSHUT2_PIN, OUTPUT);
    pinMode(XSHUT1_PIN, OUTPUT);
    digitalWrite(XSHUT1_PIN, LOW);
    digitalWrite(XSHUT2_PIN, LOW);
    delay(500);
    Wire.begin();
    pinMode(XSHUT1_PIN, INPUT);
    delay(150);
    Serial.print("sensor_1 initing.....");
    sensor1.init(true);
    delay(100);
    sensor1.setAddress((uint8_t)22);
    Serial.println("done.");
    pinMode(XSHUT2_PIN, INPUT);
    delay(150);
    sensor2.init(true);
    Serial.print("sensor_2 initing.....");
    delay(100);
    sensor2.setAddress((uint8_t)25);
    Serial.println("done.");

    sensor1.setTimeout(500);
    sensor2.setTimeout(500);

    #if defined LONG_RANGE
      // lower the return signal rate limit (default is 0.25 MCPS)
      sensor1.setSignalRateLimit(0.3);
      // increase laser pulse periods (defaults are 14 and 10 PCLKs)
      sensor1.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
      sensor1.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
      // lower the return signal rate limit (default is 0.25 MCPS)
      sensor2.setSignalRateLimit(0.3);
      // increase laser pulse periods (defaults are 14 and 10 PCLKs)
      sensor2.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
      sensor2.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    #endif

    #if defined HIGH_SPEED
      // reduce timing budget to 20 ms (default is about 33 ms)
      // 如果传感器数值跳的误触厉害，可以适当的加大这个数值
      sensor1.setMeasurementTimingBudget(20000);
      sensor2.setMeasurementTimingBudget(20000);
    #elif defined HIGH_ACCURACY
      // increase timing budget to 200 ms
      sensor1.setMeasurementTimingBudget(30000);
      sensor2.setMeasurementTimingBudget(30000);
    #endif
    sensor1.startContinuous();
    sensor2.startContinuous();
}

void blinkLED(int pin, int duration, int n) {             
  for(int i=0; i<n; i++)  {  
    digitalWrite(pin, LOW);        
    delay(duration);
    digitalWrite(pin, HIGH);
    delay(duration);
  }
}

void button() {
  if (!digitalRead(BUTTON)) {
    count++;
  }
  else {
    if (count > 1 && count <= 40) {   
      digitalWrite(LED_RED, !digitalRead(LED_RED));
    } 
    else if (count >40){
      Serial.println("\n\nSonoff Rebooting . . . . . . . . Please Wait"); 
      requestRestart = true;
    } 
    count=0;
  }
}

void checkConnection() { // 如果wifi或者mqtt断开,就重启开发板
  if (WiFi.status() == WL_CONNECTED)  {
  }
  else { 
    Serial.println("WiFi connection . . . . . . . . . . LOST");
    requestRestart = true;
  }
}

void mqtt_reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(client_id,mqtt_user,mqtt_password)) {
      Serial.println("connected");
      // 连接成功时订阅主题
      client.subscribe(SUB_TOPIC);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void get_dist(){
  uint16_t dist_1 = sensor1.readRangeContinuousMillimeters();
  uint16_t dist_2 = sensor2.readRangeContinuousMillimeters();
  #if defined PRINT_SERIAL_DEBUG
    Serial.print("sensor1:");
    Serial.print(dist_1);
    Serial.print(" last_dist_1:");
    Serial.print(last_dist_1);
    Serial.print(" sensor2:");
    Serial.print(dist_2);
    Serial.print(" last_dist_2:");
    Serial.println(last_dist_2);
  #endif

  if ((dist_1 > last_dist_1 + change_num && dist_1 > 0 ) || (dist_1 < last_dist_1 - change_num && dist_1 > 0)){
    run_time = millis();
    last_dist_1 = dist_1;
    char message_buff[80];
    String pubString = "{\"dist\": " + String(last_dist_1) + ",\"time\":" + String(run_time) + "}";
    pubString.toCharArray(message_buff, pubString.length()+1);
    client.publish(Topic_sensor1, message_buff);
  }
  if ((dist_2 > last_dist_2 + change_num && dist_2  > 0) || (dist_2 < last_dist_2 - change_num && dist_2  > 0)){
    run_time = millis();
    last_dist_2 = dist_2;
    char message_buff[80];
    String pubString = "{\"dist\": " + String(last_dist_2) + ",\"time\":" + String(run_time) + "}";
    pubString.toCharArray(message_buff, pubString.length()+1);
    client.publish(Topic_sensor2, message_buff);
  }
  // 如果传感器掉线则重启
  if (dist_1 == 65535 ){
    requestRestart = true;
    char message_buff[80];
    String pubString = "{\"logger\": \"[ERROR] sensor_1 lost\"}";
    pubString.toCharArray(message_buff, pubString.length()+1);
    client.publish(Topic_sensor2, message_buff);
  }
  if (dist_2 == 65535 ){
    requestRestart = true;
    char message_buff[80];
    String pubString = "{\"logger\": \"[ERROR] sensor_2 lost\"}";
    pubString.toCharArray(message_buff, pubString.length()+1);
    client.publish(Topic_sensor2, message_buff);
  }
}

bool wifi_setup(const char *ssid,const char *passwd){
  int wifi_connect_chack_num = 10;
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, passwd);
  Serial.printf("\nConnecting to %s Wifi.",ssid);
  while (WiFi.status() != WL_CONNECTED && wifi_connect_chack_num--){
    blinkLED(LED_RED,500,1);
    Serial.print(" .");
  }
  if (WiFi.status() == WL_CONNECTED){
    Serial.println(" DONE");
    Serial.print("IP Address is: "); Serial.println(WiFi.localIP());
    blinkLED(LED_GREEN,100,3);
    return true;
  }
  else{
    Serial.println(" WiFi FAILED!");
    Serial.println("\n----------------------------------------------------------------");
    return false;
  }
}

void setup() {
  //---osmar
  M5.begin();
  M5.Lcd.setCursor(50, 0, 4);
  M5.Lcd.println(("VLX53LOX Example"));
  M5.Lcd.sleep();
  //---osmar

  // 设置 LED gpio口方向 OUTPUT
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(BUTTON, INPUT);  // 设置 BUTTON gpio口方向 INPUT
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_BLUE, HIGH);

  Serial.begin(115200);  // 初始化 Serial

  btn_timer.attach(0.05, button);   // 每 0.05 秒,执行 button 方法

  client.setServer(mqtt_server,mqtt_port);
  client.setCallback(callback);    // 设置 mqtt 返回函数

  // Serial.println(VERSION);  // 输出固件版本到串口中 println 的意思是带回车输出
  // Serial.print("\nUnit ID: ");
  // Serial.print("esp8266-");
  // Serial.println(ESP.getChipId(), HEX); // 将ESP8266芯片ID作为32位整数返回 再编成16进制

  // WIFI Connecting
  while (WiFi.status() != WL_CONNECTED){
    if (wifi_setup(wifi1_ssid,wifi1_passwd)){
      delay(500);
    }
    else if (wifi_setup(wifi2_ssid,wifi2_passwd)){
      delay(500);
    }
  }

  // OTA
  // Hostname defaults to esp8266-[ChipID]
  //ArduinoOTA.setHostname(MQTT_CLIENT);
  // Authentication
  //ArduinoOTA.setPassword((const char *)"passwd");
  ArduinoOTA.onStart([]() {   // OTA升级开始的方法
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {     // 定义OTA升级结束后方法
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) { // OTA升级时候的输出
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {    // 定义 OTA 升级出错代码
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();   // 初始化 OTA
  delay(500);
  sensor_setup();

  String Str_sensor_1 = String(TOPIC) + "/sensor1\0";
  Str_sensor_1.toCharArray(Topic_sensor1,Str_sensor_1.length()+1);
  Serial.println(Str_sensor_1);
  String Str_sensor_2 = String(TOPIC) + "/sensor2\0";
  Str_sensor_2.toCharArray(Topic_sensor2,Str_sensor_2.length()+1);
  Serial.println(Str_sensor_2);
  // String Str_clinet_id = "ESP8266-" + String(ESP.getChipId(),HEX) + "\0";
  // Str_clinet_id.toCharArray(client_id,Str_clinet_id.length()+1);
}

void loop() {
  // 检查mqtt连接
  if (!client.connected()) {
    mqtt_reconnect();
  }
  // OTA
  ArduinoOTA.handle();
  // mqtt 连接后必须函数
  client.loop();
  // 获取距离
  get_dist();
  // 重启开发板
  if (requestRestart){
    Serial.println("Restart ...");
    //ESP.reset();
    M5.Power.reset();

  }
  delay(delay_time);
}