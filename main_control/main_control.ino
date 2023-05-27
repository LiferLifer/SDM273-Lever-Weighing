/***************************************************************
 * @file       main_control
 * @brief      The main control program of the lever weighing device realized by the linear motor module.
 * @author     Author
 * @version    Version 1.1.3
 * @date       2023-05-12
 **************************************************************/

/***************************************************************
 * @brief      连线方法
 * @note       GY25-UNO: VCC-VCC // GND-GND // TX-D8 // RX-D9
 * @note       OLED-UNO: VCC-VCC // GND-GND // SCL-A5 // SDA-A4
 * @note       HPD970-UNO: PUL- -> 11 // DIR- -> 10 // EN- 可不接 // 各正极 -> 5V
 **************************************************************/

// 引入所需的库
#include <Kalman.h>
#include <Wire.h>
#include <Math.h>
#include <Stepper.h>
#include <SoftwareSerial.h>

// // 引入驱动OLED所需的库
// #include <Adafruit_GFX.h>
// #include <Adafruit_SSD1306.h>

// // 定义OLED阵列长宽
// #define SCREEN_WIDTH 128   // 设置OLED宽度,单位:像素
// #define SCREEN_HEIGHT 64   // 设置OLED高度,单位:像素

// // 定义OLED重置引脚,虽然教程未使用, 但却是Adafruit_SSD1306库文件所必需的
// #define OLED_RESET 4
// Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// 定义步进电机驱动器引脚
const int PUL = 11;  // 定义脉冲信号引脚
const int DIR = 10;  // 定义方向信号引脚

// GY25相关定义
SoftwareSerial GY(8, 9);  // 虚拟串口，RX为D8，TX为D9
int YPR[3];
unsigned char Re_buf[8], counter = 0;
unsigned char sign = 0;

// 步进电机相关定义
#define STEPS 200
Stepper stepper(STEPS, DIR, PUL);
float initPos = 0;  // 初始步进量
float lastPos = 0;  // 上一次步进量
float nowPos = 0;   // 当前步进量

//float stepAngle = 1.8;  // 步距角
const float microStep = 200;  // 细分数
const float pitchSt = 5;      // 螺距 in mm 记得查看是几头丝杆！

float lastlastlastPitch = 0;  // 上上上次
float lastlastPitch = 0;      // 上上次
float lastPitch = 0;
float pitch = 0;

float mass = 0;
int key_1 = 1;
int key_2 = 1;
float pos[2];


/***************************************************************
  *  @brief     UNO setup function
  *  @param     none
  *  @note      none
 **************************************************************/
void setup() {
  Serial.begin(115200);
  Serial.println("!!!!!Start Up!!!!!");

  GY.begin(115200);
  Wire.begin();
  delay(2000);

  // // 初始化OLED并设置其I2C地址
  // display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  // 初始化步进电机驱动器
  pinMode(PUL, OUTPUT);
  pinMode(DIR, OUTPUT);
  stepper.setSpeed(500);

  // 装置初始化
  stepper.step(-300000);
  Serial.println("!!!!!Finish!!!!!");
}


/***************************************************************
  *  @brief     UNO loop function
  *  @param     none
  *  @note      none
 **************************************************************/
void loop() {

  while (key_1 > 0 || key_2 > 0) {
    while (GY.available()) {
      Re_buf[counter] = (unsigned char)GY.read();
      if (counter == 0 && Re_buf[0] != 0xAA) return;  // 检查帧头
      counter++;
      if (counter == 8) {
        counter = 0;  // 重新赋值，准备下一帧数据的接收
        sign = 1;
      }
    }

    // Serial.println(sign);
    if (sign) {
      Serial.println("****************wzy******************");
      sign = 0;
      if (Re_buf[0] == 0xAA && Re_buf[7] == 0x55) {  // 检查帧头，帧尾
        YPR[0] = (Re_buf[1] << 8 | Re_buf[2]);
        YPR[1] = (Re_buf[3] << 8 | Re_buf[4]);
        YPR[2] = (Re_buf[5] << 8 | Re_buf[6]);

        Serial.print("YPR:\t");
        Serial.print(YPR[0], DEC);
        Serial.print("\t");  // 显示航向
        Serial.print(YPR[1], DEC);
        Serial.print("\t");           // 显示俯仰角
        Serial.println(YPR[2], DEC);  // 显示横滚角
      }
    }

    float d = 0;
    lastPos = nowPos;

    pitch = YPR[1];
    Serial.print("pitch: ");
    Serial.println(pitch);

    if (pitch > 90) {
      stepper.step(-100);
      Serial.println("------");
      d -= 100 / microStep * pitchSt;
    } else if (pitch < -90) {
      stepper.step(+100);
      Serial.println("++++++");
      d += 100 / microStep * pitchSt;
    }

    nowPos = lastPos + d;

    Serial.print("nowPos: ");
    Serial.println(nowPos);

    if (pitch > 0 && lastPitch > 0 && lastlastPitch < 0 && lastlastlastPitch < 0) {
      key_1--;
      pos[0] = nowPos;
    } else if (pitch < 0 && lastPitch < 0 && lastlastPitch > 0 && lastlastlastPitch > 0) {
      key_2--;
      pos[1] = nowPos;
    }

    lastlastlastPitch = lastlastPitch;
    lastlastPitch = lastPitch;
    lastPitch = pitch;

    delay(10);
    // Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>zzx>>>>>>>>>>>>>>>>>>>>>>>");
  }

  mass = getMass(pos[0], pos[1]);

  // // 展示当前状态
  // normal_display(pitch, mass);

  Serial.println(pos[0]);
  Serial.println(pos[1]);
  Serial.println((pos[1] + pos[0]) / 2);
  Serial.println(mass);

  Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>zzx>>>>>>>>>>>>>>>>>>>>>>>");
}


 /***************************************************************
   *  @brief     获取当前质量
   *  @param     pos0: 初次翻转位置
   *  @param     pos1: 再次翻转位置
   *  @note      none
  **************************************************************/
float getMass(float pos0, float pos1) {
  float x;
  x = -(pos0 + pos1) / 2;
  Serial.println(x);
  mass = -1.3225 * x + 1215;
  return mass;
}


/***************************************************************
  *  @brief     OLED 调用
  *  @param     mass: the mass you want to show
  *  @note      使用 OLED 显示信息
 **************************************************************/
// void disp(float mass) {
//     // 清除屏幕
//     display.clearDisplay();

//     // 设置字体颜色,白色可见
//     display.setTextColor(WHITE);

//     // 设置字体大小
//     display.setTextSize(4);

//     // 设置光标位置
//     display.setCursor(0, 20);

//     // 打印 Mass
//     display.print("Mass:    ");
//     display.print(mass);

//     // 显示字符
//     display.display();
// }
