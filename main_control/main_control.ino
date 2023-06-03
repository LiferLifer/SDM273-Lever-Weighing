/***************************************************************
 * @file       main_control
 * @brief      The main control program of the lever weighing device realized by the linear motor module.
 * @author     Author
 * @version    Version 1.2.0
 * @date       2023-05-12
 **************************************************************/

/***************************************************************
 * @brief      连线方法
 * @note       MPU6050-UNO: VCC-VCC // GND-GND // SCL-A5 // SDA-A4 // INT-2 (Optional)
 * @note       OLED-UNO: VCC-VCC // GND-GND // SCL-无名port // SDA-无名port
 * @note       HPD970-UNO: PUL- -> 11 // DIR- -> 10 // EN- 可不接 // 各正极 -> 5V
 **************************************************************/

// 引入所需的库
#include <Kalman.h>
#include <Wire.h>
#include <Math.h>
#include <string.h>
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

// LiquidCrystal_I2C lcd(0x27, 16, 2);

// 定义步进电机驱动器引脚
const int PUL = 11;  // 定义脉冲信号引脚
const int DIR = 10;  // 定义方向信号引脚

float lastlastlastPitch = 0;  // 上上上次
float lastlastPitch = 0;      // 上上次
float lastPitch = 0;
float pitch = 0;  // Pitch角

// SoftwareSerial Voice(6, 7);  // 虚拟串口，RX为D6，TX为D7

// GY25
SoftwareSerial GY(8, 9);  // RX为D8，TX为D9
int YPR[3];
unsigned char Re_buf[8], counter = 0;
unsigned char sign = 0;

// 步进电机相关定义
#define STEPS 200
Stepper stepper(STEPS, DIR, PUL);
// float initPos = 0;  // 初始步进量
float lastPos = 0;  // 上一次步进量
float nowPos = 0;   // 当前步进量

// 其他变量
float mass = 0;
int key_1 = 1;
int key_2 = 1;
int keeeey = 1;
float pos[2];
String str;


/***************************************************************
  *  @brief     UNO setup function
  *  @param     none
  *  @note      none
 **************************************************************/
void setup() {
  Serial.begin(9600);
  // Serial.println("!!!!!Start Up!!!!!");
  Serial.print("233");

  GY.begin(115200);
  Wire.begin();
  delay(2000);

  // // 初始化OLED并设置其I2C地址
  // display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  // lcd.init();
  // lcd.backlight();

  // 初始化步进电机驱动器
  pinMode(PUL, OUTPUT);
  pinMode(DIR, OUTPUT);
  stepper.setSpeed(2000);

  // // 装置平衡初始化
  stepper.step(-300000);
  stepper.setSpeed(700);
  // Serial.println("!!!!!Finish!!!!!");
}


/***************************************************************
  *  @brief     UNO loop function
  *  @param     none
  *  @note      none
 **************************************************************/
void loop() {
  // disp(mass);
  while (key_1 > 0 || key_2 > 0) {
    while (GY.available()) {
      Re_buf[counter] = (unsigned char)GY.read();
      if (counter == 0 && Re_buf[0] != 0xAA) return;  // 检查帧头
      counter++;
      if (counter == 8)  //接收到数据
      {
        counter = 0;  //重新赋值，准备下一帧数据的接收
        sign = 1;
      }
    }

    // 控制步进电机
    // Serial.println(sign);
    if (sign) {
      // Serial.println("****************wzy******************");
      sign = 0;
      if (Re_buf[0] == 0xAA && Re_buf[7] == 0x55) {  // 检查帧头，帧尾
        YPR[0] = (Re_buf[1] << 8 | Re_buf[2]);
        YPR[1] = (Re_buf[3] << 8 | Re_buf[4]);
        YPR[2] = (Re_buf[5] << 8 | Re_buf[6]);

        // Serial.print("YPR:\t");
        // Serial.print(YPR[0], DEC);
        // Serial.print("\t");  // 显示航向
        // Serial.print(YPR[1], DEC);
        // Serial.print("\t");           // 显示俯仰角
        // Serial.println(YPR[2], DEC);  // 显示横滚角
      }
    }

    float d = 0;
    lastPos = nowPos;

    // Serial.println("--------------level!---------------");
    pitch = YPR[1];
    // Serial.print("pitch: ");
    // Serial.println(pitch);

    if (pitch > 90) {
      stepper.step(-100);
      Serial.println("------");
      d -= 2.5;
    } else if (pitch < -90) {
      stepper.step(+100);
      Serial.println("++++++");
      d += 2.5;
    }

    nowPos = lastPos + d;
    // Serial.print("nowPos: ");
    // Serial.println(nowPos);


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
  }

  mass = getMass(pos[0], pos[1]);

  // str = String(mass, 2);
  // Serial.print(str);
  delay(1500);

  if (keeeey == 1) {
    // Serial.print("您已经重达");
    mass = getMass(pos[0], pos[1]);
    str = String(mass, 2);
    Serial.print(str);
    // Serial.print("克辣");
    delay(3000);
    keeeey--;
  }

  // Serial.println(pos[0]);
  // Serial.println(pos[1]);
  // Serial.print("Pos:  ");
  // Serial.println((pos[1] + pos[0]) / 2);
  // Serial.print("mass: ");
  // Serial.println(mass);
  // disp(mass);

  // Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>zzx>>>>>>>>>>>>>>>>>>>>>>>");
}

float getMass(float pos0, float pos1) {
  float x;
  x = -(pos0 + pos1) / 2;
  mass = -1.4097 * x + 1187.7;
  return mass;
}


/***************************************************************
  *  @brief     LCD 调用
  *  @param     mass: the mass you want to show
  *  @note      使用 LCD 显示信息
 **************************************************************/
// void disp(float mass) {
//   lcd.setCursor(0, 0);
//   lcd.print("SDM273");
//   lcd.setCursor(0, 1);
//   lcd.print("Mass: ");
//   lcd.print(mass);
// }


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
//     display.setTextSize(1.5);

//     // 设置光标位置
//     display.setCursor(0, 20);

//     // 打印 Mass
//     display.print("Mass:    ");
//     display.print(mass);

//     // 显示字符
//     display.display();
// }
