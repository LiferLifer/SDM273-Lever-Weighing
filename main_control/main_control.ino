/***************************************************************
 * @file       main_control
 * @brief      The main control program of the lever weighing device realized by the linear motor module.
 * @author     Author
 * @version    Version 1.0.4
 * @date       2023-05-12(Origin)
 **************************************************************/

/***************************************************************
 * @brief      连线方法
 * @note       MPU6050-UNO: VCC-VCC // GND-GND // SCL-A5 // SDA-A4 // INT-2 (Optional)
 * @note       OLED-UNO: VCC-VCC // GND-GND // SCL-无名port // SDA-无名port
 * @note       HPD970-UNO: PUL- --> 11 // DIR- --> 10 // EN- --> GND // 各正极 --> 5V
 **************************************************************/

// 引入驱动MPU6050所需的库
#include <Kalman.h>
#include <Wire.h>
#include <Math.h>

// // 引入驱动OLED所需的库
// #include <Adafruit_GFX.h>
// #include <Adafruit_SSD1306.h>

// 步进电机控制库
#include <Stepper.h>

// // 定义OLED阵列长宽
// #define SCREEN_WIDTH 128   // 设置OLED宽度,单位:像素
// #define SCREEN_HEIGHT 64   // 设置OLED高度,单位:像素

// // 定义OLED重置引脚,虽然教程未使用, 但却是Adafruit_SSD1306库文件所必需的
// #define OLED_RESET 4
// Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// 定义步进电机驱动器引脚
const int PUL = 11;  // 定义脉冲信号引脚
const int DIR = 10;  // 定义方向信号引脚

// MPU6050相关定义
const int MPU = 0x68;  // MPU-6050的I2C地址
const int nValCnt = 7;  // 一次读取寄存器的数量
const float fRad2Deg = 57.295779513f;  // 将弧度转为角度的乘数

const int nCalibTimes = 1000;  // 校准时读数的次数
int calibData[nValCnt];  // 初始校准数据

unsigned long nLastTime = 0;  // 上一次读数的时间
Kalman kalmanPitch;  // Pitch角滤波器

float pitch;   // Pitch角

// 步进电机相关定义
#define STEPS 200  // 转一周需要的步数（脉冲数）
Stepper stepper(STEPS, DIR, PUL);

float initPos = 0;  // 初始步进量
float lastPos = 0;  // 上一次步进量
float nowPos = 0;  // 当前步进量
float diff = 0;

//float stepAngle = 1.8;  // 步距角
const float microStep = 200;  // 每 x 个脉冲转一圈
const float pitchSt = 5;  // 螺距 in mm 记得查看是几头丝杆！

// 其他变量
float mass = 0;
const float K = 0;  // 测算得到的比例系数! 


/***************************************************************
  *  @brief     UNO setup function
  *  @param     none
  *  @note      none
 **************************************************************/
void setup() {
    Serial.begin(9600);   // 初始化串口，指定波特率
    Wire.begin();         // 初始化Wire库

    // 初始化MPU6050
    WriteMPUReg(0x6B, 0);   // 启动MPU6050设备
    Calibration();          // 执行校准
    nLastTime = micros();   // 记录当前时间

    // // 初始化OLED并设置其I2C地址
    // display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

    // 初始化步进电机驱动器
    pinMode(PUL, OUTPUT);
    pinMode(DIR, OUTPUT);
    stepper.setSpeed(600);  // 每分钟电机转动步数，正整数

    // // 装置平衡初始化
    // words_display("Initializing", "Please wait...");
    // levelInit();
    // stepper.step(-100);

    Serial.println("Initialize!");
}


/***************************************************************
  *  @brief     UNO loop function
  *  @param     none
  *  @note      none
 **************************************************************/
void loop() {
    // 控制步进电机
    level();

    // // 展示当前状态
    // normal_display(pitch, mass);

    delay(500);
    Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>zzx>>>>>>>>>>>>>>>>>>>>>>>");
}


/***************************************************************
  *  @brief     MPU6050 读取俯仰角
  *  @param     none
  *  @note      读取 pitch 角，并进行卡尔曼滤波
 **************************************************************/
float getPitch() {
    int readouts[nValCnt];
    ReadAccGyr(readouts);   // 读出测量值

    float realVals[nValCnt];  // update: 7 to nValCnt
    Rectify(readouts, realVals);   // 根据校准的偏移量进行纠正

    //计算加速度向量的模长，均以g为单位
    float fNorm = sqrt(realVals[0] * realVals[0] + realVals[1] * realVals[1] + realVals[2] * realVals[2]);
    float fPitch = getRawPitch(realVals, fNorm);   // 计算Pitch角

    if (realVals[0] < 0) {
      fPitch = -fPitch;
    }

    // Serial.print("InitPitch:");
    // Serial.println(fPitch);

    // 计算两次测量的时间间隔dt，以秒为单位
    unsigned long nCurTime = micros();
    float dt = (double) (nCurTime - nLastTime) / 1000000.0;
    nLastTime = nCurTime;

    // 对Pitch角进行卡尔曼滤波
    float fNewPitch = kalmanPitch.getAngle(fPitch, realVals[5], dt);

    // 向串口打印输出Pitch角，运行时在Arduino的串口监视器中查看
    // Serial.print("Pitch:");
    // Serial.println(fNewPitch);

    return fNewPitch;
}


/***************************************************************
  *  @brief     MPU6050 寄存器写入
  *  @param     nReg 指定的寄存器写入地址
  *  @param     nVal 要写入的一个字节的值
  *  @note      向 MPU6050 写入一个字节的数据
 **************************************************************/
void WriteMPUReg(int nReg, unsigned char nVal) {
    Wire.beginTransmission(MPU);
    Wire.write(nReg);
    Wire.write(nVal);
    Wire.endTransmission(true);
}


/***************************************************************
  *  @brief     MPU6050 寄存器读出
  *  @param     nReg 指定的寄存器读出地址
  *  @note      从 MPU6050 读出一个字节的数据，返回读出的值
 **************************************************************/
unsigned char ReadMPUReg(int nReg) {
    Wire.beginTransmission(MPU);
    Wire.write(nReg);
    Wire.requestFrom(MPU, 1, true);
    Wire.endTransmission(true);
    return Wire.read();
}


/***************************************************************
  *  @brief     MPU6050 传感器读数
  *  @param     *pVals 指定的数组地址
  *  @note      读出加速度计三个分量、温度和三个角速度，保存在指定数组
 **************************************************************/
void ReadAccGyr(int *pVals) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.requestFrom(MPU, nValCnt * 2, true);
    Wire.endTransmission(true);
    for (long i = 0; i < nValCnt; ++i) {
        pVals[i] = Wire.read() << 8 | Wire.read();
    }
}


/***************************************************************
  *  @brief     初始化校准偏移
  *  @param     none
  *  @note      对大量读数进行统计，校准平均偏移量
 **************************************************************/
void Calibration() {
    float valSums[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0};
    // 先求和
    for (int i = 0; i < nCalibTimes; ++i) {
        int mpuVals[nValCnt];
        ReadAccGyr(mpuVals);
        for (int j = 0; j < nValCnt; ++j) {
            valSums[j] += mpuVals[j];
        }
    }
    // 再求平均
    for (int i = 0; i < nValCnt; ++i) {
        calibData[i] = int(valSums[i] / nCalibTimes);
    }
    // 设芯片Z轴竖直向下，设定静态工作点。
    calibData[2] += 16384;
}

/***************************************************************
  *  @brief     MPU6050 获取 Pitch 角
  *  @param     *pRealVals 校准后的测量值
  *  @param     fNorm 加速度向量的模长
  *  @note      返回计算得出 Pitch 角
 **************************************************************/
float getRawPitch(float *pRealVals, float fNorm) {
    float fNormYZ = sqrt(pRealVals[1] * pRealVals[1] + pRealVals[2] * pRealVals[2]);
    float fCos = fNormYZ / fNorm;
    return acos(fCos) * fRad2Deg;
}


/***************************************************************
  *  @brief     MPU6050 读数纠正消偏
  *  @param     *pReadout 指定的读入数组地址
  *  @param     *pRealVals 指定的校准数组地址
  *  @note      对读数进行纠正，消除偏移，并转换为物理量
 **************************************************************/
void Rectify(int *pReadout, float *pRealVals) {
    for (int i = 0; i < 3; ++i) {
        pRealVals[i] = (float) (pReadout[i] - calibData[i]) / 16384.0f;
    }
    pRealVals[3] = pReadout[3] / 340.0f + 36.53;
    for (int i = 4; i < 7; ++i) {
        pRealVals[i] = (float) (pReadout[i] - calibData[i]) / 131.0f;
    }
}


/***************************************************************
  *  @brief     平衡初始化
  *  @param     none
  *  @note      none
 **************************************************************/
// void levelInit() {
//     float d = 0;
//     mass = 0;
//     do {
//         pitch = getPitch();
//         stepper.step(-30);
//         d += 30;
//     } while(pitch < 0.5 && pitch > -0.5);
//     normal_display(pitch, mass);
//     initPos = d;
// }


/***************************************************************
  *  @brief     试图平衡
  *  @param     none
  *  @note      none
 **************************************************************/
void level() {
    float d = 0;
    lastPos = nowPos;
    Serial.println("--------------level!---------------");
    // pitch = getPitch();
    // stepper.step(2048);

    while(1) {
        Serial.println("--------------level while() in!---------------");
        pitch = getPitch();
        if(pitch > 0.5) {
            stepper.step(500);
            Serial.println("+++++++++++++++++++++++");
            d += 500 / microStep * pitchSt;
        } else if(pitch < -0.5) {
            stepper.step(-500);
            Serial.println("----------jxy----------");
            d -= 500 / microStep * pitchSt;
        }
        delay(10);
    }

    Serial.println("pitch: ");
    Serial.println(pitch);

//     do {
//       // Serial.println("wzy-----------------------------------------");
//         if(pitch > 0.5) {
//           Serial.println("wzy-----------------------------------------");
//             // advance(5, 1);
//             stepper.step(-10);
//             // d += 5;
//         } else if(pitch < -0.5) {
//             // advance(5, 0);
//             Serial.println("jxy$$$$$$$$$$$$$$$$$$$$$$$$$$$");
//             // d -= 5;
//         }
//         pitch = getPitch();
// //        Serial.print('0');
//     } while (pitch < 0.5 && pitch > -0.5);

    nowPos = lastPos + d;
    // diff = nowPos - initPos;
    // mass = getMass(diff);
    // normal_display(pitch, mass);
}


 /***************************************************************
   *  @brief     获取当前质量
   *  @param     diff: 位移差
   *  @note      none
  **************************************************************/
// float getMass(float diff) {
//     float nowMass = diff * K;
//     return nowMass;
// }


/***************************************************************
  *  @brief     OLED 一般调用
  *  @param     pitch: the pitch you want to show
  *  @param     mass: the mass you want to show
  *  @note      使用 OLED 显示计算得出的 pitch 以及 weight
 **************************************************************/
// void normal_display(float pitch, float mass) {
//     // 清除屏幕
//     display.clearDisplay();

//     // 设置字体颜色,白色可见
//     display.setTextColor(WHITE);

//     // 设置字体大小
//     display.setTextSize(3);

//     // 设置光标位置
//     display.setCursor(0, 20);

//     // 显示pitch
//     display.print("Pitch:   ");
//     display.print(pitch);

//     // 显示weight
//     display.setCursor(0, 45);
//     display.print("Mass:    ");
//     display.print(mass);

//     // 显示字符
//     display.display();
// }