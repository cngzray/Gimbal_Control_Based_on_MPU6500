#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>

// 舵机控制配置
#define SERVO1_PIN 2        // 舵机1信号引脚（控制Pitch）
#define SERVO2_PIN 3        // 舵机2信号引脚（控制Roll）

// 舵机角度范围
#define SERVO_MIN_ANGLE 0   // 最小角度
#define SERVO_MAX_ANGLE 180 // 最大角度

// Pitch角度范围（根据实际测试调整）
#define PITCH_MIN -90.0  // 最小Pitch角
#define PITCH_MAX 90.0   // 最大Pitch角

// Roll角度范围（根据实际测试调整）
#define ROLL_MIN -90.0   // 最小Roll角
#define ROLL_MAX 90.0    // 最大Roll角

// 舵机对象
Servo servo1;  // 舵机1（控制Pitch）
Servo servo2;  // 舵机2（控制Roll）



// MPU6500寄存器地址
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define GYRO_XOUT_H  0x43
#define GYRO_XOUT_L  0x44
#define GYRO_YOUT_H  0x45
#define GYRO_YOUT_L  0x46
#define GYRO_ZOUT_H  0x47
#define GYRO_ZOUT_L  0x48
#define PWR_MGMT_1   0x6B
#define WHO_AM_I     0x75

// 传感器数据变量
float ax, ay, az;  // 加速度计数据（g）
float gx, gy, gz;  // 陀螺仪数据（°/s）

// 角度数据变量
float pitchAcc = 0.0;  // 加速度计计算的俯仰角
float rollAcc = 0.0;   // 加速度计计算的横滚角
float pitchGyro = 0.0; // 陀螺仪积分的俯仰角
float rollGyro = 0.0;  // 陀螺仪积分的横滚角

// 卡尔曼滤波器参数
float Q_angle = 0.001;    // 角度过程噪声协方差
float Q_bias = 0.003;     // 陀螺仪偏差过程噪声协方差
float R_measure = 0.03;   // 测量噪声协方差

// 卡尔曼滤波器状态变量
float pitch = 0.0;        // 最优角度估计（融合后）
float roll = 0.0;         // 最优角度估计（融合后）
float pitch_bias = 0.0;   // 陀螺仪偏差估计
float roll_bias = 0.0;

// 卡尔曼滤波器协方差矩阵
float P_pitch[2][2] = {{1, 0}, {0, 1}};
float P_roll[2][2] = {{1, 0}, {0, 1}};

// 时间变量
unsigned long lastTime = 0;  // 上次更新时间

// 角度漂移修正参数
float pitchOffset = 0.0;  // Pitch角度偏移校准值
float rollOffset = 0.0;   // Roll角度偏移校准值
bool isCalibrated = false; // 是否已完成校准

// 函数声明
void initMPU6500();
void readMPU6500();
void writeMPU6500(uint8_t reg, uint8_t data);
int16_t readMPU6500Word(uint8_t reg);
void calculateAngle();
void kalmanUpdate(float* angle, float* bias, float gyro, float angle_measure, float dt, float P[2][2]);
void calibrateSensor();
void setServo1Angle(float pitchAngle);
void setServo2Angle(float rollAngle);
float mapPitchToServo(float pitchAngle);
float mapRollToServo(float rollAngle);

// MPU6500 I2C地址（根据实际检测结果设置）
#define MPU6500_ADDR 0x68

void setup() {
  // 初始化串口（ESP32-C3使用USB CDC）
  Serial.begin(115200);
  
  // 等待USB CDC初始化完成
  delay(2000);
  
  Serial.println("====================");
  Serial.println("MPU6500 + 双舵机 测试程序");
  Serial.println("====================");
  
  // 初始化I2C（SDA=GPIO4, SCL=GPIO5）
  Wire.begin(4, 5);
  
  // 初始化MPU6500
  initMPU6500();
  
  // 传感器校准
  calibrateSensor();
  
  // 初始化舵机1（控制Pitch）
  servo1.attach(SERVO1_PIN);
  servo1.write(90);  // 初始位置设为中间角度(90°)
  
  // 初始化舵机2（控制Roll）
  servo2.attach(SERVO2_PIN);
  servo2.write(90);  // 初始位置设为中间角度(90°)
  
  Serial.println("双舵机初始化完成");
}

void loop() {
  // 读取传感器数据
  readMPU6500();
  
  // 计算角度（互补滤波融合）
  calculateAngle();
  
  // 控制舵机1跟随Gyro_Pitch角度
  setServo1Angle(pitchGyro);
  
  // 控制舵机2跟随Gyro_Roll角度
  setServo2Angle(rollGyro);
  
  // 输出数据
  Serial.print("加速度: ");
  Serial.print(ax, 2); Serial.print(" g, ");
  Serial.print(ay, 2); Serial.print(" g, ");
  Serial.print(az, 2); Serial.println(" g");
  
  Serial.print("陀螺仪: ");
  Serial.print(gx, 2); Serial.print(" °/s, ");
  Serial.print(gy, 2); Serial.print(" °/s, ");
  Serial.print(gz, 2); Serial.println(" °/s");
  
  Serial.print("角度(融合后): ");
  Serial.print("Pitch="); Serial.print(pitch, 2); Serial.print("°, ");
  Serial.print("Roll="); Serial.print(roll, 2); Serial.println("°");
  
  Serial.print("角度(Raw): ");
  Serial.print("Acc_Pitch="); Serial.print(pitchAcc, 2); Serial.print("°, ");
  Serial.print("Acc_Roll="); Serial.print(rollAcc, 2); Serial.print("°, ");
  Serial.print("Gyro_Pitch="); Serial.print(pitchGyro, 2); Serial.print("°, ");
  Serial.print("Gyro_Roll="); Serial.print(rollGyro, 2); Serial.println("°");
  
  // 校准状态显示
  static int calibCount = 0;
  if (calibCount++ % 40 == 0) {
    Serial.print("校准状态: ");
    Serial.print(isCalibrated ? "已校准" : "未校准");
    Serial.print(" | 偏移量: Pitch="); Serial.print(pitchOffset, 2);
    Serial.print("°, Roll="); Serial.print(rollOffset, 2); Serial.println("°");
  }
  
  // 舵机调试信息
  static int servoDebugCount = 0;
  if (servoDebugCount++ % 20 == 0) { // 每1秒输出一次
    Serial.print("舵机1调试: Gyro_Pitch="); Serial.print(pitchGyro, 1);
    Serial.print("°, 映射角度="); Serial.print(mapPitchToServo(pitchGyro), 1);
    Serial.println("°");
    Serial.print("舵机2调试: Gyro_Roll="); Serial.print(rollGyro, 1);
    Serial.print("°, 映射角度="); Serial.print(mapRollToServo(rollGyro), 1);
    Serial.println("°");
  }
  
  Serial.println("------------------------");
  
  delay(50);
}

// 扫描I2C总线设备
void scanI2C() {
  Serial.println("扫描I2C总线...");
  byte error, address;
  int nDevices = 0;
  
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("I2C设备地址: 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      nDevices++;
    }
  }
  
  if (nDevices == 0) {
    Serial.println("未找到任何I2C设备!");
  } else {
    Serial.print("找到 ");
    Serial.print(nDevices);
    Serial.println(" 个I2C设备");
  }
}

// 初始化MPU6500
void initMPU6500() {
  // 扫描I2C总线
  scanI2C();
  
  // 检查设备是否连接
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(WHO_AM_I);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6500_ADDR, 1);
  
  if (Wire.available()) {
    uint8_t whoami = Wire.read();
    
    Serial.print("WHO_AM_I寄存器值: 0x");
    Serial.println(whoami, HEX);
    
    if (whoami == 0x70 || whoami == 0x68) {  // MPU6500=0x70, MPU6050=0x68
      Serial.println("传感器连接成功!");
    } else {
      Serial.println("传感器WHO_AM_I不匹配!");
      Serial.println("MPU6500应返回0x70, MPU6050应返回0x68");
      while (1) { delay(1000); }
    }
  } else {
    Serial.print("无法连接到传感器! 检查地址0x");
    Serial.println(MPU6500_ADDR, HEX);
    Serial.println("请确认接线: SDA=GPIO4, SCL=GPIO5");
    Serial.println("尝试其他地址: 修改MPU6500_ADDR为0x69");
    while (1) { delay(1000); }
  }
  
  // 唤醒传感器（解除睡眠模式）
  writeMPU6500(PWR_MGMT_1, 0x00);
  
  Serial.println("传感器初始化完成");
}

// 读取MPU6500数据
void readMPU6500() {
  // 读取加速度计数据
  int16_t ax_raw = readMPU6500Word(ACCEL_XOUT_H);
  int16_t ay_raw = readMPU6500Word(ACCEL_YOUT_H);
  int16_t az_raw = readMPU6500Word(ACCEL_ZOUT_H);
  
  // 读取陀螺仪数据
  int16_t gx_raw = readMPU6500Word(GYRO_XOUT_H);
  int16_t gy_raw = readMPU6500Word(GYRO_YOUT_H);
  int16_t gz_raw = readMPU6500Word(GYRO_ZOUT_H);
  
  // 转换为实际物理值
  // 加速度计量程默认±2g，灵敏度16384 LSB/g
  ax = (float)ax_raw / 16384.0;
  ay = (float)ay_raw / 16384.0;
  az = (float)az_raw / 16384.0;
  
  // 陀螺仪量程默认±250°/s，灵敏度131 LSB/(°/s)
  gx = (float)gx_raw / 131.0;
  gy = (float)gy_raw / 131.0;
  gz = (float)gz_raw / 131.0;
}

// 向MPU6500写入数据
void writeMPU6500(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

// 从MPU6500读取16位数据
int16_t readMPU6500Word(uint8_t reg) {
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6500_ADDR, 2);
  
  int16_t value = 0;
  if (Wire.available() >= 2) {
    value = (Wire.read() << 8) | Wire.read();
  }
  
  return value;
}

// 传感器校准（计算静止时的角度偏移）
void calibrateSensor() {
  Serial.println("正在校准传感器，请保持静止...");
  
  const int calibSamples = 100;
  float pitchSum = 0.0, rollSum = 0.0;
  
  for (int i = 0; i < calibSamples; i++) {
    readMPU6500();
    // 计算静止角度
    float tempPitch = atan2(ay, sqrt(ax * ax + az * az)) * (180.0 / PI);
    float tempRoll = atan2(-ax, az) * (180.0 / PI);
    pitchSum += tempPitch;
    rollSum += tempRoll;
    delay(10);
  }
  
  // 计算平均偏移量
  pitchOffset = pitchSum / calibSamples;
  rollOffset = rollSum / calibSamples;
  
  Serial.print("校准完成! 偏移量: Pitch=");
  Serial.print(pitchOffset, 2);
  Serial.print("°, Roll=");
  Serial.print(rollOffset, 2);
  Serial.println("°");
  
  isCalibrated = true;
}

// 卡尔曼滤波器更新（单个轴）
void kalmanUpdate(float* angle, float* bias, float gyro, float angle_measure, float dt, float P[2][2]) {
  // 预测步骤
  *angle += (gyro - *bias) * dt;
  
  // 更新协方差矩阵
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;
  
  // 更新步骤
  float y = angle_measure - *angle;  // 残差
  float S = P[0][0] + R_measure;     // 残差协方差
  
  // 卡尔曼增益
  float K[2];
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;
  
  // 更新状态估计
  *angle += K[0] * y;
  *bias += K[1] * y;
  
  // 更新协方差矩阵
  float P00_temp = P[0][0];
  float P01_temp = P[0][1];
  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;
}

// 卡尔曼滤波融合角度计算
void calculateAngle() {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;  // 时间间隔（秒）
  lastTime = currentTime;
  
  // 限制dt范围，避免异常值
  if (dt > 0.1) dt = 0.05;
  if (dt <= 0) dt = 0.001;
  
  // 重力向量归一化（提高加速度计角度准确性）
  float accMagnitude = sqrt(ax * ax + ay * ay + az * az);
  
  // 从加速度计计算角度（静态角度）
  float pitchAccRaw = atan2(ay, sqrt(ax * ax + az * az)) * (180.0 / PI) - pitchOffset;
  float rollAccRaw = atan2(-ax, az) * (180.0 / PI) - rollOffset;
  
  // 只有在静止时才使用加速度计数据（加速度接近1g）
  // 动态时加速度计数据不可靠
  float pitchAcc = pitchAccRaw;
  float rollAcc = rollAccRaw;
  
  if (accMagnitude < 0.8 || accMagnitude > 1.2) {
    // 运动状态，降低加速度计信任度
    pitchAcc = pitch;  // 使用当前估计值作为测量值
    rollAcc = roll;
  }
  
  // 角度边界约束
  pitchAcc = constrain(pitchAcc, -90.0, 90.0);
  rollAcc = constrain(rollAcc, -90.0, 90.0);
  
  // 卡尔曼滤波更新
  kalmanUpdate(&pitch, &pitch_bias, gx, pitchAcc, dt, P_pitch);
  kalmanUpdate(&roll, &roll_bias, gy, rollAcc, dt, P_roll);
  
  // 存储Raw角度用于显示
  pitchAcc = pitchAccRaw;
  rollAcc = rollAccRaw;
  pitchGyro = pitch + pitch_bias * dt;
  rollGyro = roll + roll_bias * dt;
  
  // 角度边界约束（防止漂移累积）
  pitch = constrain(pitch, -180.0, 180.0);
  roll = constrain(roll, -180.0, 180.0);
}

// 将Pitch角度映射到舵机角度
float mapPitchToServo(float pitchAngle) {
  return ((pitchAngle - PITCH_MIN) / (PITCH_MAX - PITCH_MIN)) * (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE) + SERVO_MIN_ANGLE;
}

// 将Roll角度映射到舵机角度
float mapRollToServo(float rollAngle) {
  return ((rollAngle - ROLL_MIN) / (ROLL_MAX - ROLL_MIN)) * (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE) + SERVO_MIN_ANGLE;
}

// 设置舵机1角度（控制Pitch）
void setServo1Angle(float pitchAngle) {
  // 将Pitch角度(-90°~90°)映射到舵机角度(0°~180°)
  float servoAngle = mapPitchToServo(pitchAngle);
  
  // 限制角度范围
  servoAngle = constrain(servoAngle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  
  // 使用Servo库设置角度
  servo1.write(servoAngle);
}

// 设置舵机2角度（控制Roll）
void setServo2Angle(float rollAngle) {
  // 将Roll角度(-90°~90°)映射到舵机角度(0°~180°)
  float servoAngle = mapRollToServo(rollAngle);
  
  // 限制角度范围
  servoAngle = constrain(servoAngle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  
  // 使用Servo库设置角度
  servo2.write(servoAngle);
}