// 舵机控制配置
#define SERVO1_PIN 2        // 舵机1信号引脚（控制Pitch）
#define SERVO2_PIN 3        // 舵机2信号引脚（控制Roll）

mpu6500采用i2c方式通讯

舵机角度可以按需要对应到融合运算的角度量或者陀螺仪自身角度数据

融合计算的角度加入了卡尔曼滤波器，自动修正角度的积累误差

<img width="500"  alt="image" src="https://github.com/user-attachments/assets/f9f2487b-4f4f-4a0a-9d42-b259748770dc" />
