#include "myiic.h"
#include "main.h"
#include <math.h>


// 延时函数（根据主频调整）
void I2C_Delay(void) {
    for (volatile int i = 0; i < 10; i++); // 调整循环次数以满足时序
}

// 初始化GPIO
void I2C_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // 使能GPIOA时钟
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    // 配置SCL和SDA为开漏输出
    GPIO_InitStruct.Pin = I2C_SCL_PIN | I2C_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // 初始状态拉高总线
    HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_SET);
}

// 起始信号
void I2C_Start(void) {
    HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_SET);
    I2C_Delay();
    HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_RESET);
    I2C_Delay();
    HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_RESET);
    I2C_Delay();
}

// 停止信号
void I2C_Stop(void) {
    HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_SET);
    I2C_Delay();
    HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_SET);
    I2C_Delay();
}

// 发送应答
void I2C_Ack(void) {
    HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_SET);
    I2C_Delay();
    HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_RESET);
    I2C_Delay();
    HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_SET);
}

// 发送非应答
void I2C_NAck(void) {
    HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_SET);
    I2C_Delay();
    HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_RESET);
    I2C_Delay();
}

// 等待应答
uint8_t I2C_Wait_Ack(void) {
    GPIO_PinState ack;
    HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_SET);
    I2C_Delay();
    
    // 切换SDA为输入模式
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = I2C_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(I2C_SDA_PORT, &GPIO_InitStruct);
    
    ack = HAL_GPIO_ReadPin(I2C_SDA_PORT, I2C_SDA_PIN);
    HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_RESET);
    I2C_Delay();
    
    // 恢复SDA为输出模式
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    HAL_GPIO_Init(I2C_SDA_PORT, &GPIO_InitStruct);
    
    return (ack == GPIO_PIN_RESET) ? 0 : 1;
}

// 发送一个字节
void I2C_SendByte(uint8_t byte) {
    for (int i = 0; i < 8; i++) {
        HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_RESET);
        I2C_Delay();
        if (byte & 0x80) {
            HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_RESET);
        }
        byte <<= 1;
        I2C_Delay();
        HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_SET);
        I2C_Delay();
    }
    HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_RESET);
}

// 接收一个字节
uint8_t I2C_ReadByte(uint8_t ack) {
    uint8_t byte = 0;
    
    // 切换SDA为输入模式
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = I2C_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(I2C_SDA_PORT, &GPIO_InitStruct);
    
    for (int i = 0; i < 8; i++) {
        HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_RESET);
        I2C_Delay();
        HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_SET);
        I2C_Delay();
        byte <<= 1;
        if (HAL_GPIO_ReadPin(I2C_SDA_PORT, I2C_SDA_PIN) == GPIO_PIN_SET) {
            byte |= 0x01;
        }
    }
    
    // 恢复SDA为输出模式
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    HAL_GPIO_Init(I2C_SDA_PORT, &GPIO_InitStruct);
    
    HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_RESET);
    if (ack) {
        I2C_Ack();
    } else {
        I2C_NAck();
    }
    return byte;
}

// 向MPU6050写寄存器
void MPU6050_WriteReg(uint8_t reg, uint8_t data) {
    I2C_Start();
    I2C_SendByte(MPU6050_ADDR);
    I2C_Wait_Ack();
    I2C_SendByte(reg);
    I2C_Wait_Ack();
    I2C_SendByte(data);
    I2C_Wait_Ack();
    I2C_Stop();
}

// 从MPU6050读寄存器
uint8_t MPU6050_ReadReg(uint8_t reg) {
    uint8_t data;
    I2C_Start();
    I2C_SendByte(MPU6050_ADDR);
    I2C_Wait_Ack();
    I2C_SendByte(reg);
    I2C_Wait_Ack();
    
    I2C_Start();
    I2C_SendByte(MPU6050_ADDR | 0x01);
    I2C_Wait_Ack();
    data = I2C_ReadByte(0); // 最后字节不发送ACK
    I2C_Stop();
    return data;
}

// 初始化MPU6050
void MPU6050_Init(void) {
    I2C_Init();
    
    HAL_Delay(100);
    
    MPU6050_WriteReg(PWR_MGMT_1, 0x00);   //解除休眠状态
    
    HAL_Delay(10);
    
    MPU6050_WriteReg(SMPLRT_DIV, 0x07);   //陀螺仪采样率，1KHz
    MPU6050_WriteReg(CONFIG, 0x06);       //低通滤波器的设置，截止频率是1K，带宽是5K
    MPU6050_WriteReg(GYRO_CONFIG, 0x18);  //陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
    MPU6050_WriteReg(ACCEL_CONFIG, 0x00); //配置加速度传感器工作在2G模式，不自检
}

/**
  * @brief   从MPU6050寄存器读取数据
  * @param   
  * @retval  
  */
void MPU6050_ReadData(u8 reg_add,unsigned char*Read,u8 num)
{
	unsigned char i;
	
	I2C_Start();
	I2C_SendByte(MPU6050_ADDR);
	I2C_Wait_Ack();
	I2C_SendByte(reg_add);
	I2C_Wait_Ack();
	
	I2C_Start();
	I2C_SendByte(MPU6050_ADDR+1);
	I2C_Wait_Ack();
	
	for(i=0;i<(num-1);i++){
		*Read=I2C_ReadByte(1);
		Read++;
	}
	*Read=I2C_ReadByte(0);
	I2C_Stop();
}

/**
  * @brief   读取MPU6050的加速度数据
  * @param   
  * @retval  
  */
void MPU6050ReadAcc(short *accData)
{
    u8 buf[6];
    MPU6050_ReadData(ACCEL_XOUT_H, buf, 6);
    accData[0] = (buf[0] << 8) | buf[1];
    accData[1] = (buf[2] << 8) | buf[3];
    accData[2] = (buf[4] << 8) | buf[5];
}

/**
  * @brief   读取MPU6050的角加速度数据
  * @param   
  * @retval  
  */
void MPU6050ReadGyro(short *gyroData)
{
    u8 buf[6];
    MPU6050_ReadData(GYRO_XOUT_H,buf,6);
    gyroData[0] = (buf[0] << 8) | buf[1];
    gyroData[1] = (buf[2] << 8) | buf[3];
    gyroData[2] = (buf[4] << 8) | buf[5];
}


/**
  * @brief   读取MPU6050的原始温度数据
  * @param   
  * @retval  
  */
void MPU6050ReadTemp(short *tempData)
{
	u8 buf[2];
    MPU6050_ReadData(TEMP_OUT_H,buf,2);     //读取温度值
    *tempData = (buf[0] << 8) | buf[1];
}


/**
  * @brief   读取MPU6050的温度数据，转化成摄氏度
  * @param   
  * @retval  
  */
void MPU6050_ReturnTemp(float *Temperature)
{
	short temp3;
	u8 buf[2];
	
	MPU6050_ReadData(TEMP_OUT_H,buf,2);     //读取温度值
    temp3= (buf[0] << 8) | buf[1];	
	*Temperature=((double) temp3/340.0)+36.53;

}

// 读取原始数据
void MPU6050_ReadRawData(int16_t *accel, int16_t *gyro) {
    uint8_t buf[14];
    
    // 读取14字节数据 (0x3B - 0x48)
    I2C_Start();
    I2C_SendByte(MPU6050_ADDR);
    I2C_Wait_Ack();
    I2C_SendByte(ACCEL_XOUT_H);
    I2C_Wait_Ack();
    
    I2C_Start();
    I2C_SendByte(MPU6050_ADDR | 0x01);
    I2C_Wait_Ack();
    
    for (int i = 0; i < 13; i++) {
        buf[i] = I2C_ReadByte(1); // 发送ACK
    }
    buf[13] = I2C_ReadByte(0);    // 最后字节不发送ACK
    I2C_Stop();
    // 组合数据
    accel[0] = (int16_t)((buf[0] << 8) | buf[1]);  // Accel X accel_base, gyro_base
    accel[1] = (int16_t)((buf[2] << 8) | buf[3]);  // Accel Y
    accel[2] = (int16_t)((buf[4] << 8) | buf[5]);  // Accel Z
    gyro[0]  = (int16_t)((buf[8] << 8) | buf[9]);  // Gyro X
    gyro[1]  = (int16_t)((buf[10] << 8) | buf[11]);// Gyro Y
    gyro[2]  = (int16_t)((buf[12] << 8) | buf[13]);// Gyro Z    
//    // 组合数据
//    accel[0] = (int16_t)((buf[0] << 8) | buf[1]) - accel_base[0];  // Accel X accel_base, gyro_base
//    accel[1] = (int16_t)((buf[2] << 8) | buf[3]) - accel_base[1];  // Accel Y
//    accel[2] = (int16_t)((buf[4] << 8) | buf[5]) - accel_base[2];  // Accel Z
//    gyro[0]  = (int16_t)((buf[8] << 8) | buf[9]) - gyro_base[0];  // Gyro X
//    gyro[1]  = (int16_t)((buf[10] << 8) | buf[11]) - gyro_base[1];// Gyro Y
//    gyro[2]  = (int16_t)((buf[12] << 8) | buf[13]) - gyro_base[2];// Gyro Z
}
float roll_dt = 0, pitch_dt = 0, yaw_dt = 0;
float prev_time; // 初始时间
// 计算俯仰角(Pitch)和横滚角(Roll)
void CalculateAngles(int16_t *accel, int16_t *gyro, float *pitch, float *roll, float *yaw) {
//    float alpha = 0.9;       // 互补滤波系数
    
    // 转换为g单位 (±2g范围)
    float acc_x_g = accel[0] / 16384.0f;
    float acc_y_g = accel[1] / 16384.0f;
    float acc_z_g = accel[2] / 16384.0f;
    
    float gyro_x_dps = gyro[0] / 131.0f;
    float gyro_y_dps = gyro[1] / 131.0f;
    float gyro_z_dps = gyro[2] / 131.0f;
    
//        // 2. 加速度计计算 Roll/Pitch（弧度）
//    float roll_acc = atan2(acc_y_g, sqrt(acc_x_g * acc_x_g + acc_z_g * acc_z_g));
//    float pitch_acc = atan2(-acc_x_g, sqrt(acc_y_g * acc_y_g + acc_z_g * acc_z_g));
    *roll = atan2(acc_y_g, sqrt(acc_x_g * acc_x_g + acc_z_g * acc_z_g))* 180.0 / M_PI;
    *pitch = atan2(-acc_x_g, sqrt(acc_y_g * acc_y_g + acc_z_g * acc_z_g))* 180.0 / M_PI;    
        // 3. 陀螺仪积分更新姿态角
//    roll_dt += gyro_x_dps * dt;
//    pitch_dt += gyro_y_dps * dt;
        // 计算时间差（秒）
    float current_time = HAL_GetTick();
    float dt = (current_time - prev_time) / 1000.0; // 转换为秒
    prev_time = current_time;
    
    yaw_dt += gyro_z_dps * dt;

//    // 4. 互补滤波融合
//    *roll = alpha * roll_dt + (1 - alpha) * (roll_acc * 180.0 / M_PI);
//    *pitch = alpha * pitch_dt + (1 - alpha) * (pitch_acc * 180.0 / M_PI);

    // 5. 角度归一化
    if (yaw_dt > 360.0) yaw_dt -= 360.0;
    if (yaw_dt < 0.0) yaw_dt += 360.0;
    *yaw = yaw_dt;
//    // 计算角度 (弧度)
//    *pitch = atan2(ay, sqrt(ax * ax + az * az));
//    *roll  = atan2(-ax, sqrt(ay * ay + az * az));
//    
//    // 转换为角度
//    *pitch = *pitch * 180.0f / M_PI;
//    *roll  = *roll * 180.0f / M_PI;
}


