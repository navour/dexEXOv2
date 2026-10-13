/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

//#include "myiic.h" 

#include "mpu6050.h"
//#include "usmart.h"   
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// DMA缓冲区：3通道 × 2轮 = 6 个 halfword，与DMA halfword对齐
#define ADC_CHANNELS    3
#define ADC_DMA_LEN     (ADC_CHANNELS * 2)   // 6
uint16_t ADC_DMA_Buffer[ADC_DMA_LEN];

uint8_t DMA_cov = 1;
uint8_t settle_count = 0;   // mux切换后的建立等待计数

uint16_t ADC_arrayBend[18];
uint16_t arrayBend[18];

/* 打印数组函数 */
uint16_t CBD_arrayData1[64];   // 组合数据缓冲区
uint16_t CBD_arrayData2[64];
uint16_t CBD_arrayData3[64];
uint16_t CBD_arrayData4[64];
uint16_t CBD_arrayData5[64];

char SPI_NSS_flag = 1;

/* 通信状态 */
#define TRIGGER_CMD 0x01
volatile uint8_t spiRxComplete = 0;
volatile uint8_t spiTxComplete = 0;
volatile uint8_t spiError = 0;

/* 数据缓冲区 */
uint8_t SPI_RxData[130];
uint8_t txTrigger = TRIGGER_CMD;

/* 高效打印缓冲区 */
uint8_t Bend_Buffer[640];

int16_t accel[3], gyro[3], angle[3];
float pitch, roll, yaw; 		//欧拉角
int16_t pitch_100, roll_100, yaw_100;

short aacx,aacy,aacz;		//加速度传感器原始数据
short gyrox,gyroy,gyroz;	//陀螺仪原始数据
short temp;					//温度	

const  char footer = 0x55;

int16_t SPI_time = 1000;
int16_t accel_base[3], gyro_base[3];

// 定义发送缓冲区
#define TX_BUF_SIZE sizeof(accel) + sizeof(gyro) + sizeof(pitch_100) + sizeof(roll_100) + sizeof(yaw_100)  + sizeof(arrayBend) + sizeof(Bend_Buffer) + 4
uint8_t txBuffer[TX_BUF_SIZE]; // 4字节头部 + 数据 + 2字节CRC(可选)

// 传感器数据结构体
typedef struct {
    uint16_t  BLE_accel[3];
    uint16_t  BLE_gyro[3];
    uint16_t  BLE_angle[3];
    uint16_t  BLE_arrayBend[18];
    uint16_t BLE_arrayA1[32], BLE_arrayB1[32];
    uint16_t BLE_arrayA2[32], BLE_arrayB2[32];
    uint16_t BLE_arrayA3[32], BLE_arrayB3[32];
    uint16_t BLE_arrayA4[32], BLE_arrayB4[32];
    uint16_t BLE_arrayA5[32], BLE_arrayB5[32];
} SensorData_t;

SensorData_t sensor_data;

// USART2 DMA发送完成标志
volatile uint8_t usart1TxComplete = 0;

// 记录命令与日志状态
static volatile bool log_active = false;
static uint32_t log_start_ms = 0;
static uint8_t uart_rx_byte = 0;
static char cmd_buf[16];
static uint8_t cmd_idx = 0;
static volatile bool cmd_ready = false;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        usart1TxComplete = 0;
        HAL_GPIO_TogglePin(TP_SW_GPIO_Port,TP_SW_Pin);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        char c = (char)uart_rx_byte;
        if (c == '\r' || c == '\n') {
            if (cmd_idx > 0) {
                cmd_buf[cmd_idx] = '\0';
                cmd_ready = true;
                cmd_idx = 0;
            }
        } else {
            if (cmd_idx < (sizeof(cmd_buf) - 1)) {
                cmd_buf[cmd_idx++] = c;
            }
        }
        HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
    }
}

/* SPI传输完成回调 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    if(hspi->Instance == SPI1) {
        spiTxComplete = 1;
    }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
    if(hspi->Instance == SPI1) {
        spiRxComplete = 1;
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
    if(hspi->Instance == SPI1) {
        spiError = 1;
    }
}


/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
//    static unsigned char ledState = 0;
    if (htim == (&htim2))
    {
//        HAL_GPIO_TogglePin(TP_SW_GPIO_Port,TP_SW_Pin);
    }
}
/* USER CODE END 4 */

/* USER CODE BEGIN PV */

void Set_Bend_Address(uint8_t addr){
    	switch(addr)
		{
			case 0:  //Bend_X1 0 0Bend_W3 1  Bend_S1 2  0X03
			{
                HAL_GPIO_WritePin(BCT_AB_GPIO_Port, BCT_AB_Pin, GPIO_PIN_SET);  
                HAL_GPIO_WritePin(BCT_BB_GPIO_Port, BCT_BB_Pin, GPIO_PIN_SET);  
                HAL_GPIO_WritePin(BCT_CB_GPIO_Port, BCT_CB_Pin, GPIO_PIN_RESET);
                break;
			}
			case 1:  //Bend_X2 3  Bend_WZC 4  Bend_S2 5  0X00
			{
				HAL_GPIO_WritePin(BCT_AB_GPIO_Port, BCT_AB_Pin, GPIO_PIN_RESET);  
                HAL_GPIO_WritePin(BCT_BB_GPIO_Port, BCT_BB_Pin, GPIO_PIN_RESET);  
                HAL_GPIO_WritePin(BCT_CB_GPIO_Port, BCT_CB_Pin, GPIO_PIN_RESET);
                break;
			}
            case 2:  //Bend_X3 6  Bend_Z1 7  Bend_S3 8  0X01
			{
                HAL_GPIO_WritePin(BCT_AB_GPIO_Port, BCT_AB_Pin, GPIO_PIN_SET);  
                HAL_GPIO_WritePin(BCT_BB_GPIO_Port, BCT_BB_Pin, GPIO_PIN_RESET);  
                HAL_GPIO_WritePin(BCT_CB_GPIO_Port, BCT_CB_Pin, GPIO_PIN_RESET);
                break;
			}
			case 3:  //Bend_XWC 9  Bend_Z2 10  Bend_SDC 11  0X02
			{
				HAL_GPIO_WritePin(BCT_AB_GPIO_Port, BCT_AB_Pin, GPIO_PIN_RESET);  
                HAL_GPIO_WritePin(BCT_BB_GPIO_Port, BCT_BB_Pin, GPIO_PIN_SET);  
                HAL_GPIO_WritePin(BCT_CB_GPIO_Port, BCT_CB_Pin, GPIO_PIN_RESET);
                break;
			}
            case 4:  //Bend_W1 12  Bend_Z3 13  Bend_D1 14  0X04
			{
                HAL_GPIO_WritePin(BCT_AB_GPIO_Port, BCT_AB_Pin, GPIO_PIN_RESET);  
                HAL_GPIO_WritePin(BCT_BB_GPIO_Port, BCT_BB_Pin, GPIO_PIN_RESET);  
                HAL_GPIO_WritePin(BCT_CB_GPIO_Port, BCT_CB_Pin, GPIO_PIN_SET);
                break;
			}
			case 5:  //Bend_W2 15  Bend_ZSC 16  Bend_D2 17  0X06
			{
				HAL_GPIO_WritePin(BCT_AB_GPIO_Port, BCT_AB_Pin, GPIO_PIN_RESET);  
                HAL_GPIO_WritePin(BCT_BB_GPIO_Port, BCT_BB_Pin, GPIO_PIN_SET);  
                HAL_GPIO_WritePin(BCT_CB_GPIO_Port, BCT_CB_Pin, GPIO_PIN_SET);
                break;
			} 
		}
}

// 扫描状态
typedef enum {
    SCAN_IDLE,
    SCAN_SETUP,
    SCAN_SETTLING,   // 等待切换后第一次DMA完成（丢弃旧数据）
    SCAN_WAIT_ADC,   // 等待第二次DMA完成（读取新数据）
    SCAN_COMPLETE
} ScanState_t;

volatile ScanState_t scanState = SCAN_SETUP;
uint8_t current_Bend = 0;

// 阻塞式轮询扫描：每个mux地址独立做一次ADC单次扫描
// 流程：禁用阵列 -> 切mux -> 使能阵列 -> 等建立 -> 启动ADC -> 逐通道读取 -> 禁用
void ProcessResistorScan(void) {
    for (int addr = 1; addr < 6; addr++) {
        // 1. 禁用阵列（断开模拟通路）
        HAL_GPIO_WritePin(BCT_ENB_GPIO_Port, BCT_ENB_Pin, GPIO_PIN_SET);
        
        // 2. 切换mux地址
        Set_Bend_Address(addr);
        
        // 3. 使能阵列
        HAL_GPIO_WritePin(BCT_ENB_GPIO_Port, BCT_ENB_Pin, GPIO_PIN_RESET);
        
        // 4. 等待模拟信号建立（mux开关 + 传感器RC时间常数）
        HAL_Delay(1);
        
        // 5. 启动ADC单次扫描（3通道依次转换后自动停止）
        HAL_ADC_Start(&hadc1);
        
        // 6. 逐通道轮询读取
        // Rank1 = CH1 (Bend_XW)
        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
            ADC_arrayBend[addr] = (uint16_t)HAL_ADC_GetValue(&hadc1);
        
        // Rank2 = CH2 (Bend_WZ)
        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
            ADC_arrayBend[addr + 6] = (uint16_t)HAL_ADC_GetValue(&hadc1);
        
        // Rank3 = CH14 (Bend_SD)
        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
            ADC_arrayBend[addr + 12] = (uint16_t)HAL_ADC_GetValue(&hadc1);
        
        HAL_ADC_Stop(&hadc1);
        
        // 7. 禁用阵列
        HAL_GPIO_WritePin(BCT_ENB_GPIO_Port, BCT_ENB_Pin, GPIO_PIN_SET); 
				
				HAL_Delay(2);
    }
    
    // 更新传感器数据
    for (int i = 0; i < 18; i++) {
        sensor_data.BLE_arrayBend[i] = ADC_arrayBend[i];
    }
}

// 设置NSS地址
void SetNSSAddress(uint8_t addr) {
  HAL_GPIO_WritePin(SPI_S1_GPIO_Port, SPI_S1_Pin, GPIO_PIN_SET); // S1
  HAL_GPIO_WritePin(SPI_S2_GPIO_Port, SPI_S2_Pin, GPIO_PIN_SET); // S2
  HAL_GPIO_WritePin(SPI_S3_GPIO_Port, SPI_S3_Pin, GPIO_PIN_SET); // S3
  HAL_GPIO_WritePin(SPI_S4_GPIO_Port, SPI_S4_Pin, GPIO_PIN_SET); // S4
  HAL_GPIO_WritePin(SPI_S5_GPIO_Port, SPI_S5_Pin, GPIO_PIN_SET); // S5
  switch(addr)
  {
       case 1:
       {
          HAL_GPIO_WritePin(SPI_S1_GPIO_Port, SPI_S1_Pin, GPIO_PIN_RESET); // S1
          break;
       }
       case 2:
       {
          HAL_GPIO_WritePin(SPI_S2_GPIO_Port, SPI_S2_Pin, GPIO_PIN_RESET); // S2
          break;
       }
       case 3:
       {
          HAL_GPIO_WritePin(SPI_S3_GPIO_Port, SPI_S3_Pin, GPIO_PIN_RESET); // S3
          break;
       }
       case 4:
       {
          HAL_GPIO_WritePin(SPI_S4_GPIO_Port, SPI_S4_Pin, GPIO_PIN_RESET); // S4
          break;
       }
       case 5:
       {
          HAL_GPIO_WritePin(SPI_S5_GPIO_Port, SPI_S5_Pin, GPIO_PIN_RESET); // S5
          break;
       }
       default:
          break;
  }
}

void SPI_Communication(void)
{

            SetNSSAddress(SPI_NSS_flag);
            HAL_Delay(100);
            memset(SPI_RxData, 0, sizeof(SPI_RxData));
            HAL_StatusTypeDef status = HAL_SPI_Receive(&hspi1, SPI_RxData, 130, 100);
            
            if (status == HAL_OK) {
                if(SPI_RxData[0] == 0xA1 && SPI_RxData[129] == 0x1A) {
                    memcpy(sensor_data.BLE_arrayA1, SPI_RxData + 1, 64);
                    memcpy(sensor_data.BLE_arrayB1, SPI_RxData + 65, 64);
                }
                if(SPI_RxData[0] == 0xA2 && SPI_RxData[129] == 0x2A) {
                    memcpy(sensor_data.BLE_arrayA2, SPI_RxData + 1, 64);
                    memcpy(sensor_data.BLE_arrayB2, SPI_RxData + 65, 64);
                }
                if(SPI_RxData[0] == 0xA3 && SPI_RxData[129] == 0x3A) {
                    memcpy(sensor_data.BLE_arrayA3, SPI_RxData + 1, 64);
                    memcpy(sensor_data.BLE_arrayB3, SPI_RxData + 65, 64);
                }
                if(SPI_RxData[0] == 0xA4 && SPI_RxData[129] == 0x4A) {
                    memcpy(sensor_data.BLE_arrayA4, SPI_RxData + 1, 64);
                    memcpy(sensor_data.BLE_arrayB4, SPI_RxData + 65, 64);
                }
                if(SPI_RxData[0] == 0xA5 && SPI_RxData[129] == 0x5A) {
                    memcpy(sensor_data.BLE_arrayA5, SPI_RxData + 1, 64);
                    memcpy(sensor_data.BLE_arrayB5, SPI_RxData + 65, 64);
                }
                SPI_NSS_flag ++;
                SetNSSAddress(0);
            }

            if(SPI_NSS_flag == 6)SPI_NSS_flag = 1;

            
            if(spiError) {
                const char *errorMsg = "SPI Error! Reinitializing...\r\n";
                HAL_UART_Transmit(&huart2, (uint8_t*)errorMsg, strlen(errorMsg), 100);
                
                HAL_SPI_DeInit(&hspi1);
                MX_SPI1_Init();
                spiError = 0;
            }
}

#define JSON_BUFFER_SIZE 2048
static char json_buffer[JSON_BUFFER_SIZE];

// 辅助函数：将数组转换为JSON数组字符串
static int array_to_json(char* buffer, uint16_t* array, int size) {
    int pos = 0;
    pos += sprintf(buffer + pos, "[");
    
    for(int i = 0; i < size; i++) {
        pos += sprintf(buffer + pos, "%u", array[i]);
        if(i < size - 1) {
            pos += sprintf(buffer + pos, ",");
        }
    }
    pos += sprintf(buffer + pos, "]");
    return pos;
}

// 将ADC值转换为压力(N)
// V = AD/4096*3.3
// R = (45.3*V + 7.0215) / (4.795 - V)   (R单位: KΩ)
// 分段线性：使用商家标定点(1/R 与 F)
static const float FORCE_F_KG_TABLE[10] = {
    0.5f, 1.0f, 1.5f, 2.0f, 2.5f, 3.0f, 3.5f, 4.0f, 4.5f, 5.0f
};
static const float INV_R_TABLE[10] = {
    0.011111f, 0.022222f, 0.038462f, 0.058824f, 0.083333f,
    0.111111f, 0.136986f, 0.166667f, 0.192308f, 0.222222f
};

static void ad_to_params(uint16_t ad, float* v, float* r, float* inv_r)
{
    float vv = (float)ad * 3.3f / 4096.0f;
    // 避免分母为0或负值
    if (vv >= 4.795f) {
        vv = 4.794f;
    }

    float rr = (45.3f * vv + 7.0215f) / (4.795f - vv); // KΩ
    float inv = 0.0f;
    if (rr > 0.0f) {
        inv = 1.0f / rr;
    }

    if (v) {
        *v = vv;
    }
    if (r) {
        *r = rr;
    }
    if (inv_r) {
        *inv_r = inv;
    }
}

static float force_from_inv_r(float inv_r)
{
    // 分段线性插值计算F(kg)
    float f_kg_raw = FORCE_F_KG_TABLE[0];
    if (inv_r <= INV_R_TABLE[0]) {
        f_kg_raw = FORCE_F_KG_TABLE[0];
    } else if (inv_r >= INV_R_TABLE[9]) {
        f_kg_raw = FORCE_F_KG_TABLE[9];
    } else {
        for (int i = 0; i < 9; i++) {
            float x0 = INV_R_TABLE[i];
            float x1 = INV_R_TABLE[i + 1];
            if (inv_r >= x0 && inv_r <= x1) {
                float y0 = FORCE_F_KG_TABLE[i];
                float y1 = FORCE_F_KG_TABLE[i + 1];
                float t = (inv_r - x0) / (x1 - x0);
                f_kg_raw = y0 + t * (y1 - y0);
                break;
            }
        }
    }

    float f_kg = f_kg_raw;
    float f = f_kg * 9.80665f; // N
    if (f < 0.0f) {
        f = 0.0f;
    }
    return f;
}

// 串扰补偿系数（0.0=不补偿，调高直到相邻通道不再联动为止）
// 校准方法：单独按压 f[0]，记录此时 f[1] 的力值，
//           CROSSTALK_K = f[1]_measured / f[0]_measured
#define CROSSTALK_K  0.0f   // 初始设为0，用户需自行校准后填入

// 计算单个ADC值对应的力(N)
static float compute_force(uint16_t ad)
{
    float inv_r = 0.0f;
    ad_to_params(ad, NULL, NULL, &inv_r);
    return force_from_inv_r(inv_r);
}

// 发送传感器数据: 通道0-12原始值，通道13-17经力值计算后输出，格式 {ch1,...,ch18}

void send_sensor_data_json(SensorData_t* data) {
    int pos = 0;
    pos += sprintf(json_buffer + pos, "{");
    // 通道 0-12：输出原始ADC值
    for (int i = 0; i < 13; i++) {
        pos += sprintf(json_buffer + pos, "%u,", data->BLE_arrayBend[i]);
    }
    // 通道 13-17：经过力值计算后输出(N)
    for (int i = 13; i < 18; i++) {
        float f = compute_force(data->BLE_arrayBend[i]);
        if (f < 0.0f) f = 0.0f;
        if (i < 17) {
            pos += sprintf(json_buffer + pos, "%.2f,", f);
        } else {
            pos += sprintf(json_buffer + pos, "%.2f", f);
        }
    }
    pos += sprintf(json_buffer + pos, "}\n");

    // 通过DMA发送（非阻塞）
    HAL_UART_Transmit_DMA(&huart2, (uint8_t*)json_buffer, pos);
    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)json_buffer, pos);
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    

  char IMU_valuesstr[1000];
//    float BAT_VEL = 0;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
    HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
//  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

//    HAL_SPI_TransmitReceive(&hspi1, rxData, rxData, 1, 10);
    
    MPU_Init();						//初始化MPU6050
    while(mpu_dmp_init());
     
    HAL_GPIO_WritePin(BAT_ADC_SW_GPIO_Port, BAT_ADC_SW_Pin, GPIO_PIN_SET);//读取电池电压开关
  
    HAL_GPIO_WritePin(POWER_SW_GPIO_Port, POWER_SW_Pin, GPIO_PIN_SET);//外设电源开关
    
    // 改为单次扫描模式（非连续），每次切mux后精确控制ADC采样
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    HAL_ADC_Init(&hadc1);
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

    SetNSSAddress(0);
//    HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//      BAT_VEL = (float)ADC_DMA_Buffer[3]*3.02*2.034/4096;  //电池电压
//      sprintf(BAT_valuesstr,"%.2f \r\n",BAT_VEL);
//      HAL_UART_Transmit(&huart2, (uint8_t *)BAT_valuesstr, 20, 0xffff);


//        if(mpu_dmp_get_data(&pitch,&roll,&yaw) == 0)
//        {
////            HAL_UART_Transmit(&huart1, (uint8_t *)IMU_valuesstr, sizeof(IMU_valuesstr), 0xffff);
////            sprintf(IMU_valuesstr,"tt: %d\r\n",
////                                mpu_dmp_get_data(&pitch,&roll,&yaw));
//            if(pitch < 0)sensor_data.BLE_angle[0] = (pitch+360)*100 ;
//            else sensor_data.BLE_angle[0] = pitch*100 ;
//            if(roll  < 0)sensor_data.BLE_angle[1] = (roll +360)*100 ;
//            else sensor_data.BLE_angle[1]  = roll*100;
//            if(yaw   < 0)sensor_data.BLE_angle[2] = (yaw  +360)*100 ;
//            else sensor_data.BLE_angle[2]   = yaw*100;
//        }
////        temp=MPU_Get_Temperature();	//得到温度值
//        MPU_Get_Accelerometer(&sensor_data.BLE_accel[0],&sensor_data.BLE_accel[1],&sensor_data.BLE_accel[2]);	//得到加速度传感器数据
//        MPU_Get_Gyroscope(&sensor_data.BLE_gyro[0],&sensor_data.BLE_gyro[1],&sensor_data.BLE_gyro[2]);	//得到陀螺仪数据

//        sprintf(IMU_valuesstr,"tt:temp：%d aacx:%d aacy:%d aacz:%d gyrox:%d gyroy:%d gyroz:%d pitch:%.2f roll:%.2f yaw:%.2f \r\n",
//                                temp, aacx, aacy, aacz, gyrox, gyroy, gyroz, pitch, roll, yaw);

        if (cmd_ready) {
            cmd_ready = false;
            if (strcmp(cmd_buf, "record") == 0) {
                log_active = true;
                log_start_ms = HAL_GetTick();
                const char *hdr = "time_ms,ad\r\n";
                HAL_UART_Transmit(&huart2, (uint8_t*)hdr, strlen(hdr), 100);
            } else if (strcmp(cmd_buf, "over") == 0) {
                log_active = false;
                const char *end = "OVER\r\n";
                HAL_UART_Transmit(&huart2, (uint8_t*)end, strlen(end), 100);
            }
        }

        if (log_active) {
            uint32_t t_ms = HAL_GetTick() - log_start_ms;
            uint16_t ad = sensor_data.BLE_arrayBend[17];
            char line[32];
            int n = snprintf(line, sizeof(line), "%lu,%u\r\n", (unsigned long)t_ms, ad);
            HAL_UART_Transmit(&huart2, (uint8_t*)line, n, 100);
        } else {
            send_sensor_data_json(&sensor_data);
        }

        ProcessResistorScan();
        
//        SPI_Communication();
//        
//        if(usart1TxComplete == 0)
//        {
////            Send_Combined_Data();
//            usart1TxComplete = 1;
//        }
        

  }
 
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 28;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

///* USER CODE BEGIN 4 */
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
////    static unsigned char ledState = 0;
//    if (htim == (&htim2))
//    {
////        HAL_GPIO_TogglePin(TP_SW_GPIO_Port,TP_SW_Pin);
//    }
//}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */