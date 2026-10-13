# STM32G431CBT6-60422（18个通道打印）源码检查

日期：2026-10-10。用户指定此目录为最新固件。检查范围：Core 主程序、时钟、ADC、GPIO、UART/DMA、中断、MPU6050驱动、启动文件、Keil工程及随附编译记录。
未修改固件、未编译生成新HEX、未烧录、未连接硬件。当前环境未发现 arm-none-eabi-gcc、armclang 或 clang。
源码路径前缀：`STM32G431CBT6-60422（18个通道打印）/STM32G431CBT6-60422（18个通道打印）/`。

## 更新先前判断

此版 main.c:490 输出前13项ADC、后5项力值的18项紧凑帧，符合当前上位机输入格式。
此前关于0119源码是单通道JSON、与实板格式不同的说明只适用于0119旧目录，不应再用于否定此新版。
主时钟故障和IMU初始化阻塞这两条排查分支仍成立；源码不能证明实板当前运行的二进制与目录一致。

## 高优先级：与无输出或异常运行有关

### 1. 系统时钟、电压档位、Flash等待周期不匹配（确定）

- Core/Src/main.c:669 使用 PWR_REGULATOR_VOLTAGE_SCALE1（非BOOST）。
- main.c:674–697：HSE=24MHz，PLLM=2、PLLN=28、PLLR=2，AHB=1，得到168MHz；Flash仅FLASH_LATENCY_3。
- 随附HAL Src/stm32g4xx_hal_pwr_ex.c:135–147规定普通Range1至150MHz，BOOST至170MHz。
- 随附HAL Src/stm32g4xx_hal_rcc.c:184–199表格规定普通Range1的3WS至120MHz；BOOST下168MHz需要4WS。

后果：已超出规定运行条件，可能HardFault、卡死、温度/板间差异，不能将不稳定直接归因于晶体损坏。
修复方向：若保持168MHz，需要BOOST和4WS，并按ST流程配置；此低速采集应用也可回到工程.ioc原来的96MHz、普通Range1、3WS。不要只改Flash而遗漏电压档位。
.ioc:374/384仍记载PLLN=16和96MHz，与main.c实际不一致；重新生成工程可能覆盖手工更改。

### 2. ADC时钟84MHz，超过规格（确定）

Core/Src/adc.c:48使用ADC_CLOCK_SYNC_PCLK_DIV2。该宏实际代表AHB同步时钟/2；168/2=84MHz。
STM32G431数据手册Table60规定Range1单ADC最大60MHz。
修复方向：168MHz下至少改/4得到42MHz；若系统改96MHz，/2为48MHz。
这通常影响ADC可靠性，不能独立证明BLE零通知的原因。

### 3. 压力采集被不再使用的IMU无限重试阻塞（确定）

main.c:569–570：MPU_Init(); while(mpu_dmp_init());
压力电源POWER_SW置高在574行，发送在639行。IMU通信、DMP装载或自检持续失败，就到不了压力发送。
主循环的IMU姿态读取已经注释，压力用途仍被IMU启动绑住。
修复方向：FSR专用固件禁用IMU依赖；需要IMU时用有限次数/有限时长初始化，保留错误码，压力采集独立继续。诊断时先观察返回码，不直接判断IMU已坏。

另有初始化返回值判断缺陷：Core/MPU6050/eMPL/inv_mpu.c:871成功返回122，失败返回负数；2956行却用if(mpu_init())，成功与失败都为真。不能简单改为==0而不同时统一底层返回约定。

### 4. PB4被两个功能争用（确定）

Core/Src/usart.c:205–213先配置PB4=USART2_RX；随后Core/MPU6050/mpu6050.c:43–50把PB4改成推挽输出低，用作MPU_AD0控制。
用户原理图也把RXD-MCU和MPU_AD0放在同一PB4网络。
后果：USART2接收record/over命令无法按预期工作；外部串口TX连接该点时可能与低电平输出争用。
这不是USART1/PB6蓝牙发送线的直接冲突。需要按实物确认AD0连接，明确PB4单一用途；不能仅把它重新配置为UART就假定IMU地址仍正确。

## 数据正确性与健壮性

### 5. 宣称18通道，实际只扫描15通道（确定）

main.c:262：for (int addr = 1; addr < 6; addr++)。
写入索引addr、addr+6、addr+12，因此0、6、12三个槽从未更新，静态初始化为0。
若要完整18通道应从0开始扫描，之后逐路核对顺序。
后5项13..17均在现有扫描范围，不能用漏扫这三路解释五路FSR完全无通知。

### 6. 首帧未采样，ADC错误会伪装成有效力值（确定）

main.c:639先发送，642才采样；首帧五个FSR ADC全为0，现有换算输出每指49.03N（满量程）。
ADC Start、校准及重新Init返回值未检查；轮询失败不更新对应槽，而旧值仍作为新帧发布。
因此新鲜BLE帧不保证采样新鲜，上位机超时检测不能发现这种陈旧测量。
修复方向：先完成有效整帧采样再发布；记录各阶段错误，失败时采用与上位机共同约定的失效处理，不能将0或旧值冒充有效测量。
3通道顺序转换、逐个轮询、无AutoWait，在CPU未及时读取时还有OVR/通道错配风险，需检查OVR并考虑单次3项DMA或不连续逐通道触发。

### 7. UART DMA缺少返回值和缓冲区生命周期保护（确定的设计缺口）

main.c:490–511每次重写同一个json_buffer，再交给USART1/2 DMA；不检查HAL_BUSY/HAL_ERROR，也不确认两路均已发送完成。
当前约100字节帧在921600下约1ms，扫描至少包含15ms延时，因此正常节奏通常有足够间隔；不能声称必然发生覆盖或以此认定当前根因。
应使用忙状态/完成计数与不可修改的发送缓冲区，记录错误并设计恢复路径。
已核对dma.c、usart.c及stm32g4xx_it.c：两路DMA和USART中断均配置且调用HAL处理器，没有发现“完全漏配发送完成中断”。

### 8. record模式会主动停止蓝牙传感器发送（条件分支）

main.c:620–639收到USART2的record后，只向USART2打印，USART1不再发送传感器帧，直到over。
完全断电后log_active重置false，因此它不能单独解释当前断电重启仍无数据；且PB4冲突使此接收路径本身有问题。

## 现场定位顺序

1. 先通过ST-Link不下载连接，核对实板RCC/FLASH/PWR寄存器与源码，查看程序PC位置。
2. 若卡在时钟初始化/Error_Handler：记录HSEON/HSERDY/PLLRDY、RCC_CFGR.SWS、PWR_CR5.R1MODE、FLASH_ACR.LATENCY。Error_Handler有多个入口，不能仅凭函数名认定晶振。
3. 若反复进入mpu_dmp_init：记录mpu_init及各阶段返回值，核对MPU供电、I2C ACK/时序。
4. 若到达发送函数：查看huart1.gState、ErrorCode、DMA计数和返回状态，同时测R44两端波形。
5. 若UART已有正确18项帧再查BLE模块透传参数。

## 验证边界与参考

源码核算确认：SYSCLK=168MHz、ADC=84MHz、漏采索引0/6/12、ADC=0输出49.03325N。
随附旧编译日志为0错误10警告；这不是本次构建验证。随附调用图main局部栈仅40字节，未使用的IMU_valuesstr被优化掉，不能凭源码的1000字节数组直接认定当前二进制栈溢出。
硬件根因仍待ST-Link或波形确认；相同代码在左板工作不代表上述超规格配置合规，也不证明右板故障一定由软件引起。

ST官方数据手册：https://www.st.com/resource/en/datasheet/stm32g431cb.pdf
