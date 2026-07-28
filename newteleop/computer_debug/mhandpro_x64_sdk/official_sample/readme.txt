# mHandPro Linux SDK Demo
支持操作系统	ubuntu 20.04/22.04 x64/arm64
指定编码格式	UTF-8

#常见问题：	
1.串口权限问题Error opening port: Permission denied
解决办法
sudo chown (user_name) /dev/(serial_name)
eg.sudo chown virdyn /dev/ttyXRUSB0

2025/01/09
1.Create and pass all test code.
2.Add serial driver.
3.Add arm64/x64 .so files.

2025/03/18
Add Magnetic calibration and postrue calibration.

2025/04/27
Add&check kylin v10 arm64 version .so file.
