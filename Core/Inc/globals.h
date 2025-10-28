#ifndef GLOBALS_H
#define GLOBALS_H

#include <stdint.h>

#define SMP_LEN  1024
/* MPU6050 I2C设备地址定义 */
#define MPU6050_ADDR    (0x68 << 1)  // 若AD0接地，7位地址0x68，左移1位得到8位地址0xD0
#define WHO_AM_I_REG    0x75        // WHO_AM_I寄存器地址，默认值0x68
#define PWR_MGMT_1_REG  0x6B        // 电源管理寄存器1
#define SMPLRT_DIV_REG  0x19        // 采样率分频寄存器
#define CONFIG_REG      0x1A        // 配置寄存器（含DLPF设置）
#define GYRO_CONFIG_REG 0x1B        // 陀螺仪配置寄存器
#define ACCEL_CONFIG_REG 0x1C       // 加速度计配置寄存器
 




extern int16_t   rawBuf[2][SMP_LEN];
extern uint8_t   bufIdx;
extern uint8_t   bufFull;
extern float     rmsBuf[2];
#endif
