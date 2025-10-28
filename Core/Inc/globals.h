#ifndef GLOBALS_H
#define GLOBALS_H

#include <stdint.h>

#define SMP_LEN  1024
/* MPU6050 I2C�豸��ַ���� */
#define MPU6050_ADDR    (0x68 << 1)  // ��AD0�ӵأ�7λ��ַ0x68������1λ�õ�8λ��ַ0xD0
#define WHO_AM_I_REG    0x75        // WHO_AM_I�Ĵ�����ַ��Ĭ��ֵ0x68
#define PWR_MGMT_1_REG  0x6B        // ��Դ����Ĵ���1
#define SMPLRT_DIV_REG  0x19        // �����ʷ�Ƶ�Ĵ���
#define CONFIG_REG      0x1A        // ���üĴ�������DLPF���ã�
#define GYRO_CONFIG_REG 0x1B        // ���������üĴ���
#define ACCEL_CONFIG_REG 0x1C       // ���ٶȼ����üĴ���
 




extern int16_t   rawBuf[2][SMP_LEN];
extern uint8_t   bufIdx;
extern uint8_t   bufFull;
extern float     rmsBuf[2];
#endif
