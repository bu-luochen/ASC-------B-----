#include "stm32f10x.h"                  // Device header

#include "MPU6050_Reg.h"

#define MPU6050_ADDRESS				0xD0


void MPU6050_WaitEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT)
{
	uint32_t Timeout = 10000;
	while(I2C_CheckEvent(I2Cx, I2C_EVENT) != SUCCESS)
	{
		Timeout --;
		if(Timeout == 0){
			break;
		}
	}
}


void MPU6050_WriteReg(uint8_t RegAddresss,uint8_t Data)//指定寄存器位置写入数据
{
	
	
	I2C_GenerateSTART(I2C2,ENABLE);
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);
	
	I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Transmitter);
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
	
	I2C_SendData(I2C2,RegAddresss);
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTING);
	
	I2C_SendData(I2C2,Data);
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
	
	I2C_GenerateSTOP(I2C2, ENABLE);
//	MyI2C_Start();
//	MyI2C_SendByte(MPU6050_ADDRESS);
//	MyI2C_ReceiveAck();
//	MyI2C_SendByte(RegAddresss);
//	MyI2C_ReceiveAck();
//	MyI2C_SendByte(Data);
//	MyI2C_ReceiveAck();
//	MyI2C_Stop();
}

uint8_t MPU6050_ReadReg(uint8_t RegAddresss)//指定寄存器位置写入数据
{
	uint8_t Data;
	
	
	I2C_GenerateSTART(I2C2,ENABLE);
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);
	
	I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Transmitter);
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
	
	I2C_SendData(I2C2,RegAddresss);
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
	
	I2C_GenerateSTART(I2C2,ENABLE);
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);
	
	I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Receiver);
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);
	
	I2C_AcknowledgeConfig(I2C2, DISABLE);
	I2C_GenerateSTOP(I2C2, ENABLE);
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);
	
	Data = I2C_ReceiveData(I2C2);
	
	I2C_AcknowledgeConfig(I2C2, ENABLE);
//	MyI2C_Start();
//	MyI2C_SendByte(MPU6050_ADDRESS);
//	MyI2C_ReceiveAck();
//	MyI2C_SendByte(RegAddresss);
//	MyI2C_ReceiveAck();
//	//设置当前地址指针
//	MyI2C_Start();
//	MyI2C_SendByte(MPU6050_ADDRESS | 0x01);//读取寄存器
//	MyI2C_ReceiveAck();
//	Data = MyI2C_ReceiveByte();
//	MyI2C_SendAck(1);//不给从机应答
//	MyI2C_Stop();
	
	return Data;
}



void MPU6050_Init(void)
{
//	MyI2C_Init();
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	GPIO_SetBits(GPIOB,GPIO_Pin_11|GPIO_Pin_0);
	
	I2C_InitTypeDef I2C_InitStructure;
	I2C_InitStructure.I2C_Mode=I2C_Mode_I2C;
	I2C_InitStructure.I2C_ClockSpeed=50000;
	I2C_InitStructure.I2C_DutyCycle=I2C_DutyCycle_2;//占空比
	I2C_InitStructure.I2C_Ack=I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress=I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_OwnAddress1=0x00;//stm32作为从机
	I2C_Init(I2C2,&I2C_InitStructure);
	
	I2C_Cmd(I2C2,ENABLE);
	
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1,0x01);//选x轴的时钟，同时解除睡眠
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2,0x00);//6个轴均不待机
	
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV,0x09);//采样率分频，10分频
	MPU6050_WriteReg(MPU6050_CONFIG,0x06);//不需要外部同步，数字滤波选择
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG,0x18);//陀螺仪配置，自测使不能，满量程选择（最大）
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG,0x18);//加速度计配置，自测使不能，满量程选择（最大），高通滤波器（不使用）
	
	
}

uint8_t MPU6050_GetID(void)
{
	return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

void MPU6050_GetData(int16_t *AccX,int16_t *AccY,int16_t *AccZ,int16_t *GyroX,int16_t *GyroY,int16_t *GyroZ)
{
	uint8_t DataH,DataL;
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);//高八位
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);
	*AccX = (DataH << 8) | DataL; 
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
	*AccY = (DataH << 8) | DataL; 
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
	*AccZ = (DataH << 8) | DataL; 
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
	*GyroX = (DataH << 8) | DataL; 
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
	*GyroY = (DataH << 8) | DataL; 
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
	*GyroZ = (DataH << 8) | DataL; 
	
	
}
typedef struct{
	
	int32_t sumAX;
	int32_t sumAY;
	int32_t sumAZ;
	int32_t sumGX;
	int32_t sumGY;
	int32_t sumGZ;
}MPU6050_DataInitDef;

uint8_t MPU6050_DataInit(int16_t *AccX,int16_t *AccY,int16_t *AccZ,int16_t *GyroX,int16_t *GyroY,int16_t *GyroZ,MPU6050_DataInitDef *Result)
{
	static uint8_t k,flag = 0;
	if(flag){
		return 1;
	} else {
		if(k <= 49){
			k ++;
			Result -> sumAX  += *AccX;
			Result -> sumAY += *AccY;
			Result -> sumAZ += *AccZ;
			Result -> sumGX += *GyroX;
			Result -> sumGY += *GyroY;
			Result -> sumGZ += *GyroZ;
			return 0;
		} else {
			Result -> sumAX /= k;
			Result -> sumAY /= k;
			Result -> sumAZ /= k;
			Result -> sumGX /= k;
			Result -> sumGY /= k;
			Result -> sumGZ /= k;
			flag = 1;
			return 1;
		}
	}
	
}

