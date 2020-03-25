/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       mpu6500middleware.c/h
  * @brief      mpu6500磁力计中间层，完成mpu6500的通信函数,延时函数。
  *             
  * @note       IST8310只支持IIC读取
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#include "mpu6500driver_middleware.h"
#include "stm32f4xx.h"
#include "delay.h"

#if defined(MPU6500_USE_SPI)

#include "spi.h"

#elif defined(MPU6500_USE_IIC)

#endif

void mpu6500_GPIO_init(void)
{

#if defined(MPU6500_USE_SPI)

    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);

    //MPU NS
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOF, &GPIO_InitStructure);

#elif defined(MPU6500_USE_IIC)

#endif
}

void mpu6500_com_init(void)
{

#if defined(MPU6500_USE_SPI)
    SPI5Init();

#elif defined(MPU6500_USE_IIC)

#else

#error "Please select the communication of MPU6500"

#endif
}

void mpu6500_middleware_delay_ms(uint16_t ms)
{
    delay_ms(ms);
}
void mpu6500_middleware_delay_us(uint32_t us)
{
    delay_us(us);
}

#if defined(MPU6500_USE_SPI)

void mpu6500_SPI_NS_H(void)
{
    GPIO_SetBits(GPIOF, GPIO_Pin_6);
}
void mpu6500_SPI_NS_L(void)
{
    GPIO_ResetBits(GPIOF, GPIO_Pin_6);
}

static uint8_t mpu6500_SPI_read_write_byte(uint8_t TxData)
{
    uint8_t retry = 0;
    while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET)
    {
        retry++;
        if (retry > 200)
        {
            return 0;
        }
    }

    SPI_I2S_SendData(SPI5, TxData);

    retry = 0;

    while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_RXNE) == RESET)
    {
        retry++;
        if (retry > 200)
        {
            return 0;
        }
    }

    return SPI_I2S_ReceiveData(SPI5);
}
void mpu6500_write_single_reg(uint8_t reg, uint8_t data)
{
    mpu6500_SPI_NS_L();
    mpu6500_SPI_read_write_byte(reg);
    mpu6500_SPI_read_write_byte(data);
    mpu6500_SPI_NS_H();
}

uint8_t mpu6500_read_single_reg(uint8_t reg)
{
    uint8_t res;
    mpu6500_SPI_NS_L();
    mpu6500_SPI_read_write_byte(reg | MPU_SPI_READ_MSB);
    res = mpu6500_SPI_read_write_byte(0xFF);
    mpu6500_SPI_NS_H();
    return res;
}

void mpu6500_write_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    mpu6500_SPI_NS_L();
    mpu6500_SPI_read_write_byte(reg);
    if (len != 0)
    {
        uint8_t i;
        for (i = 0; i < len; i++)
        {
            mpu6500_SPI_read_write_byte(*buf);
            buf++;
        }
    }
    mpu6500_SPI_NS_H();
}

void mpu6500_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    mpu6500_SPI_NS_L();
    mpu6500_SPI_read_write_byte(reg | MPU_SPI_READ_MSB);
    if (len != 0)
    {
        uint8_t i;
        for (i = 0; i < len; i++)
        {
            *buf = mpu6500_SPI_read_write_byte(0xFF);
            buf++;
        }
    }
    mpu6500_SPI_NS_H();
}

#elif defined(MPU6500_USE_IIC)

#endif
