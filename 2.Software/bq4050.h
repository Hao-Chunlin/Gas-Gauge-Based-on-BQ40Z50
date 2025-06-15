#ifndef __BQ4050_H
#define __BQ4050_H

#define I2C_SCL_GPIO_Port GPIOA
#define I2C_SDA_GPIO_Port GPIOA
#define I2C_SCL_Pin GPIO_Pin_0
#define I2C_SDA_Pin GPIO_Pin_1

#define READ_SDA GPIO_ReadInputDataBit(I2C_SDA_GPIO_Port, I2C_SDA_Pin)
#define SCL_H GPIO_WriteBit(I2C_SCL_GPIO_Port, I2C_SCL_Pin, Bit_SET)
#define SCL_L GPIO_WriteBit(I2C_SCL_GPIO_Port, I2C_SCL_Pin, Bit_RESET)
#define SDA_H GPIO_WriteBit(I2C_SDA_GPIO_Port, I2C_SDA_Pin, Bit_SET)
#define SDA_L GPIO_WriteBit(I2C_SDA_GPIO_Port, I2C_SDA_Pin, Bit_RESET)

void BQ4050_GPIO_Init(void);
void SDA_OUT(void);
void SDA_IN(void);

#define BQ4050_ADDRRESS 0x16

struct bq4050_type {
  uint16_t manufacturer_num;
  uint16_t absolute_SOC;
  uint16_t relative_SOC;
  float temperature;
  uint16_t voltage;
  int16_t current;
  int16_t power;
  uint16_t cell_voltage_arr[4];
  int16_t cell_current_arr[4];
  int16_t temperature_arr[7];
};
extern struct bq4050_type bq4050;
uint8_t BQ4050_Read_Info(void);
uint8_t SMBus_Read_Word(uint8_t slave_add, uint8_t cmd, uint16_t* dat);
#endif
