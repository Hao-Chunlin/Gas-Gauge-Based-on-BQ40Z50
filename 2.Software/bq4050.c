#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include "bq4050.h"
#include "delay.h"
#include "string.h"
#include "sys.h"

struct bq4050_type bq4050 = {0};

void BQ4050_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

  GPIO_InitStructure.GPIO_Pin = I2C_SCL_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_SetBits(GPIOA, I2C_SCL_Pin);

  GPIO_InitStructure.GPIO_Pin = I2C_SDA_Pin;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_SetBits(GPIOA, I2C_SDA_Pin);

  GPIO_SetBits(I2C_SCL_GPIO_Port, I2C_SCL_Pin);
  GPIO_SetBits(I2C_SDA_GPIO_Port, I2C_SDA_Pin);
}

void SDA_OUT(void) {
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = I2C_SDA_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(I2C_SDA_GPIO_Port, &GPIO_InitStructure);
}

void SDA_IN(void) {
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = I2C_SDA_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(I2C_SDA_GPIO_Port, &GPIO_InitStructure);
}
////////////////////////////////////////////////////////////////////////////////////
// ����IIC��ʼ�ź�
void SMBus_Start() {
  delay_us(50);
  SDA_OUT();
  SCL_H;
  SDA_H;
  delay_us(10);
  SDA_L;
  delay_us(10);
  SCL_L;
  delay_us(5);
}

// ����IICֹͣ�ź�
void SMBus_Stop() {
  delay_us(50);
  SDA_OUT();
  SCL_H;
  SDA_L;
  delay_us(10);
  SDA_H;
  delay_us(10);
}

// ����ACKӦ��
void SMBus_Ack() {
  SDA_OUT();
  SCL_L;
  SDA_L;
  delay_us(10);
  SCL_H;
  delay_us(10);
  SCL_L;
  delay_us(5);
}

// ������ACKӦ��
void SMBus_NAck() {
  SDA_OUT();
  SCL_L;
  SDA_H;
  delay_us(10);
  SCL_H;
  delay_us(10);
  SCL_L;
  delay_us(5);
}

// �ȴ�Ӧ���źŵ���,����ֵ:1��ʾAck,0��ʾNAck
uint8_t SMBus_Wait_Ack() {
  uint16_t timeout = 2000;
  SDA_OUT();
  SCL_L;
  SDA_H;
  SDA_IN();  // SDA����Ϊ����
  delay_us(50);
  while (READ_SDA) {
    timeout--;
    if (timeout == 0) {
      SMBus_Stop();
      return 1;
    }
    delay_us(1);
  }
  SCL_H;  // �ڴӻ�Ӧ���ڼ�,����һ��SCL�ߵ�ƽ�ź�
  delay_us(10);
  SCL_L;
  delay_us(10);
  return 0;
}

// ������дһ���ֽ�����
void SMBus_Write(uint8_t dat) {
  SDA_OUT();
  for (uint8_t i = 0; i < 8; i++) {
    if ((dat & 0x80) >> 7)
      SDA_H;
    else
      SDA_L;
    dat <<= 1;
    delay_us(10);
    SCL_H;  // ��ʱ����Ϊ��,֪ͨ��������ʼ��������λ
    delay_us(10);
    SCL_L;
    delay_us(10);
  }
  delay_us(5);
}

// ��1���ֽ�,ack=1ʱ,����ACK,ack=0,����nACK
uint8_t SMBus_Read() {
  uint8_t receive = 0;
  SDA_IN();  // SDA����Ϊ����
  SCL_L;
  delay_us(300);  // ��ȡǰҪ���㹻������ʱ,��Ȼ���׳���
  for (uint8_t i = 0; i < 8; i++) {
    delay_us(20);
    SCL_H;
    receive <<= 1;
    if (READ_SDA)
      receive++;
    delay_us(10);
    SCL_L;
  }
  delay_us(5);
  return receive;
}
////////////////////////////////////////////////////////////////////////////////////
// CRCУ�麯��
uint8_t calculate_crc8(uint8_t* message, uint8_t len)  // CRC-8У�� X8+X2+X+1
{
  uint8_t crc = 0;
  while (len--) {
    crc ^= (*message++);
    for (uint8_t i = 0; i < 8; i++) {
      if (crc & 0x80)
        crc = (crc << 1) ^ 0x07;
      else
        crc <<= 1;
    }
  }
  return crc;
}

/***
 * @brief SMBusд����ָ��,����0��ʾ����
 * @param slave_add��ʾ�ӻ���ַ,��ǰ7λ,���1λ��Ч
 * @param cmd��ʾд��ļĴ�����ַ
 * @param dat��ʾҪд������ݵ�ָ��
 * @param dat_len��ʾд�����ݵĳ���
 */
uint8_t SMBus_Write_Block(uint8_t slave_add,
                          uint8_t cmd,
                          uint8_t* dat,
                          uint8_t dat_len) {
  if (dat_len > 64)
    return 0;
  slave_add &= 0xFE;  // ֻ����ǰ7λ��ַ
  SMBus_Start();
  SMBus_Write(slave_add);
  if (SMBus_Wait_Ack())
    return 1;

  SMBus_Write(cmd);
  if (SMBus_Wait_Ack())
    return 2;

  SMBus_Write(dat_len);
  if (SMBus_Wait_Ack())
    return 3;

  // ��4λ��ʼΪblock����
  for (uint8_t i = 0; i < dat_len; i++) {
    SMBus_Write(dat[i]);
    if (SMBus_Wait_Ack())
      return 4;
  }

  SMBus_Stop();
  return 0;
}

/***
 * @brief SMBus������ָ��,����ʵ��block���ݳ���,����0��ʾ����
 * @param slave_add��ʾ�ӻ���ַ,��ǰ7λ,���1λ��Ч
 * @param cmd��ʾд��ļĴ�����ַ
 * @param dat��ʾ���ص�����Ҫ���浽�ı�����ָ��
 * @param dat_len��ʾ��ȡ����(���64�ֽ�)
 */
uint8_t SMBus_Read_Block(uint8_t slave_add,
                         uint8_t cmd,
                         uint8_t* dat,
                         uint8_t dat_len) {
  if (dat_len > 64)
    return 0;
  slave_add &= 0xFE;  // ֻ����ǰ7λ��ַ
  // ���洫���ڼ���������,���ʽ����:
  // �ӻ�д��ַ+����+�ӻ�����ַ+�������ݳ���+дblockʱд��ĵ�ַ(2�ֽ�)+����(���64�ֽ�)
  uint8_t message[70] = {0};
  uint8_t pec = 0;  // У���ֽ�
  message[0] = slave_add;
  message[1] = cmd;
  message[2] = slave_add + 1;
  SMBus_Start();
  SMBus_Write(slave_add);
  if (SMBus_Wait_Ack())
    return 1;

  SMBus_Write(cmd);
  if (SMBus_Wait_Ack())
    return 2;

  SMBus_Start();
  SMBus_Write(slave_add + 1);
  if (SMBus_Wait_Ack())
    return 3;

  // ��ȡ���ĵ�1���ֽ�Ϊ֮����յ����ݳ���
  message[3] = SMBus_Read();
  if (message[3] != dat_len + 2)
    return 4;
  SMBus_Ack();

  message[4] = SMBus_Read();  // дblockʱ���͵ļĴ�����λ��ַ
  SMBus_Ack();
  message[5] = SMBus_Read();  // дblockʱ���͵ļĴ�����λ��ַ
  SMBus_Ack();

  // ��block�ж�ȡ����,�64���ֽ�
  for (uint8_t i = 0; i < dat_len && i < 64; i++) {
    message[i + 6] = SMBus_Read();
    SMBus_Ack();
  }
  pec = SMBus_Read();
  SMBus_NAck();
  SMBus_Stop();

  if (pec != calculate_crc8(message, dat_len + 6))
    return 5;

  for (uint8_t i = 0; i < dat_len && i < 64; i++)
    dat[i] = message[i + 6];

  return 0;
}

/***
 * @brief SMBus��һ����,����0��ʾ����
 * @param slave_add��ʾ�ӻ���ַ,��ǰ7λ,���1λ��Ч
 * @param cmd��ʾ��ȡ�ļĴ�����ַ
 * @param dat��ʾ���ص�����Ҫ���浽�ı�����ָ��
 */
uint8_t SMBus_Read_Word(uint8_t slave_add, uint8_t cmd, uint16_t* dat) {
  slave_add &= 0xFE;  // ֻ����ǰ7λ��ַ
  // ���洫���ڼ���������,���ʽ����:
  // �ӻ�д��ַ+����+�ӻ�����ַ+����(2�ֽ�)
  uint8_t message[5] = {0};
  uint8_t pec = 0;
  message[0] = slave_add;
  message[1] = cmd;
  message[2] = slave_add + 1;
  SMBus_Start();
  SMBus_Write(slave_add);
  if (SMBus_Wait_Ack())
    return 1;

  SMBus_Write(cmd);
  if (SMBus_Wait_Ack())
    return 2;

  SMBus_Start();
  SMBus_Write(slave_add + 1);
  if (SMBus_Wait_Ack())
    return 3;

  message[3] = SMBus_Read();
  SMBus_Ack();
  message[4] = SMBus_Read();
  SMBus_Ack();
  pec = SMBus_Read();  // ����У��
  SMBus_NAck();
  SMBus_Stop();

  // �Դ������ݽ���crcУ��
  if (pec != calculate_crc8(message, 5))
    return 4;

  *dat = (message[3] | (message[4] << 8));

  return 0;
}

// bq4050ͨ��ManufacturerBlockAccess()��ȡ�Ĵ�������
uint8_t BQ4050_MAC_Read(uint8_t cmd, uint8_t* dat, uint8_t dat_len) {
  if (dat_len > 64)
    return 1;
  memset(dat, 0, dat_len);
  uint8_t cmd_list[2] = {cmd, 0x00};
  if (SMBus_Write_Block(BQ4050_ADDRRESS, 0x44, cmd_list, 2))
    return 2;
  if (SMBus_Read_Block(BQ4050_ADDRRESS, 0x44, dat, dat_len))
    return 3;
  return 0;
}

// bq4050��ȡһ��2�ֽ�16λ�޷�������,div����Ϊ�������ƫ��ֵ
uint8_t BQ4050_U2_Read(uint8_t cmd, uint16_t* dat, uint16_t div) {
  uint16_t temp = 0;
  if (SMBus_Read_Word(BQ4050_ADDRRESS, cmd, &temp))
    return 1;
  if ((temp > (*dat) ? temp - (*dat) : (*dat) - temp) > div && (*dat) != 0)
    return 2;  // ƫ����󣬲�����ֵ

  *dat = temp;
  return 3;
}

// bq4050��ȡһ��2�ֽ�16λ�з�������
uint8_t BQ4050_I2_Read(uint8_t cmd, int16_t* dat, uint16_t div) {
  int16_t temp = 0;
  if (SMBus_Read_Word(BQ4050_ADDRRESS, cmd, (uint16_t*)&temp))
    return 1;
  if ((temp - (*dat) > 0 ? temp - (*dat) : -((*dat) - temp)) > div &&
      (*dat) != 0)
    return 2;  // ƫ�����,������ֵ

  *dat = temp;
  return 0;
}

// ��Ϣ��ȡ
uint8_t BQ4050_Read_Info() {
  uint8_t status = 0;
  uint8_t data[64] = {0};
  uint16_t temp = 0;
  status = BQ4050_U2_Read(0x09, &bq4050.voltage, 1000);
  status = BQ4050_I2_Read(0x0A, &bq4050.current, 65535);
  status = BQ4050_U2_Read(0x0E, &bq4050.absolute_SOC, 10);
  status = BQ4050_U2_Read(0x0D, &bq4050.relative_SOC, 10);
  status = BQ4050_U2_Read(0x08, &temp, 10);
  bq4050.temperature = (float)temp / 10.0 - 273.0;  // �¶Ȼ���

  for (uint8_t i = 0; i < 4; i++)
    status = BQ4050_U2_Read((0x3F - i), &bq4050.cell_voltage_arr[i], 1000);

  // ��MAC��DAStatus1�Ĵ�������,�������ڵ�ص�ѹ����������Ϣ,��ϸ�������ֲ�
  status = BQ4050_MAC_Read(0x71, data, 32);
  for (uint8_t i = 0; i < 4; i++)
    bq4050.cell_current_arr[i] =
        (data[(i + 6) * 2] | (data[(i + 6) * 2 + 1] << 8));
  bq4050.power = (data[14 * 2] | (data[14 * 2 + 1] << 8));

  // ��DAStatus2����,�����¶ȴ�������Ϣ
  status = BQ4050_MAC_Read(0x72, data, 14);
  for (uint8_t i = 0; i < 7; i++)
    bq4050.temperature_arr[i] =
        (data[i * 2] | (data[i * 2 + 1] << 8)) / 10 - 273;

  return status;
}
