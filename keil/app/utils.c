#include "main.h"
#include "processing.h"
#include "utils.h"

// hal handles
extern UART_HandleTypeDef huart5;

void transmit_bno055_raw_data(uint8_t* buf)
{
  // change endianness
  swap_LSB_MSB(buf, BNO055_BUF_SIZE);
  // transmit raw data using uart
  if (HAL_UART_Transmit(&huart5, buf, BNO055_BUF_SIZE, HAL_MAX_DELAY) != HAL_OK)
  {
    Error_Handler();
  }
}

void update_bno055_sample(uint8_t* buf)
{
  // temporary variables
  uint8_t tmp_lsb;
  uint8_t tmp_msb;
  int16_t tmp;

  // update bno055 accelerometer sample
  // x axis
  tmp_lsb = buf[BNO055_BUF_OFFSET_ACCEL_X_LSB];
  tmp_msb = buf[BNO055_BUF_OFFSET_ACCEL_X_MSB];
  tmp = combine_LSB_MSB(tmp_lsb, tmp_msb);
  accel[VEC_3_X] = (double) tmp / BNO055_ACCEL_DIV_MSQ;
  // y axis
  tmp_lsb = buf[BNO055_BUF_OFFSET_ACCEL_Y_LSB];
  tmp_msb = buf[BNO055_BUF_OFFSET_ACCEL_Y_MSB];
  tmp = combine_LSB_MSB(tmp_lsb, tmp_msb);
  accel[VEC_3_Y] = (double) tmp / BNO055_ACCEL_DIV_MSQ;
  // z axis
  tmp_lsb = buf[BNO055_BUF_OFFSET_ACCEL_Z_LSB];
  tmp_msb = buf[BNO055_BUF_OFFSET_ACCEL_Z_MSB];
  tmp = combine_LSB_MSB(tmp_lsb, tmp_msb);
  accel[VEC_3_Z] = (double) tmp / BNO055_ACCEL_DIV_MSQ;

  // update bno055 magnetometer sample
  // x axis
  tmp_lsb = buf[BNO055_BUF_OFFSET_MAG_X_LSB];
  tmp_msb = buf[BNO055_BUF_OFFSET_MAG_X_MSB];
  tmp = combine_LSB_MSB(tmp_lsb, tmp_msb);
  mag[VEC_3_X] = (double) tmp / BNO055_MAG_DIV_UT;
  // y axis
  tmp_lsb = buf[BNO055_BUF_OFFSET_MAG_Y_LSB];
  tmp_msb = buf[BNO055_BUF_OFFSET_MAG_Y_MSB];
  tmp = combine_LSB_MSB(tmp_lsb, tmp_msb);
  mag[VEC_3_Y] = (double) tmp / BNO055_MAG_DIV_UT;
  // z axis
  tmp_lsb = buf[BNO055_BUF_OFFSET_MAG_Z_LSB];
  tmp_msb = buf[BNO055_BUF_OFFSET_MAG_Z_MSB];
  tmp = combine_LSB_MSB(tmp_lsb, tmp_msb);
  mag[VEC_3_Z] = (double) tmp / BNO055_MAG_DIV_UT;

  // update bno055 gyroscope sample
  // x axis
  tmp_lsb = buf[BNO055_BUF_OFFSET_GYRO_X_LSB];
  tmp_msb = buf[BNO055_BUF_OFFSET_GYRO_X_MSB];
  tmp = combine_LSB_MSB(tmp_lsb, tmp_msb);
  gyro[VEC_3_X] = (double) tmp / BNO055_GYRO_DIV_RPS;
  // y axis
  tmp_lsb = buf[BNO055_BUF_OFFSET_GYRO_Y_LSB];
  tmp_msb = buf[BNO055_BUF_OFFSET_GYRO_Y_MSB];
  tmp = combine_LSB_MSB(tmp_lsb, tmp_msb);
  gyro[VEC_3_Y] = (double) tmp / BNO055_GYRO_DIV_RPS;
  // z axis
  tmp_lsb = buf[BNO055_BUF_OFFSET_GYRO_Z_LSB];
  tmp_msb = buf[BNO055_BUF_OFFSET_GYRO_Z_MSB];
  tmp = combine_LSB_MSB(tmp_lsb, tmp_msb);
  gyro[VEC_3_Z] = (double) tmp / BNO055_GYRO_DIV_RPS;

  // update bno055 quaternion data sample
  // w axis
  tmp_lsb = buf[BNO055_BUF_OFFSET_QUAT_W_LSB];
  tmp_msb = buf[BNO055_BUF_OFFSET_QUAT_W_MSB];
  tmp = combine_LSB_MSB(tmp_lsb, tmp_msb);
  quat[QUAT_W] = (double) tmp / BNO055_QUATERNION_DIV;
  // x axis
  tmp_lsb = buf[BNO055_BUF_OFFSET_QUAT_X_LSB];
  tmp_msb = buf[BNO055_BUF_OFFSET_QUAT_X_MSB];
  tmp = combine_LSB_MSB(tmp_lsb, tmp_msb);
  quat[QUAT_X] = (double) tmp / BNO055_QUATERNION_DIV;
  // y axis
  tmp_lsb = buf[BNO055_BUF_OFFSET_QUAT_Y_LSB];
  tmp_msb = buf[BNO055_BUF_OFFSET_QUAT_Y_MSB];
  tmp = combine_LSB_MSB(tmp_lsb, tmp_msb);
  quat[QUAT_Y] = (double) tmp / BNO055_QUATERNION_DIV;
  // z axis
  tmp_lsb = buf[BNO055_BUF_OFFSET_QUAT_Z_LSB];
  tmp_msb = buf[BNO055_BUF_OFFSET_QUAT_Z_MSB];
  tmp = combine_LSB_MSB(tmp_lsb, tmp_msb);
  quat[QUAT_Z] = (double) tmp / BNO055_QUATERNION_DIV;
}

void transmit_neom9n_raw_data(neom9n_buf_t* buf)
{
  // transmit raw data using uart
  if (HAL_UART_Transmit(&huart5, (uint8_t*) buf, sizeof(neom9n_buf_t), HAL_MAX_DELAY) != HAL_OK)
  {
    Error_Handler();
  }
}

void update_neom9n_sample(neom9n_buf_t* buf)
{
  // update neo-m9n velocity sample
  // north axis
  vel[VEC_NED_N] = buf->v_N / NEOM9N_VEL_NED_DIV_MPS;
  // east axis
  vel[VEC_NED_E] = buf->v_E / NEOM9N_VEL_NED_DIV_MPS;
  // down axis
  vel[VEC_NED_D] = buf->v_D / NEOM9N_VEL_NED_DIV_MPS;
}

static int16_t combine_LSB_MSB(uint8_t lsb, uint8_t msb)
{
  // combine LSB MSB
  return (int16_t)((((int32_t)(int8_t)(msb)) << BNO055_SHIFT_EIGHT_BITS) | (lsb));
}

static void swap_LSB_MSB(uint8_t* buf, uint8_t length)
{
  if (length % 2)
  {
    // invalid buffer length
    Error_Handler();
  }
  uint8_t tmp;
  // swap LSB MSB
  for (int i = 0; i < length; i+=2)
  {
    tmp = buf[i];
    buf[i] = buf[i+1];
    buf[i+1] = tmp;
  }
}
