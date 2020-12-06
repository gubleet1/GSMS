#include <stdlib.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "main.h"
#include "processing.h"
#include "utils.h"

// hal handles
extern UART_HandleTypeDef huart5;

// debug buffer
uint8_t debug_curr_buf_index = 0u;
uint8_t debug_next_buf_index = 0u;
multi_buf_t debug_buf[BUF_DEPTH_DOUBLE];
uint8_t debug_tx_busy = 0u;
static uint32_t debug_buf_neom9n_padding[VEC_3_SIZE]
  = {DEBUG_BUF_NEOM9N_PADDING_VAL,
     DEBUG_BUF_NEOM9N_PADDING_VAL,
     DEBUG_BUF_NEOM9N_PADDING_VAL};

void multi_buf_init(multi_buf_t* multi_buf, uint8_t depth, uint8_t size)
{
  // check depth and size
  if (depth == 0 || size == 0)
  {
    Error_Handler();
  }
  // initialize buffers
  for (int i = 0; i < depth; i++)
  {
    // allocate memory
    multi_buf[i].data = (uint8_t*) malloc(size * sizeof(uint8_t));
    // set empty flag
    multi_buf[i].empty = 1u;
  }
}

void index_inc_wrap(uint8_t* index, uint8_t size)
{
  // increment the index and wrap to zero if size is reached
  *index = (*index + 1u) % size;
}

void transmit_debug(void)
{
  // transmit current debug buffer to host using uart
  uint8_t *p_debug_buf = debug_buf[debug_curr_buf_index].data;
  if (HAL_UART_Transmit_DMA(&huart5, p_debug_buf, DEBUG_BUF_SIZE) != HAL_OK)
  {
    gsms_error += 0x10;
    Error_Handler();
  }
}

void debug_tick(void)
{
  // get tick
  uint32_t t = HAL_GetTick();
  // add tick to debug buffer
  uint8_t *p_debug_buf = debug_buf[debug_next_buf_index].data;
  memcpy(p_debug_buf + DEBUG_BUF_OFFSET_TICK, &t, sizeof(uint32_t));
}

void debug_bno055_raw_data(uint8_t* buf)
{
  // add bno055 raw data to debug buffer
  uint8_t *p_debug_buf = debug_buf[debug_next_buf_index].data;
  memcpy(p_debug_buf + DEBUG_BUF_OFFSET_BNO055, buf, 38u);
  // change endianness
  change_endian_16_t(p_debug_buf + DEBUG_BUF_OFFSET_BNO055, 38u);
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
  tmp = combine_uint8_t(tmp_lsb, tmp_msb);
  accel[VEC_3_X] = (double) tmp / BNO055_ACCEL_DIV_MSQ;
  // y axis
  tmp_lsb = buf[BNO055_BUF_OFFSET_ACCEL_Y_LSB];
  tmp_msb = buf[BNO055_BUF_OFFSET_ACCEL_Y_MSB];
  tmp = combine_uint8_t(tmp_lsb, tmp_msb);
  accel[VEC_3_Y] = (double) tmp / BNO055_ACCEL_DIV_MSQ;
  // z axis
  tmp_lsb = buf[BNO055_BUF_OFFSET_ACCEL_Z_LSB];
  tmp_msb = buf[BNO055_BUF_OFFSET_ACCEL_Z_MSB];
  tmp = combine_uint8_t(tmp_lsb, tmp_msb);
  accel[VEC_3_Z] = (double) tmp / BNO055_ACCEL_DIV_MSQ;

  // update bno055 magnetometer sample
  // x axis
  tmp_lsb = buf[BNO055_BUF_OFFSET_MAG_X_LSB];
  tmp_msb = buf[BNO055_BUF_OFFSET_MAG_X_MSB];
  tmp = combine_uint8_t(tmp_lsb, tmp_msb);
  mag[VEC_3_X] = (double) tmp / BNO055_MAG_DIV_UT;
  // y axis
  tmp_lsb = buf[BNO055_BUF_OFFSET_MAG_Y_LSB];
  tmp_msb = buf[BNO055_BUF_OFFSET_MAG_Y_MSB];
  tmp = combine_uint8_t(tmp_lsb, tmp_msb);
  mag[VEC_3_Y] = (double) tmp / BNO055_MAG_DIV_UT;
  // z axis
  tmp_lsb = buf[BNO055_BUF_OFFSET_MAG_Z_LSB];
  tmp_msb = buf[BNO055_BUF_OFFSET_MAG_Z_MSB];
  tmp = combine_uint8_t(tmp_lsb, tmp_msb);
  mag[VEC_3_Z] = (double) tmp / BNO055_MAG_DIV_UT;

  // update bno055 gyroscope sample
  // x axis
  tmp_lsb = buf[BNO055_BUF_OFFSET_GYRO_X_LSB];
  tmp_msb = buf[BNO055_BUF_OFFSET_GYRO_X_MSB];
  tmp = combine_uint8_t(tmp_lsb, tmp_msb);
  gyro[VEC_3_X] = (double) tmp / BNO055_GYRO_DIV_RPS;
  // y axis
  tmp_lsb = buf[BNO055_BUF_OFFSET_GYRO_Y_LSB];
  tmp_msb = buf[BNO055_BUF_OFFSET_GYRO_Y_MSB];
  tmp = combine_uint8_t(tmp_lsb, tmp_msb);
  gyro[VEC_3_Y] = (double) tmp / BNO055_GYRO_DIV_RPS;
  // z axis
  tmp_lsb = buf[BNO055_BUF_OFFSET_GYRO_Z_LSB];
  tmp_msb = buf[BNO055_BUF_OFFSET_GYRO_Z_MSB];
  tmp = combine_uint8_t(tmp_lsb, tmp_msb);
  gyro[VEC_3_Z] = (double) tmp / BNO055_GYRO_DIV_RPS;

  // update bno055 quaternion data sample
  // w axis
  tmp_lsb = buf[BNO055_BUF_OFFSET_QUAT_W_LSB];
  tmp_msb = buf[BNO055_BUF_OFFSET_QUAT_W_MSB];
  tmp = combine_uint8_t(tmp_lsb, tmp_msb);
  quat[QUAT_W] = (double) tmp / BNO055_QUATERNION_DIV;
  // x axis
  tmp_lsb = buf[BNO055_BUF_OFFSET_QUAT_X_LSB];
  tmp_msb = buf[BNO055_BUF_OFFSET_QUAT_X_MSB];
  tmp = combine_uint8_t(tmp_lsb, tmp_msb);
  quat[QUAT_X] = (double) tmp / BNO055_QUATERNION_DIV;
  // y axis
  tmp_lsb = buf[BNO055_BUF_OFFSET_QUAT_Y_LSB];
  tmp_msb = buf[BNO055_BUF_OFFSET_QUAT_Y_MSB];
  tmp = combine_uint8_t(tmp_lsb, tmp_msb);
  quat[QUAT_Y] = (double) tmp / BNO055_QUATERNION_DIV;
  // z axis
  tmp_lsb = buf[BNO055_BUF_OFFSET_QUAT_Z_LSB];
  tmp_msb = buf[BNO055_BUF_OFFSET_QUAT_Z_MSB];
  tmp = combine_uint8_t(tmp_lsb, tmp_msb);
  quat[QUAT_Z] = (double) tmp / BNO055_QUATERNION_DIV;

  // update bno055 linear acceleration data sample
  // x axis
  tmp_lsb = buf[BNO055_BUF_OFFSET_LIN_ACCEL_X_LSB];
  tmp_msb = buf[BNO055_BUF_OFFSET_LIN_ACCEL_X_MSB];
  tmp = combine_uint8_t(tmp_lsb, tmp_msb);
  lin_accel[VEC_3_X] = (double) tmp / BNO055_ACCEL_DIV_MSQ;
  // y axis
  tmp_lsb = buf[BNO055_BUF_OFFSET_LIN_ACCEL_Y_LSB];
  tmp_msb = buf[BNO055_BUF_OFFSET_LIN_ACCEL_Y_MSB];
  tmp = combine_uint8_t(tmp_lsb, tmp_msb);
  lin_accel[VEC_3_Y] = (double) tmp / BNO055_ACCEL_DIV_MSQ;
  // z axis
  tmp_lsb = buf[BNO055_BUF_OFFSET_LIN_ACCEL_Z_LSB];
  tmp_msb = buf[BNO055_BUF_OFFSET_LIN_ACCEL_Z_MSB];
  tmp = combine_uint8_t(tmp_lsb, tmp_msb);
  lin_accel[VEC_3_Z] = (double) tmp / BNO055_ACCEL_DIV_MSQ;

  // update bno055 calibration status
  uint8_t calib_stat = buf[BNO055_BUF_OFFSET_CALIB_STAT];
  calib_ok = ((calib_stat == 0xFF) ? 0x01 : 0x00);
}

void debug_neom9n_raw_data(neom9n_buf_t* buf)
{
  // add neo-m9n raw data to debug buffer
  uint8_t *p_debug_buf = debug_buf[debug_next_buf_index].data;
  memcpy(p_debug_buf + DEBUG_BUF_OFFSET_NEOM9N, buf, NEOM9N_BUF_SIZE);
  // change endianness
  change_endian_32_t(p_debug_buf + DEBUG_BUF_OFFSET_NEOM9N, NEOM9N_BUF_SIZE);
}

void debug_neom9n_padding(void)
{
  // add neo-m9n padding to debug buffer
  uint8_t *p_debug_buf = debug_buf[debug_next_buf_index].data;
  memcpy(p_debug_buf + DEBUG_BUF_OFFSET_NEOM9N, debug_buf_neom9n_padding, NEOM9N_BUF_SIZE);
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

void debug_att_kf(void)
{
  // add attitude kalman filter output to debug buffer
  uint8_t *p_debug_buf = debug_buf[debug_next_buf_index].data;
  int16_t tmp;
  for (int i = 0; i < QUAT_SIZE; i++)
  {
    tmp = (int16_t) (quat_kf[i] * DEBUG_BUF_QUAT_SCALE);
    memcpy(p_debug_buf + DEBUG_BUF_OFFSET_QUAT_KF + (i * 2), &tmp, 2u);
  }
  // change endianness
  change_endian_16_t(p_debug_buf + DEBUG_BUF_OFFSET_QUAT_KF, 8u);
}

void vec_copy(double* vdst, double* vsrc, uint8_t size)
{
  // size check
  if (size == 0)
  {
    // invalid size
    Error_Handler();
  }
  // copy vector
  for (int i = 0; i < size; i++)
  {
    vdst[i] = vsrc[i];
  }
}

void vec_sub(double* va, double* vb, double* vres, uint8_t size)
{
  // size check
  if (size == 0)
  {
    // invalid size
    Error_Handler();
  }
  // subtract vectors
  for (int i = 0; i < size; i++)
  {
    vres[i] = va[i] - vb[i];
  }
}

static int16_t combine_uint8_t(uint8_t lsb, uint8_t msb)
{
  // combine two uint8_t to int16_t
  return (int16_t)((((int32_t)(int8_t)(msb)) << BNO055_SHIFT_EIGHT_BITS) | (lsb));
}

static void change_endian_16_t(uint8_t* buf, uint8_t length)
{
  if (length % 2)
  {
    // invalid buffer length
    Error_Handler();
  }
  uint8_t tmp;
  // change endianness
  for (int i = 0; i < length; i+=2)
  {
    tmp = buf[i];
    buf[i] = buf[i+1];
    buf[i+1] = tmp;
  }
}

static void change_endian_32_t(uint8_t* buf, uint8_t length)
{
  if (length % 4)
  {
    // invalid buffer length
    Error_Handler();
  }
  uint8_t tmp;
  // change endianness
  for (int i = 0; i < length; i+=4)
  {
    tmp = buf[i];
    buf[i] = buf[i+3];
    buf[i+3] = tmp;
    tmp = buf[i+1];
    buf[i+1] = buf[i+2];
    buf[i+2] = tmp;
  }
}
