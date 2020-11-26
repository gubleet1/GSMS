#ifndef GSMS_UTILS_H
#define GSMS_UTILS_H

#include <stdint.h>
#include "app_main.h"
#include "bno055.h"
#include "neom9n.h"
#include "gsms_types.h"

// debug defines
#define GSMS_DEBUG 1

// debug buffer defines
#define DEBUG_BUF_SIZE 48
#define DEBUG_BUF_OFFSET_BNO055 0
#define DEBUG_BUF_OFFSET_NEOM9N 32
#define DEBUG_BUF_OFFSET_TICK 44

// vector defines
// dimensions
#define VEC_3_SIZE 3
// x, y and z components of 3 element vectors
#define VEC_3_X 0
#define VEC_3_Y 1
#define VEC_3_Z 2
// north, east and down components of NED vectors
#define VEC_NED_N 0
#define VEC_NED_E 1
#define VEC_NED_D 2
// east, north and up components of ENU vectors
#define VEC_ENU_E 0
#define VEC_ENU_N 1
#define VEC_ENU_U 2

// quaternion defines
// dimensions
#define QUAT_SIZE 4
// w, x, y and z components of quaternions
#define QUAT_W 0
#define QUAT_X 1
#define QUAT_Y 2
#define QUAT_Z 3

// bno055 buffer accelerometer data offset defnies
#define BNO055_BUF_OFFSET_ACCEL_X_LSB  (BNO055_ACCEL_DATA_X_LSB_ADDR - BNO055_BUF_BASE_REG_ADDR)
#define BNO055_BUF_OFFSET_ACCEL_X_MSB  (BNO055_ACCEL_DATA_X_MSB_ADDR - BNO055_BUF_BASE_REG_ADDR)
#define BNO055_BUF_OFFSET_ACCEL_Y_LSB  (BNO055_ACCEL_DATA_Y_LSB_ADDR - BNO055_BUF_BASE_REG_ADDR)
#define BNO055_BUF_OFFSET_ACCEL_Y_MSB  (BNO055_ACCEL_DATA_Y_MSB_ADDR - BNO055_BUF_BASE_REG_ADDR)
#define BNO055_BUF_OFFSET_ACCEL_Z_LSB  (BNO055_ACCEL_DATA_Z_LSB_ADDR - BNO055_BUF_BASE_REG_ADDR)
#define BNO055_BUF_OFFSET_ACCEL_Z_MSB  (BNO055_ACCEL_DATA_Z_MSB_ADDR - BNO055_BUF_BASE_REG_ADDR)

// bno055 buffer magnetometer data offset defnies
#define BNO055_BUF_OFFSET_MAG_X_LSB    (BNO055_MAG_DATA_X_LSB_ADDR - BNO055_BUF_BASE_REG_ADDR)
#define BNO055_BUF_OFFSET_MAG_X_MSB    (BNO055_MAG_DATA_X_MSB_ADDR - BNO055_BUF_BASE_REG_ADDR)
#define BNO055_BUF_OFFSET_MAG_Y_LSB    (BNO055_MAG_DATA_Y_LSB_ADDR - BNO055_BUF_BASE_REG_ADDR)
#define BNO055_BUF_OFFSET_MAG_Y_MSB    (BNO055_MAG_DATA_Y_MSB_ADDR - BNO055_BUF_BASE_REG_ADDR)
#define BNO055_BUF_OFFSET_MAG_Z_LSB    (BNO055_MAG_DATA_Z_LSB_ADDR - BNO055_BUF_BASE_REG_ADDR)
#define BNO055_BUF_OFFSET_MAG_Z_MSB    (BNO055_MAG_DATA_Z_MSB_ADDR - BNO055_BUF_BASE_REG_ADDR)

// bno055 buffer gyroscope data offset defnies
#define BNO055_BUF_OFFSET_GYRO_X_LSB   (BNO055_GYRO_DATA_X_LSB_ADDR - BNO055_BUF_BASE_REG_ADDR)
#define BNO055_BUF_OFFSET_GYRO_X_MSB   (BNO055_GYRO_DATA_X_MSB_ADDR - BNO055_BUF_BASE_REG_ADDR)
#define BNO055_BUF_OFFSET_GYRO_Y_LSB   (BNO055_GYRO_DATA_Y_LSB_ADDR - BNO055_BUF_BASE_REG_ADDR)
#define BNO055_BUF_OFFSET_GYRO_Y_MSB   (BNO055_GYRO_DATA_Y_MSB_ADDR - BNO055_BUF_BASE_REG_ADDR)
#define BNO055_BUF_OFFSET_GYRO_Z_LSB   (BNO055_GYRO_DATA_Z_LSB_ADDR - BNO055_BUF_BASE_REG_ADDR)
#define BNO055_BUF_OFFSET_GYRO_Z_MSB   (BNO055_GYRO_DATA_Z_MSB_ADDR - BNO055_BUF_BASE_REG_ADDR)

// bno055 buffer quaternion data offset defines
#define BNO055_BUF_OFFSET_QUAT_W_LSB   (BNO055_QUATERNION_DATA_W_LSB_ADDR - BNO055_BUF_BASE_REG_ADDR)
#define BNO055_BUF_OFFSET_QUAT_W_MSB   (BNO055_QUATERNION_DATA_W_MSB_ADDR - BNO055_BUF_BASE_REG_ADDR)
#define BNO055_BUF_OFFSET_QUAT_X_LSB   (BNO055_QUATERNION_DATA_X_LSB_ADDR - BNO055_BUF_BASE_REG_ADDR)
#define BNO055_BUF_OFFSET_QUAT_X_MSB   (BNO055_QUATERNION_DATA_X_MSB_ADDR - BNO055_BUF_BASE_REG_ADDR)
#define BNO055_BUF_OFFSET_QUAT_Y_LSB   (BNO055_QUATERNION_DATA_Y_LSB_ADDR - BNO055_BUF_BASE_REG_ADDR)
#define BNO055_BUF_OFFSET_QUAT_Y_MSB   (BNO055_QUATERNION_DATA_Y_MSB_ADDR - BNO055_BUF_BASE_REG_ADDR)
#define BNO055_BUF_OFFSET_QUAT_Z_LSB   (BNO055_QUATERNION_DATA_Z_LSB_ADDR - BNO055_BUF_BASE_REG_ADDR)
#define BNO055_BUF_OFFSET_QUAT_Z_MSB   (BNO055_QUATERNION_DATA_Z_MSB_ADDR - BNO055_BUF_BASE_REG_ADDR)

// debug buffer
extern uint8_t debug_curr_buf_index;
extern uint8_t debug_next_buf_index;
extern multi_buf_t debug_buf[];

// multi buffer initialization
void multi_buf_init(multi_buf_t*, uint8_t, uint8_t);
// index increment with wraparound
void index_inc_wrap(uint8_t*, uint8_t);

// transmit debug buffer to host
void transmit_debug(void);
// add tick to debug buffer
void debug_tick(void);

// add bno055 raw data to debug buffer
void debug_bno055_raw_data(uint8_t*);
// update bno055 sample variables from raw data
void update_bno055_sample(uint8_t*);

// add neo-m9n raw data to debug buffer
void debug_neom9n_raw_data(neom9n_buf_t*);
// add neo-m9n padding to debug buffer
void debug_neom9n_padding(void);
// update neo-m9n sample variables from raw data
void update_neom9n_sample(neom9n_buf_t*);

// combine unsigned LSB and MSB to signed int16_t
static int16_t combine_LSB_MSB(uint8_t, uint8_t);
// change endianness
static void swap_LSB_MSB(uint8_t*, uint8_t);

#endif
