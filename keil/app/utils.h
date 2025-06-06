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
#define DEBUG_BUF_SIZE 68
#define DEBUG_BUF_OFFSET_BNO055 0
#define DEBUG_BUF_OFFSET_NEOM9N 38
#define DEBUG_BUF_OFFSET_QUAT_KF 50
#define DEBUG_BUF_OFFSET_TICK 58
#define DEBUG_BUF_OFFSET_VEL_KF 62
#define DEBUG_BUF_NEOM9N_PADDING_VAL 0x00000080
#define DEBUG_BUF_QUAT_SCALE ((double) (1u << 14u))
#define DEBUG_BUF_VEL_SCALE 1000.0

// math constants
#define M_PI 3.14159265358979323846

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

// bno055 buffer accelerometer data offset defines
#define BNO055_BUF_OFFSET_ACCEL_X_LSB  (BNO055_ACCEL_DATA_X_LSB_ADDR - BNO055_BUF_BASE_REG_ADDR)
#define BNO055_BUF_OFFSET_ACCEL_X_MSB  (BNO055_ACCEL_DATA_X_MSB_ADDR - BNO055_BUF_BASE_REG_ADDR)
#define BNO055_BUF_OFFSET_ACCEL_Y_LSB  (BNO055_ACCEL_DATA_Y_LSB_ADDR - BNO055_BUF_BASE_REG_ADDR)
#define BNO055_BUF_OFFSET_ACCEL_Y_MSB  (BNO055_ACCEL_DATA_Y_MSB_ADDR - BNO055_BUF_BASE_REG_ADDR)
#define BNO055_BUF_OFFSET_ACCEL_Z_LSB  (BNO055_ACCEL_DATA_Z_LSB_ADDR - BNO055_BUF_BASE_REG_ADDR)
#define BNO055_BUF_OFFSET_ACCEL_Z_MSB  (BNO055_ACCEL_DATA_Z_MSB_ADDR - BNO055_BUF_BASE_REG_ADDR)

// bno055 buffer magnetometer data offset defines
#define BNO055_BUF_OFFSET_MAG_X_LSB    (BNO055_MAG_DATA_X_LSB_ADDR - BNO055_BUF_BASE_REG_ADDR)
#define BNO055_BUF_OFFSET_MAG_X_MSB    (BNO055_MAG_DATA_X_MSB_ADDR - BNO055_BUF_BASE_REG_ADDR)
#define BNO055_BUF_OFFSET_MAG_Y_LSB    (BNO055_MAG_DATA_Y_LSB_ADDR - BNO055_BUF_BASE_REG_ADDR)
#define BNO055_BUF_OFFSET_MAG_Y_MSB    (BNO055_MAG_DATA_Y_MSB_ADDR - BNO055_BUF_BASE_REG_ADDR)
#define BNO055_BUF_OFFSET_MAG_Z_LSB    (BNO055_MAG_DATA_Z_LSB_ADDR - BNO055_BUF_BASE_REG_ADDR)
#define BNO055_BUF_OFFSET_MAG_Z_MSB    (BNO055_MAG_DATA_Z_MSB_ADDR - BNO055_BUF_BASE_REG_ADDR)

// bno055 buffer gyroscope data offset defines
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

// bno055 buffer linear acceleration data offset defines
#define BNO055_BUF_OFFSET_LIN_ACCEL_X_LSB  (BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR - BNO055_BUF_BASE_REG_ADDR)
#define BNO055_BUF_OFFSET_LIN_ACCEL_X_MSB  (BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR - BNO055_BUF_BASE_REG_ADDR)
#define BNO055_BUF_OFFSET_LIN_ACCEL_Y_LSB  (BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR - BNO055_BUF_BASE_REG_ADDR)
#define BNO055_BUF_OFFSET_LIN_ACCEL_Y_MSB  (BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR - BNO055_BUF_BASE_REG_ADDR)
#define BNO055_BUF_OFFSET_LIN_ACCEL_Z_LSB  (BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR - BNO055_BUF_BASE_REG_ADDR)
#define BNO055_BUF_OFFSET_LIN_ACCEL_Z_MSB  (BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR - BNO055_BUF_BASE_REG_ADDR)

// bno055 buffer status register offset defines
#define BNO055_BUF_OFFSET_CALIB_STAT  (BNO055_CALIB_STAT_ADDR - BNO055_BUF_BASE_REG_ADDR)

// debug buffer
extern uint8_t debug_curr_buf_index;
extern uint8_t debug_next_buf_index;
extern multi_buf_t debug_buf[];
extern uint8_t debug_tx_busy;

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

// add attitude kalman filter output to debug buffer
void debug_att_kf(void);

// add velocity kalman filter output to debug buffer
void debug_vel_kf(void);

// copy vector
void vec_copy(double*, double*, uint8_t);
// subtract vectors
void vec_sub(double*, double*, double*, uint8_t);

// combine two uint8_t to int16_t
static int16_t combine_uint8_t(uint8_t, uint8_t);
// change endianness of 16 bit types
static void change_endian_16_t(uint8_t*, uint8_t);
// change endianness of 32 bit types
static void change_endian_32_t(uint8_t*, uint8_t);

#endif
