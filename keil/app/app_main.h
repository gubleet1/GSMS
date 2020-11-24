#ifndef GSMS_APP_MAIN_H
#define GSMS_APP_MAIN_H

#include <stdint.h>
#include "bno055.h"

// bno055 buffer defnies
#define BNO055_BUF_SIZE                (32u)
#define BNO055_BUF_BASE_REG_ADDR       (BNO055_ACCEL_DATA_X_LSB_ADDR)

// bno055 buffer type
typedef struct {
  uint8_t* data;
  uint8_t empty;
} bno055_buf_t;

// bno055 buffers
extern uint8_t bno055_curr_buf_index;
extern uint8_t bno055_next_buf_index;
extern bno055_buf_t bno055_buf[2];

// app initialization
void app_init(void);

// bno055 setup
static void bno055_setup(void);
// bno055 start
static void bno055_start(void);
// bno055 buffer initialization
static void bno055_buf_init(void);
// bno055 buffer index increment
void bno055_buf_inc(uint8_t*);
// bno055 i2c bus functions
int8_t bno055_i2c_bus_read(uint8_t, uint8_t, uint8_t*, uint8_t);
int8_t bno055_i2c_bus_write(uint8_t, uint8_t, uint8_t*, uint8_t);

// neo-m9n setup
static void neom9n_setup(void);
// neo-m9n start
static void neom9n_start(void);

#endif
