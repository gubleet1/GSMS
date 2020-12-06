#ifndef GSMS_APP_MAIN_H
#define GSMS_APP_MAIN_H

#include <stdint.h>
#include "bno055.h"
#include "gsms_types.h"

// bno055 buffer defnies
#define BNO055_BUF_SIZE                (46u)
#define BNO055_BUF_BASE_REG_ADDR       (BNO055_ACCEL_DATA_X_LSB_ADDR)

// bno055 buffer
extern uint8_t bno055_curr_buf_index;
extern uint8_t bno055_next_buf_index;
extern multi_buf_t bno055_buf[];

// app initialization
void app_init(void);

// bno055 setup
static void bno055_setup(void);
// bno055 start
static void bno055_start(void);
// bno055 i2c bus functions
int8_t bno055_i2c_bus_read(uint8_t, uint8_t, uint8_t*, uint8_t);
int8_t bno055_i2c_bus_write(uint8_t, uint8_t, uint8_t*, uint8_t);

// neo-m9n setup
static void neom9n_setup(void);
// neo-m9n start
static void neom9n_start(void);

#endif
