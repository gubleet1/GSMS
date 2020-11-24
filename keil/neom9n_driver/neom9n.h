#ifndef GSMS_NEOM9N_H
#define GSMS_NEOM9N_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// neo-m9n defines
#define NEOM9N_OUTPUT_RATE 25
#define NEOM9N_BAUD_RATE 115200
#define NEOM9N_DEFAULT_BAUD_RATE 38400
#define NEOM9N_UPDATE_BUF 1

// neo-m9n constants
#define NEOM9N_VEL_NED_DIV_MPS (1000.0)

// neo-m9n buffer type
typedef struct {
  // NED velocity
  int32_t v_N;
  int32_t v_E;
  int32_t v_D;
} neom9n_buf_t;

// neo-m9n buffer
extern neom9n_buf_t neom9n_buf;

// initialize driver
uint8_t neom9n_init(void);
// connect to device
static uint8_t connect(void);
// configure device
static uint8_t configure(void);

// check for new bytes in UART buffer and process them
void neom9n_check(void);
// check if unread raw data is available
uint8_t neom9n_data_ready(uint8_t);
// update buffer with raw data
void neom9n_update_buf(void);

#ifdef __cplusplus
}
#endif

#endif
