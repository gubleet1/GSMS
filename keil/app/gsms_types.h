#ifndef GSMS_TYPES_H
#define GSMS_TYPES_H

#include <stdint.h>

// multi buffer defines
#define BUF_DEPTH_DOUBLE 2

// multi buffer type
typedef struct {
  uint8_t* data;
  uint8_t empty;
} multi_buf_t;

#endif
