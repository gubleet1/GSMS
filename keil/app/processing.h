#ifndef GSMS_PROCESSING_H
#define GSMS_PROCESSING_H

#include <stdint.h>

// debug variables
extern uint8_t dt_proc;
extern double att_error;

// bno055 sample
extern double accel[];
extern double mag[];
extern double gyro[];
extern double quat[];
extern double lin_accel[];
extern uint8_t calib_ok;

// neo-m9n sample
extern double vel[];

// attitude kalman filter output
extern double quat_kf[];

// velocity kalman filter output
extern double velocity_kf[];

// data processing enabled
extern uint8_t processing_enabled;

// data processing
void processing(void);
// attitude processing
static void attitude_processing(void);
// velocity processing
static void velocity_processing(void);

// display bno055 calibration status
static void display_calib_stat(void);

#endif
