#include "stm32f4xx_hal.h"
#include "main.h"
#include "app_main.h"
#include "neom9n.h"
#include "att_kf.h"
#include "utils.h"
#include "processing.h"

// debug variables
uint8_t dt_proc;
double att_error;

// bno055 sample
double accel[VEC_3_SIZE];
double mag[VEC_3_SIZE];
double gyro[VEC_3_SIZE];
double quat[QUAT_SIZE];
double lin_accel[VEC_3_SIZE];

// neo-m9n sample
double vel[VEC_3_SIZE];

// attitude kalman filter output
double quat_kf[QUAT_SIZE];

// velocity kalman filter output
double vel_kf[VEC_3_SIZE];

// data processing enabled
uint8_t processing_enabled = 0u;

void processing(void)
{
  if (processing_enabled)
  {
    // data processing is enabled
    if (!bno055_buf[bno055_curr_buf_index].empty)
    {
      // start time
      uint32_t t0;
      if (GSMS_DEBUG) {
        // record start time
        t0 = HAL_GetTick();
        // check if next debug buffer is empty
        if (!debug_buf[debug_next_buf_index].empty)
        {
          gsms_error += 0x20;
          Error_Handler();
        }
      }

      // unread bno055 raw data is available
      // update bno055 sample
      update_bno055_sample(bno055_buf[bno055_curr_buf_index].data);
      // attitude processing
      attitude_processing();

      if (GSMS_DEBUG) {
        // add bno055 raw data to debug buffer
        debug_bno055_raw_data(bno055_buf[bno055_curr_buf_index].data);
      }

      // processing of current bno055 buffer is complete, set the empty flag
      bno055_buf[bno055_curr_buf_index].empty = 1u;
      // increment current buffer index
      index_inc_wrap(&bno055_curr_buf_index, BUF_DEPTH_DOUBLE);

      // velocity processing
      velocity_processing();

      if (GSMS_DEBUG) {
        // add attitude kalman filter output to debug buffer
        debug_att_kf();
        // add tick to debug buffer
        debug_tick();

        // writing into next debug buffer is complete, clear the empty flag
        debug_buf[debug_next_buf_index].empty = 0u;
        // increment next buffer index
        index_inc_wrap(&debug_next_buf_index, BUF_DEPTH_DOUBLE);

        // calculate attitude error
        att_error = angle_between_quat(quat_kf, quat) / M_PI * 180.0;
        // calculate delta t
        dt_proc = (uint8_t) (HAL_GetTick() - t0);
      }
    }
  }
}

static void attitude_processing(void)
{
  // prediction step (time update)
  att_kf_predict(gyro);
  // measurement step (measurement update)
  // measure gravity vector
  double gravity_i[VEC_3_SIZE] = {0.0, 0.0, 1.0};
  att_kf_measure(gravity_i, accel);
  // measure north vector
  double north_i[VEC_3_SIZE] = {0.0, 0.446, -0.895};
  att_kf_measure(north_i, mag);
  // propagate attitude error
  att_kf_propagate();
  // get attitude kalman filter output
  att_kf_q_ref(quat_kf);
}

static void velocity_processing(void)
{
  // prediction step (time update)

  // check if unread neo-m9n raw data is available
  if (neom9n_data_ready(NEOM9N_UPDATE_BUF))
  {
    // unread neo-m9n raw data is available
    // update neo-m9n sample
    update_neom9n_sample(&neom9n_buf);

    // measurement step (measurement update)

    if (GSMS_DEBUG) {
      // add neo-m9n raw data to debug buffer
      debug_neom9n_raw_data(&neom9n_buf);
    }
  } else {
    if (GSMS_DEBUG) {
      // add neo-m9n padding to debug buffer
      debug_neom9n_padding();
    }
  }
  // get velocity kalman filter output
}
