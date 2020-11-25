#include <stdlib.h>
#include "stm32f4xx_hal.h"
#include "app_main.h"
#include "neom9n.h"
#include "att_kf.h"
#include "utils.h"
#include "processing.h"

// hal handles
extern UART_HandleTypeDef huart5;

// bno055 sample
double accel[VEC_3_SIZE];
double mag[VEC_3_SIZE];
double gyro[VEC_3_SIZE];
double quat[QUAT_SIZE];

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
      // unread bno055 raw data is available
      // update bno055 sample
      update_bno055_sample(bno055_buf[bno055_curr_buf_index].data);
      // attitude processing
      attitude_processing();

      if (GSMS_DEBUG) {
        // transmit bno055 raw data to host
        transmit_bno055_raw_data(bno055_buf[bno055_curr_buf_index].data);
      }

      // processing of current bno055 buffer is complete, set the empty flag
      bno055_buf[bno055_curr_buf_index].empty = 1u;
      // increment current bno055 buffer index
      bno055_buf_inc(&bno055_curr_buf_index);

      // check if unread neo-m9n raw data is available
      if (neom9n_data_ready(NEOM9N_UPDATE_BUF))
      {
        // unread neo-m9n raw data is available
        // update neo-m9n sample
        update_neom9n_sample(&neom9n_buf);
        // velocity processing
        velocity_processing();

        if (GSMS_DEBUG) {
          // transmit neo-m9n raw data to host
          transmit_neom9n_raw_data(&neom9n_buf);
        }

      } else {
        if (GSMS_DEBUG) {
          // transmit padding
          uint8_t* buf = (uint8_t*) calloc(1, sizeof(neom9n_buf_t));
          HAL_UART_Transmit(&huart5, buf, sizeof(neom9n_buf_t), HAL_MAX_DELAY);
          free(buf);
        }
      }

      if (GSMS_DEBUG) {
        // transmit tick
        uint32_t t = HAL_GetTick();
        HAL_UART_Transmit(&huart5, (uint8_t*) &t, 4, HAL_MAX_DELAY);
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
}
