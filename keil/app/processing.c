#include "stm32f4xx_hal.h"
#include "main.h"
#include "app_main.h"
#include "neom9n.h"
#include "att_kf.h"
#include "vel_kf.h"
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
uint8_t calib_ok;

// neo-m9n sample
double vel[VEC_3_SIZE];

// attitude kalman filter output
double quat_kf[QUAT_SIZE];

// velocity kalman filter output
double velocity_kf[VEC_3_SIZE];

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
      // display bno055 calibration status
      display_calib_stat();

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
        // add velocity kalman filter output to debug buffer
        debug_vel_kf();
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
  double accel_in[VEC_3_SIZE];
  vec_copy(accel_in, accel, VEC_3_SIZE);
  att_kf_measure(gravity_i, accel_in);
  // measure north vector
  double north_i[VEC_3_SIZE] = {0.0, 0.446, -0.895};
  double mag_in[VEC_3_SIZE];
  vec_copy(mag_in, mag, VEC_3_SIZE);
  att_kf_measure(north_i, mag_in);
  // propagate attitude error
  att_kf_propagate();
  // get attitude kalman filter output
  att_kf_q_ref(quat_kf);
}

static void velocity_processing(void)
{
  // calculate linear acceleration
  double quat_kf_conj[QUAT_SIZE];
  quat_conj(quat_kf, quat_kf_conj);
  double gravity_i[VEC_3_SIZE] = {0.0, 0.0, 9.8053};
  double gravity_b[VEC_3_SIZE];
  // rotate gravity vector from the inertial frame into the body frame
  rotate_by_quat(gravity_i, quat_kf_conj, gravity_b);
  // subtract gravity vector from acceleration vector
  double lin_accel_kf[VEC_3_SIZE];
  vec_sub(accel, gravity_b, lin_accel_kf, VEC_3_SIZE);

  // prediction step (time update)
  vel_kf_predict(lin_accel_kf);

  // check if unread neo-m9n raw data is available
  if (neom9n_data_ready(NEOM9N_UPDATE_BUF))
  {
    // unread neo-m9n raw data is available
    // update neo-m9n sample
    update_neom9n_sample(&neom9n_buf);

    // change velocity reference frame from NED to ENU
    double vel_ENU[VEC_3_SIZE];
    vel_ENU[VEC_ENU_E] = vel[VEC_NED_E];
    vel_ENU[VEC_ENU_N] = vel[VEC_NED_N];
    vel_ENU[VEC_ENU_U] = -vel[VEC_NED_D];
    // correct declination
    double ang = 2.9425 / 180.0 * M_PI;
    double measured_i[VEC_3_SIZE];
    rotate_z_by_ang(vel_ENU, ang, measured_i);
    // rotate measurement from the inertial frame into the body frame
    double measured_b[VEC_3_SIZE];
    rotate_by_quat(measured_i, quat_kf_conj, measured_b);

    // measurement step (measurement update)
    // measure velocity vector
    vel_kf_measure(measured_b);

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
  vel_kf_vel(velocity_kf);
}

static void display_calib_stat(void)
{
  // display the calibration status using the green LED
  if (calib_ok)
  {
    // turn on the green LED
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  }
  else
  {
    // turn off the green LED
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  }
}
