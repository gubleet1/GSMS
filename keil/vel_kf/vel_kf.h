/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: vel_kf.h
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 30-Nov-2020 23:53:28
 */

#ifndef VEL_KF_H
#define VEL_KF_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

/* Variable Declarations */
extern double vel_kf_x[3];
extern double vel_kf_Q[9];
extern double vel_kf_R[9];
extern double vel_kf_P[9];

#ifdef __cplusplus

extern "C" {

#endif

  /* Function Declarations */
  extern void quat_conj(const double q[4], double qc[4]);
  extern void rotate_z_by_ang(const double v[3], double ang, double v_rot[3]);
  extern void vel_kf(void);
  extern void vel_kf_initialize(void);
  extern void vel_kf_measure(const double measured_b[3]);
  extern void vel_kf_predict(const double lin_accel[3]);
  extern void vel_kf_terminate(void);
  extern void vel_kf_vel(double vel[3]);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for vel_kf.h
 *
 * [EOF]
 */
