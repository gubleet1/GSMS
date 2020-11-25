/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: att_kf.h
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 25-Nov-2020 21:15:25
 */

#ifndef ATT_KF_H
#define ATT_KF_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

/* Variable Declarations */
extern double b_att_kf_q_ref[4];
extern double att_kf_x[6];
extern double att_kf_Q[36];
extern double att_kf_R[4];
extern double att_kf_P[36];

#ifdef __cplusplus

extern "C" {

#endif

  /* Function Declarations */
  extern double angle_between_quat(const double q1[4], const double q2[4]);
  extern void att_kf(void);
  extern void att_kf_initialize(void);
  extern void att_kf_measure(double expected_i[3], double measured_b[3]);
  extern void att_kf_predict(const double w[3]);
  extern void att_kf_propagate(void);
  extern void att_kf_q_ref(double q_ref[4]);
  extern void att_kf_terminate(void);
  extern void rotate_by_quat(const double v[3], const double q[4], double v_rot
    [3]);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for att_kf.h
 *
 * [EOF]
 */
