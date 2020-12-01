/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: vel_kf.c
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 30-Nov-2020 23:53:28
 */

/* Include Files */
#include "vel_kf.h"
#include <math.h>
#include <string.h>

/* Variable Definitions */
double vel_kf_x[3];
double vel_kf_Q[9];
double vel_kf_R[9];
double vel_kf_P[9];
static bool isInitialized_vel_kf = false;

/* Function Definitions */
/*
 * QUAT_CONJ quaternion conjugate
 * Arguments    : const double q[4]
 *                double qc[4]
 * Return Type  : void
 */
void quat_conj(const double q[4], double qc[4])
{
  if (!isInitialized_vel_kf) {
    vel_kf_initialize();
  }

  qc[0] = q[0];
  qc[1] = -q[1];
  qc[2] = -q[2];
  qc[3] = -q[3];
}

/*
 * ROTATE_Z_BY_ANG rotate 3 element vector around z axis by angle
 * Arguments    : const double v[3]
 *                double ang
 *                double v_rot[3]
 * Return Type  : void
 */
void rotate_z_by_ang(const double v[3], double ang, double v_rot[3])
{
  double c_Rz_tmp[9];
  double Rz_tmp;
  double b_Rz_tmp;
  double d;
  int i;
  if (!isInitialized_vel_kf) {
    vel_kf_initialize();
  }

  Rz_tmp = sin(ang);
  b_Rz_tmp = cos(ang);
  c_Rz_tmp[0] = b_Rz_tmp;
  c_Rz_tmp[3] = -Rz_tmp;
  c_Rz_tmp[6] = 0.0;
  c_Rz_tmp[1] = Rz_tmp;
  c_Rz_tmp[4] = b_Rz_tmp;
  c_Rz_tmp[7] = 0.0;
  c_Rz_tmp[2] = 0.0;
  c_Rz_tmp[5] = 0.0;
  c_Rz_tmp[8] = 1.0;
  Rz_tmp = v[0];
  b_Rz_tmp = v[1];
  d = v[2];
  for (i = 0; i < 3; i++) {
    v_rot[i] = (c_Rz_tmp[i] * Rz_tmp + c_Rz_tmp[i + 3] * b_Rz_tmp) + c_Rz_tmp[i
      + 6] * d;
  }
}

/*
 * VEL_KF initialize velocity kalman filter
 * Arguments    : void
 * Return Type  : void
 */
void vel_kf(void)
{
  static const double dv[9] = { 2.163237932025E-6, 0.0, 0.0, 0.0,
    2.163237932025E-6, 0.0, 0.0, 0.0, 2.163237932025E-6 };

  static const double dv1[9] = { 0.0048738787165873371, 0.0, 0.0, 0.0,
    0.0048738787165873371, 0.0, 0.0, 0.0, 0.0048738787165873371 };

  static const short iv[9] = { 1000, 0, 0, 0, 1000, 0, 0, 0, 1000 };

  int i;
  if (!isInitialized_vel_kf) {
    vel_kf_initialize();
  }

  for (i = 0; i < 9; i++) {
    vel_kf_Q[i] = dv[i];
    vel_kf_R[i] = dv1[i];
    vel_kf_P[i] = iv[i];
  }

  /*  state vector x */
  /*  process noise covariance matrix Q */
  /*  measurement noise covariance matrix R */
  /*  error covariance matrix P */
  /*  velocity kalman filter constants */
  /*  [m/s^2/sqrt(Hz)] */
  /*  [m/s/sqrt(Hz)] */
  /*  [m/s] */
  /*  initialize kalman filter variables */
  vel_kf_x[0] = 0.0;
  vel_kf_x[1] = 0.0;
  vel_kf_x[2] = 0.0;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void vel_kf_initialize(void)
{
  memset(&vel_kf_P[0], 0, 9U * sizeof(double));
  memset(&vel_kf_R[0], 0, 9U * sizeof(double));
  memset(&vel_kf_Q[0], 0, 9U * sizeof(double));
  vel_kf_x[0] = 0.0;
  vel_kf_x[1] = 0.0;
  vel_kf_x[2] = 0.0;
  isInitialized_vel_kf = true;
}

/*
 * VEL_KF_MEASURE measurement step (measurement update)
 * Arguments    : const double measured_b[3]
 * Return Type  : void
 */
void vel_kf_measure(const double measured_b[3])
{
  double K[9];
  double S[9];
  double K_tmp;
  double a21;
  double b_K_tmp;
  double c_K_tmp;
  double d_K_tmp;
  double maxval;
  double measured_b_idx_0;
  double measured_b_idx_1;
  int e_K_tmp;
  int f_K_tmp;
  int r1;
  int r2;
  int r3;
  int rtemp;
  if (!isInitialized_vel_kf) {
    vel_kf_initialize();
  }

  /*  state vector x */
  /*  measurement noise covariance matrix R */
  /*  error covariance matrix P */
  /*  calculate measurement innovation vector y */
  /*  calculate kalman gain matrix K */
  for (rtemp = 0; rtemp < 9; rtemp++) {
    S[rtemp] = vel_kf_P[rtemp] + vel_kf_R[rtemp];
  }

  r1 = 0;
  r2 = 1;
  r3 = 2;
  maxval = fabs(S[0]);
  a21 = fabs(S[1]);
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }

  if (fabs(S[2]) > maxval) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }

  S[r2] /= S[r1];
  S[r3] /= S[r1];
  S[r2 + 3] -= S[r2] * S[r1 + 3];
  S[r3 + 3] -= S[r3] * S[r1 + 3];
  S[r2 + 6] -= S[r2] * S[r1 + 6];
  S[r3 + 6] -= S[r3] * S[r1 + 6];
  if (fabs(S[r3 + 3]) > fabs(S[r2 + 3])) {
    rtemp = r2;
    r2 = r3;
    r3 = rtemp;
  }

  S[r3 + 3] /= S[r2 + 3];
  S[r3 + 6] -= S[r3 + 3] * S[r2 + 6];

  /*  calculate corrected (aposteriori) state vector x(+) */
  K[3 * r1] = vel_kf_P[0] / S[r1];
  maxval = S[r1 + 3];
  K[3 * r2] = vel_kf_P[3] - K[3 * r1] * maxval;
  a21 = S[r1 + 6];
  K[3 * r3] = vel_kf_P[6] - K[3 * r1] * a21;
  K_tmp = S[r2 + 3];
  K[3 * r2] /= K_tmp;
  b_K_tmp = S[r2 + 6];
  K[3 * r3] -= K[3 * r2] * b_K_tmp;
  c_K_tmp = S[r3 + 6];
  K[3 * r3] /= c_K_tmp;
  d_K_tmp = S[r3 + 3];
  K[3 * r2] -= K[3 * r3] * d_K_tmp;
  K[3 * r1] -= K[3 * r3] * S[r3];
  K[3 * r1] -= K[3 * r2] * S[r2];
  measured_b_idx_0 = measured_b[0] - vel_kf_x[0];
  e_K_tmp = 3 * r1 + 1;
  K[e_K_tmp] = vel_kf_P[1] / S[r1];
  rtemp = 3 * r2 + 1;
  K[rtemp] = vel_kf_P[4] - K[e_K_tmp] * maxval;
  f_K_tmp = 3 * r3 + 1;
  K[f_K_tmp] = vel_kf_P[7] - K[e_K_tmp] * a21;
  K[rtemp] /= K_tmp;
  K[f_K_tmp] -= K[rtemp] * b_K_tmp;
  K[f_K_tmp] /= c_K_tmp;
  K[rtemp] -= K[f_K_tmp] * d_K_tmp;
  K[e_K_tmp] -= K[f_K_tmp] * S[r3];
  K[e_K_tmp] -= K[rtemp] * S[r2];
  measured_b_idx_1 = measured_b[1] - vel_kf_x[1];
  e_K_tmp = 3 * r1 + 2;
  K[e_K_tmp] = vel_kf_P[2] / S[r1];
  rtemp = 3 * r2 + 2;
  K[rtemp] = vel_kf_P[5] - K[e_K_tmp] * maxval;
  f_K_tmp = 3 * r3 + 2;
  K[f_K_tmp] = vel_kf_P[8] - K[e_K_tmp] * a21;
  K[rtemp] /= K_tmp;
  K[f_K_tmp] -= K[rtemp] * b_K_tmp;
  K[f_K_tmp] /= c_K_tmp;
  K[rtemp] -= K[f_K_tmp] * d_K_tmp;
  K[e_K_tmp] -= K[f_K_tmp] * S[r3];
  K[e_K_tmp] -= K[rtemp] * S[r2];
  maxval = measured_b[2] - vel_kf_x[2];
  for (rtemp = 0; rtemp < 3; rtemp++) {
    vel_kf_x[rtemp] += (K[rtemp] * measured_b_idx_0 + K[rtemp + 3] *
                        measured_b_idx_1) + K[rtemp + 6] * maxval;
  }

  /*  calculate corrected (aposteriori) error covariance matrix P(+) */
  memset(&S[0], 0, 9U * sizeof(double));
  S[0] = 1.0;
  S[4] = 1.0;
  S[8] = 1.0;
  for (rtemp = 0; rtemp < 9; rtemp++) {
    S[rtemp] -= K[rtemp];
  }

  for (rtemp = 0; rtemp < 3; rtemp++) {
    maxval = S[rtemp];
    a21 = S[rtemp + 3];
    K_tmp = S[rtemp + 6];
    for (e_K_tmp = 0; e_K_tmp < 3; e_K_tmp++) {
      K[rtemp + 3 * e_K_tmp] = (maxval * vel_kf_P[3 * e_K_tmp] + a21 * vel_kf_P
        [3 * e_K_tmp + 1]) + K_tmp * vel_kf_P[3 * e_K_tmp + 2];
    }
  }

  memcpy(&vel_kf_P[0], &K[0], 9U * sizeof(double));
}

/*
 * VEL_KF_PREDICT prediction step (time update)
 * Arguments    : const double lin_accel[3]
 * Return Type  : void
 */
void vel_kf_predict(const double lin_accel[3])
{
  int i;
  if (!isInitialized_vel_kf) {
    vel_kf_initialize();
  }

  /*  state vector x */
  /*  process noise covariance matrix Q */
  /*  error covariance matrix P */
  /*  calculate delta time */
  /*  [s] */
  /*  calculate predicted (apriori) state vector x(-) */
  vel_kf_x[0] += lin_accel[0] * 0.01;
  vel_kf_x[1] += lin_accel[1] * 0.01;
  vel_kf_x[2] += lin_accel[2] * 0.01;

  /*  calculate predicted (apriori) error covariance matrix P(-) */
  for (i = 0; i < 9; i++) {
    vel_kf_P[i] += vel_kf_Q[i];
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void vel_kf_terminate(void)
{
  /* (no terminate code required) */
  isInitialized_vel_kf = false;
}

/*
 * VEL_KF_VEL velocity
 * Arguments    : double vel[3]
 * Return Type  : void
 */
void vel_kf_vel(double vel[3])
{
  if (!isInitialized_vel_kf) {
    vel_kf_initialize();
  }

  /*  state vector x */
  /*  return velocity */
  vel[0] = vel_kf_x[0];
  vel[1] = vel_kf_x[1];
  vel[2] = vel_kf_x[2];
}

/*
 * File trailer for vel_kf.c
 *
 * [EOF]
 */
