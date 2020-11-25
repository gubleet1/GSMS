/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: att_kf.c
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 25-Nov-2020 21:15:25
 */

/* Include Files */
#include "att_kf.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Variable Definitions */
double b_att_kf_q_ref[4];
double att_kf_x[6];
double att_kf_Q[36];
double att_kf_R[4];
double att_kf_P[36];
static bool isInitialized_att_kf = false;

/* Function Declarations */
static void PadeApproximantOfDegree(const double A[144], unsigned char m, double
  F[144]);
static double rt_powd_snf(double u0, double u1);

/* Function Definitions */
/*
 * Arguments    : const double A[144]
 *                unsigned char m
 *                double F[144]
 * Return Type  : void
 */
static void PadeApproximantOfDegree(const double A[144], unsigned char m, double
  F[144])
{
  double A2[144];
  double A3[144];
  double A4[144];
  double V[144];
  double b_A4[144];
  double d;
  double s;
  int b_i;
  int b_tmp;
  int i;
  int i1;
  int ix;
  int iy;
  int j;
  int jA;
  int jp1j;
  int k;
  int mmj_tmp;
  signed char ipiv[12];
  signed char i2;
  for (i = 0; i < 12; i++) {
    for (i1 = 0; i1 < 12; i1++) {
      d = 0.0;
      for (iy = 0; iy < 12; iy++) {
        d += A[i + 12 * iy] * A[iy + 12 * i1];
      }

      A2[i + 12 * i1] = d;
    }
  }

  if (m == 3) {
    memcpy(&F[0], &A2[0], 144U * sizeof(double));
    for (k = 0; k < 12; k++) {
      jp1j = k + 12 * k;
      F[jp1j] += 60.0;
    }

    for (i = 0; i < 12; i++) {
      for (i1 = 0; i1 < 12; i1++) {
        d = 0.0;
        for (iy = 0; iy < 12; iy++) {
          d += A[i + 12 * iy] * F[iy + 12 * i1];
        }

        A4[i + 12 * i1] = d;
      }
    }

    for (i = 0; i < 144; i++) {
      F[i] = A4[i];
      V[i] = 12.0 * A2[i];
    }

    d = 120.0;
  } else {
    for (i = 0; i < 12; i++) {
      for (i1 = 0; i1 < 12; i1++) {
        d = 0.0;
        for (iy = 0; iy < 12; iy++) {
          d += A2[i + 12 * iy] * A2[iy + 12 * i1];
        }

        A3[i + 12 * i1] = d;
      }
    }

    if (m == 5) {
      for (i = 0; i < 144; i++) {
        F[i] = A3[i] + 420.0 * A2[i];
      }

      for (k = 0; k < 12; k++) {
        jp1j = k + 12 * k;
        F[jp1j] += 15120.0;
      }

      for (i = 0; i < 12; i++) {
        for (i1 = 0; i1 < 12; i1++) {
          d = 0.0;
          for (iy = 0; iy < 12; iy++) {
            d += A[i + 12 * iy] * F[iy + 12 * i1];
          }

          A4[i + 12 * i1] = d;
        }
      }

      for (i = 0; i < 144; i++) {
        F[i] = A4[i];
        V[i] = 30.0 * A3[i] + 3360.0 * A2[i];
      }

      d = 30240.0;
    } else {
      for (i = 0; i < 12; i++) {
        for (i1 = 0; i1 < 12; i1++) {
          d = 0.0;
          for (iy = 0; iy < 12; iy++) {
            d += A3[i + 12 * iy] * A2[iy + 12 * i1];
          }

          b_A4[i + 12 * i1] = d;
        }
      }

      if (m == 7) {
        for (i = 0; i < 144; i++) {
          F[i] = (b_A4[i] + 1512.0 * A3[i]) + 277200.0 * A2[i];
        }

        for (k = 0; k < 12; k++) {
          jp1j = k + 12 * k;
          F[jp1j] += 8.64864E+6;
        }

        for (i = 0; i < 12; i++) {
          for (i1 = 0; i1 < 12; i1++) {
            d = 0.0;
            for (iy = 0; iy < 12; iy++) {
              d += A[i + 12 * iy] * F[iy + 12 * i1];
            }

            A4[i + 12 * i1] = d;
          }
        }

        for (i = 0; i < 144; i++) {
          F[i] = A4[i];
          V[i] = (56.0 * b_A4[i] + 25200.0 * A3[i]) + 1.99584E+6 * A2[i];
        }

        d = 1.729728E+7;
      } else if (m == 9) {
        for (i = 0; i < 12; i++) {
          for (i1 = 0; i1 < 12; i1++) {
            d = 0.0;
            for (iy = 0; iy < 12; iy++) {
              d += b_A4[i + 12 * iy] * A2[iy + 12 * i1];
            }

            V[i + 12 * i1] = d;
          }
        }

        for (i = 0; i < 144; i++) {
          F[i] = ((V[i] + 3960.0 * b_A4[i]) + 2.16216E+6 * A3[i]) + 3.027024E+8 *
            A2[i];
        }

        for (k = 0; k < 12; k++) {
          jp1j = k + 12 * k;
          F[jp1j] += 8.8216128E+9;
        }

        for (i = 0; i < 12; i++) {
          for (i1 = 0; i1 < 12; i1++) {
            d = 0.0;
            for (iy = 0; iy < 12; iy++) {
              d += A[i + 12 * iy] * F[iy + 12 * i1];
            }

            A4[i + 12 * i1] = d;
          }
        }

        for (i = 0; i < 144; i++) {
          F[i] = A4[i];
          V[i] = ((90.0 * V[i] + 110880.0 * b_A4[i]) + 3.027024E+7 * A3[i]) +
            2.0756736E+9 * A2[i];
        }

        d = 1.76432256E+10;
      } else {
        for (i = 0; i < 144; i++) {
          F[i] = (3.352212864E+10 * b_A4[i] + 1.05594705216E+13 * A3[i]) +
            1.1873537964288E+15 * A2[i];
        }

        for (k = 0; k < 12; k++) {
          jp1j = k + 12 * k;
          F[jp1j] += 3.238237626624E+16;
        }

        for (i = 0; i < 144; i++) {
          A4[i] = (b_A4[i] + 16380.0 * A3[i]) + 4.08408E+7 * A2[i];
        }

        for (i = 0; i < 12; i++) {
          for (i1 = 0; i1 < 12; i1++) {
            d = 0.0;
            for (iy = 0; iy < 12; iy++) {
              d += b_A4[i + 12 * iy] * A4[iy + 12 * i1];
            }

            iy = i + 12 * i1;
            V[iy] = d + F[iy];
          }
        }

        for (i = 0; i < 12; i++) {
          for (i1 = 0; i1 < 12; i1++) {
            d = 0.0;
            for (iy = 0; iy < 12; iy++) {
              d += A[i + 12 * iy] * V[iy + 12 * i1];
            }

            F[i + 12 * i1] = d;
          }
        }

        for (i = 0; i < 144; i++) {
          A4[i] = (182.0 * b_A4[i] + 960960.0 * A3[i]) + 1.32324192E+9 * A2[i];
        }

        for (i = 0; i < 12; i++) {
          for (i1 = 0; i1 < 12; i1++) {
            d = 0.0;
            for (iy = 0; iy < 12; iy++) {
              d += b_A4[i + 12 * iy] * A4[iy + 12 * i1];
            }

            iy = i + 12 * i1;
            V[iy] = ((d + 6.704425728E+11 * b_A4[iy]) + 1.29060195264E+14 *
                     A3[iy]) + 7.7717703038976E+15 * A2[iy];
          }
        }

        d = 6.476475253248E+16;
      }
    }
  }

  for (k = 0; k < 12; k++) {
    iy = k + 12 * k;
    V[iy] += d;
  }

  for (k = 0; k < 144; k++) {
    d = F[k];
    V[k] -= d;
    d *= 2.0;
    F[k] = d;
  }

  for (i = 0; i < 12; i++) {
    ipiv[i] = (signed char)(i + 1);
  }

  for (j = 0; j < 11; j++) {
    mmj_tmp = 10 - j;
    b_tmp = j * 13;
    jp1j = b_tmp + 2;
    iy = 12 - j;
    jA = 0;
    ix = b_tmp;
    d = fabs(V[b_tmp]);
    for (k = 2; k <= iy; k++) {
      ix++;
      s = fabs(V[ix]);
      if (s > d) {
        jA = k - 1;
        d = s;
      }
    }

    if (V[b_tmp + jA] != 0.0) {
      if (jA != 0) {
        iy = j + jA;
        ipiv[j] = (signed char)(iy + 1);
        ix = j;
        for (k = 0; k < 12; k++) {
          d = V[ix];
          V[ix] = V[iy];
          V[iy] = d;
          ix += 12;
          iy += 12;
        }
      }

      i = (b_tmp - j) + 12;
      for (b_i = jp1j; b_i <= i; b_i++) {
        V[b_i - 1] /= V[b_tmp];
      }
    }

    iy = b_tmp + 12;
    jA = b_tmp;
    for (b_i = 0; b_i <= mmj_tmp; b_i++) {
      d = V[iy];
      if (V[iy] != 0.0) {
        ix = b_tmp + 1;
        i = jA + 14;
        i1 = (jA - j) + 24;
        for (jp1j = i; jp1j <= i1; jp1j++) {
          V[jp1j - 1] += V[ix] * -d;
          ix++;
        }
      }

      iy += 12;
      jA += 12;
    }

    i2 = ipiv[j];
    if (i2 != j + 1) {
      for (b_i = 0; b_i < 12; b_i++) {
        iy = j + 12 * b_i;
        d = F[iy];
        jp1j = (i2 + 12 * b_i) - 1;
        F[iy] = F[jp1j];
        F[jp1j] = d;
      }
    }
  }

  for (j = 0; j < 12; j++) {
    iy = 12 * j;
    for (k = 0; k < 12; k++) {
      jA = 12 * k;
      i = k + iy;
      if (F[i] != 0.0) {
        i1 = k + 2;
        for (b_i = i1; b_i < 13; b_i++) {
          jp1j = (b_i + iy) - 1;
          F[jp1j] -= F[i] * V[(b_i + jA) - 1];
        }
      }
    }
  }

  for (j = 0; j < 12; j++) {
    iy = 12 * j;
    for (k = 11; k >= 0; k--) {
      jA = 12 * k;
      i = k + iy;
      d = F[i];
      if (d != 0.0) {
        F[i] = d / V[k + jA];
        for (b_i = 0; b_i < k; b_i++) {
          jp1j = b_i + iy;
          F[jp1j] -= F[i] * V[b_i + jA];
        }
      }
    }
  }

  for (k = 0; k < 12; k++) {
    jp1j = k + 12 * k;
    F[jp1j]++;
  }
}

/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_powd_snf(double u0, double u1)
{
  double d;
  double d1;
  double y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else {
    d = fabs(u0);
    d1 = fabs(u1);
    if (rtIsInf(u1)) {
      if (d == 1.0) {
        y = 1.0;
      } else if (d > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = rtNaN;
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

/*
 * ANGLE_BETWEEN_QUAT angle of rotation between two quaternions
 * Arguments    : const double q1[4]
 *                const double q2[4]
 * Return Type  : double
 */
double angle_between_quat(const double q1[4], const double q2[4])
{
  double v;
  if (!isInitialized_att_kf) {
    att_kf_initialize();
  }

  v = ((q1[0] * q2[0] + q1[1] * q2[1]) + q1[2] * q2[2]) + q1[3] * q2[3];
  v = v * v * 2.0 - 1.0;
  if (v > 1.0) {
    v = 1.0;
  } else {
    if (v < -1.0) {
      v = -1.0;
    }
  }

  return acos(v);
}

/*
 * ATT_KF initialize attitude kalman filter
 * Arguments    : void
 * Return Type  : void
 */
void att_kf(void)
{
  static const double dv[6] = { 2.7415567780803767E-7, 2.7415567780803767E-7,
    2.7415567780803767E-7, 1.3707783890401886E-11, 1.3707783890401886E-11,
    1.3707783890401886E-11 };

  static const double dv1[6] = { 1000.0, 1000.0, 1000.0, 0.0012184696791468343,
    0.0012184696791468343, 0.0012184696791468343 };

  int i;
  if (!isInitialized_att_kf) {
    att_kf_initialize();
  }

  b_att_kf_q_ref[0] = 1.0;
  att_kf_R[0] = 0.0048738787165873371;
  b_att_kf_q_ref[1] = 0.0;
  att_kf_R[1] = 0.0;
  b_att_kf_q_ref[2] = 0.0;
  att_kf_R[2] = 0.0;
  b_att_kf_q_ref[3] = 0.0;
  att_kf_R[3] = 0.0048738787165873371;

  /*  reference attitude quaternion q_ref */
  /*  state vector x */
  /*  process noise covariance matrix Q */
  /*  measurement noise covariance matrix R */
  /*  error covariance matrix P */
  /*  attitude kalman filter constants */
  /*  [rad/s/sqrt(Hz)] */
  /*  [rad/s^2/sqrt(Hz)] */
  /*  [uT/s/sqrt(Hz)], [m/s^3/sqrt(Hz)] */
  /*  [-] */
  /*  [rad/s] */
  /*  initialize kalman filter variables */
  for (i = 0; i < 6; i++) {
    att_kf_x[i] = 0.0;
  }

  memset(&att_kf_Q[0], 0, 36U * sizeof(double));
  for (i = 0; i < 6; i++) {
    att_kf_Q[i + 6 * i] = dv[i];
  }

  memset(&att_kf_P[0], 0, 36U * sizeof(double));
  for (i = 0; i < 6; i++) {
    att_kf_P[i + 6 * i] = dv1[i];
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void att_kf_initialize(void)
{
  int i;
  att_kf_R[0] = 0.0;
  att_kf_R[1] = 0.0;
  att_kf_R[2] = 0.0;
  att_kf_R[3] = 0.0;
  memset(&att_kf_P[0], 0, 36U * sizeof(double));
  memset(&att_kf_Q[0], 0, 36U * sizeof(double));
  for (i = 0; i < 6; i++) {
    att_kf_x[i] = 0.0;
  }

  b_att_kf_q_ref[0] = 0.0;
  b_att_kf_q_ref[1] = 0.0;
  b_att_kf_q_ref[2] = 0.0;
  b_att_kf_q_ref[3] = 0.0;
  isInitialized_att_kf = true;
}

/*
 * ATT_KF_MEASURE measurement step (measurement update)
 * Arguments    : double expected_i[3]
 *                double measured_b[3]
 * Return Type  : void
 */
void att_kf_measure(double expected_i[3], double measured_b[3])
{
  static const double dv[3] = { 0.0, 1.0, 0.0 };

  static const double dv1[3] = { 0.0, 0.0, 1.0 };

  static const double v2[3] = { 1.0, 0.0, 0.0 };

  static const signed char a[6] = { 0, 0, 1, 0, 0, 1 };

  static const signed char iv[3] = { 1, 0, 0 };

  double c_I[36];
  double d_I[36];
  double H[12];
  double K[12];
  double b_H[12];
  double b_b_to_m[9];
  double b_a[6];
  double c_a[6];
  double b_to_m[4];
  double i_to_m[4];
  double m1[3];
  double m2[3];
  double m3[3];
  double a22;
  double absxk;
  double d;
  double d1;
  double s;
  double scale;
  double t;
  double y;
  int K_tmp;
  int b_K_tmp;
  int k;
  int r1;
  int r2;
  signed char b_I[36];
  signed char i;
  signed char i1;
  if (!isInitialized_att_kf) {
    att_kf_initialize();
  }

  /*  reference attitude quaternion q_ref */
  /*  state vector x */
  /*  measurement noise covariance matrix R */
  /*  error covariance matrix P */
  /*  skip the measurement step if the vector measured_b is close to zero */
  scale = 3.3121686421112381E-170;
  absxk = fabs(measured_b[0]);
  if (absxk > 3.3121686421112381E-170) {
    d = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    d = t * t;
  }

  absxk = fabs(measured_b[1]);
  if (absxk > scale) {
    t = scale / absxk;
    d = d * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    d += t * t;
  }

  absxk = fabs(measured_b[2]);
  if (absxk > scale) {
    t = scale / absxk;
    d = d * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    d += t * t;
  }

  d = scale * sqrt(d);
  if (!(d < 0.001)) {
    /*  normalize expected vector in the inertial frame */
    scale = 3.3121686421112381E-170;
    absxk = fabs(expected_i[0]);
    if (absxk > 3.3121686421112381E-170) {
      s = 1.0;
      scale = absxk;
    } else {
      t = absxk / 3.3121686421112381E-170;
      s = t * t;
    }

    absxk = fabs(expected_i[1]);
    if (absxk > scale) {
      t = scale / absxk;
      s = s * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      s += t * t;
    }

    absxk = fabs(expected_i[2]);
    if (absxk > scale) {
      t = scale / absxk;
      s = s * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      s += t * t;
    }

    s = scale * sqrt(s);

    /*  normalize measured vector in the body frame */
    /*  Note: The following steps calculate a rotation matrix to transform the */
    /*        vector measured_b from the body frame into the inertial frame and to */
    /*        rotate it onto the vector m (normal vector of the projection plane). */
    /* QUAT_FROM_VECT quaternion from two 3 element vectors */
    scale = 3.3121686421112381E-170;
    d1 = expected_i[0] / s;
    expected_i[0] = d1;
    measured_b[0] /= d;
    absxk = fabs(d1);
    if (absxk > 3.3121686421112381E-170) {
      y = 1.0;
      scale = absxk;
    } else {
      t = absxk / 3.3121686421112381E-170;
      y = t * t;
    }

    d1 = expected_i[1] / s;
    expected_i[1] = d1;
    measured_b[1] /= d;
    absxk = fabs(d1);
    if (absxk > scale) {
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }

    d1 = expected_i[2] / s;
    expected_i[2] = d1;
    measured_b[2] /= d;
    absxk = fabs(d1);
    if (absxk > scale) {
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }

    y = scale * sqrt(y);
    m2[0] = expected_i[0] / y;
    m2[1] = expected_i[1] / y;
    m2[2] = d1 / y;
    s = (m2[0] + m2[1] * 0.0) + m2[2] * 0.0;
    if (s > 0.99999) {
      i_to_m[0] = 1.0;
      i_to_m[1] = 0.0;
      i_to_m[2] = 0.0;
      i_to_m[3] = 0.0;
    } else if (s < -0.99999) {
      m1[0] = m2[1] - m2[2];
      m1[1] = m2[2] - m2[0];
      m1[2] = m2[0] - m2[1];
      scale = 3.3121686421112381E-170;
      absxk = fabs(m1[0]);
      if (absxk > 3.3121686421112381E-170) {
        s = 1.0;
        scale = absxk;
      } else {
        t = absxk / 3.3121686421112381E-170;
        s = t * t;
      }

      absxk = fabs(m1[1]);
      if (absxk > scale) {
        t = scale / absxk;
        s = s * t * t + 1.0;
        scale = absxk;
      } else {
        t = absxk / scale;
        s += t * t;
      }

      absxk = fabs(m1[2]);
      if (absxk > scale) {
        t = scale / absxk;
        s = s * t * t + 1.0;
        scale = absxk;
      } else {
        t = absxk / scale;
        s += t * t;
      }

      s = scale * sqrt(s);
      i_to_m[0] = 0.0;
      i_to_m[1] = m1[0] / s;
      i_to_m[2] = m1[1] / s;
      i_to_m[3] = m1[2] / s;
    } else {
      s = sqrt((s + 1.0) * 2.0);
      i_to_m[0] = s * 0.5;
      i_to_m[1] = (m2[1] * 0.0 - m2[2] * 0.0) / s;
      i_to_m[2] = (m2[2] - m2[0] * 0.0) / s;
      i_to_m[3] = (m2[0] * 0.0 - m2[1]) / s;
    }

    /* QUAT_MULT quaternion multiplication */
    b_to_m[0] = ((i_to_m[0] * b_att_kf_q_ref[0] - i_to_m[1] * b_att_kf_q_ref[1])
                 - i_to_m[2] * b_att_kf_q_ref[2]) - i_to_m[3] * b_att_kf_q_ref[3];
    b_to_m[1] = ((i_to_m[0] * b_att_kf_q_ref[1] + i_to_m[1] * b_att_kf_q_ref[0])
                 + i_to_m[2] * b_att_kf_q_ref[3]) - i_to_m[3] * b_att_kf_q_ref[2];
    b_to_m[2] = ((i_to_m[0] * b_att_kf_q_ref[2] - i_to_m[1] * b_att_kf_q_ref[3])
                 + i_to_m[2] * b_att_kf_q_ref[0]) + i_to_m[3] * b_att_kf_q_ref[1];
    b_to_m[3] = ((i_to_m[0] * b_att_kf_q_ref[3] + i_to_m[1] * b_att_kf_q_ref[2])
                 - i_to_m[2] * b_att_kf_q_ref[1]) + i_to_m[3] * b_att_kf_q_ref[0];

    /* ROTM_FROM_QUAT rotation matrix from quaternion */
    rotate_by_quat(v2, b_to_m, m1);
    rotate_by_quat(dv, b_to_m, m2);
    rotate_by_quat(dv1, b_to_m, m3);
    b_b_to_m[0] = m1[0];
    b_b_to_m[3] = m2[0];
    b_b_to_m[6] = m3[0];
    b_b_to_m[1] = m1[1];
    b_b_to_m[4] = m2[1];
    b_b_to_m[7] = m3[1];
    b_b_to_m[2] = m1[2];
    b_b_to_m[5] = m2[2];
    b_b_to_m[8] = m3[2];

    /*  rotate vector expected_i from the inertial frame into the body frame */
    /* QUAT_CONJ quaternion conjugate */
    i_to_m[0] = b_att_kf_q_ref[0];
    i_to_m[1] = -b_att_kf_q_ref[1];
    i_to_m[2] = -b_att_kf_q_ref[2];
    i_to_m[3] = -b_att_kf_q_ref[3];
    rotate_by_quat(expected_i, i_to_m, m1);

    /*  Note: The following steps rotate the vector measured_b using the rotation */
    /*        matrix b_to_m and then calculate the measurement vector z by */
    /*        projecting the rotated vector measured_b onto the y, z plane. */
    /*  skip the measurement step if the expected and the measured vector are antiparallel */
    d = 0.0;
    scale = 3.3121686421112381E-170;
    d1 = measured_b[0];
    a22 = measured_b[1];
    y = measured_b[2];
    for (k = 0; k < 3; k++) {
      s = ((b_b_to_m[k] * d1 + b_b_to_m[k + 3] * a22) + b_b_to_m[k + 6] * y) +
        (double)iv[k];
      m2[k] = s;
      absxk = fabs(s);
      if (absxk > scale) {
        t = scale / absxk;
        d = d * t * t + 1.0;
        scale = absxk;
      } else {
        t = absxk / scale;
        d += t * t;
      }
    }

    d = scale * sqrt(d);
    if (!(d < 0.001)) {
      m2[0] /= d;
      m2[1] /= d;
      m2[2] /= d;

      /*  calculate observation matrix H */
      /* CROSS_PROD_MAT cross product matrix of 3 element vector */
      for (K_tmp = 0; K_tmp < 2; K_tmp++) {
        i = a[K_tmp + 2];
        i1 = a[K_tmp + 4];
        for (r2 = 0; r2 < 3; r2++) {
          b_a[K_tmp + (r2 << 1)] = (0.0 * b_b_to_m[3 * r2] + (double)i *
            b_b_to_m[3 * r2 + 1]) + (double)i1 * b_b_to_m[3 * r2 + 2];
        }
      }

      b_b_to_m[0] = 0.0;
      b_b_to_m[3] = -m1[2];
      b_b_to_m[6] = m1[1];
      b_b_to_m[1] = m1[2];
      b_b_to_m[4] = 0.0;
      b_b_to_m[7] = -m1[0];
      b_b_to_m[2] = -m1[1];
      b_b_to_m[5] = m1[0];
      b_b_to_m[8] = 0.0;
      for (K_tmp = 0; K_tmp < 2; K_tmp++) {
        d = b_a[K_tmp];
        d1 = b_a[K_tmp + 2];
        a22 = b_a[K_tmp + 4];
        for (r2 = 0; r2 < 3; r2++) {
          c_a[K_tmp + (r2 << 1)] = (d * b_b_to_m[3 * r2] + d1 * b_b_to_m[3 * r2
            + 1]) + a22 * b_b_to_m[3 * r2 + 2];
        }
      }

      for (K_tmp = 0; K_tmp < 3; K_tmp++) {
        r1 = K_tmp << 1;
        H[r1] = c_a[r1];
        k = (K_tmp + 3) << 1;
        H[k] = 0.0;
        H[r1 + 1] = c_a[r1 + 1];
        H[k + 1] = 0.0;
      }

      /*  calculate measurement innovation vector y */
      /*  calculate kalman gain matrix K */
      for (K_tmp = 0; K_tmp < 2; K_tmp++) {
        for (r2 = 0; r2 < 6; r2++) {
          b_K_tmp = K_tmp + (r2 << 1);
          K[r2 + 6 * K_tmp] = H[b_K_tmp];
          d = 0.0;
          for (r1 = 0; r1 < 6; r1++) {
            d += H[K_tmp + (r1 << 1)] * att_kf_P[r1 + 6 * r2];
          }

          b_H[b_K_tmp] = d;
        }
      }

      for (K_tmp = 0; K_tmp < 2; K_tmp++) {
        for (r2 = 0; r2 < 2; r2++) {
          d = 0.0;
          for (r1 = 0; r1 < 6; r1++) {
            d += b_H[K_tmp + (r1 << 1)] * K[r1 + 6 * r2];
          }

          r1 = K_tmp + (r2 << 1);
          b_to_m[r1] = d + att_kf_R[r1];
        }
      }

      i_to_m[0] = b_to_m[0];
      i_to_m[1] = b_to_m[1];
      i_to_m[2] = b_to_m[2];
      i_to_m[3] = b_to_m[3];
      for (K_tmp = 0; K_tmp < 6; K_tmp++) {
        for (r2 = 0; r2 < 2; r2++) {
          d = 0.0;
          for (r1 = 0; r1 < 6; r1++) {
            d += att_kf_P[K_tmp + 6 * r1] * K[r1 + 6 * r2];
          }

          b_H[K_tmp + 6 * r2] = d;
        }
      }

      if (fabs(b_to_m[1]) > fabs(b_to_m[0])) {
        r1 = 1;
        r2 = 0;
      } else {
        r1 = 0;
        r2 = 1;
      }

      s = i_to_m[r2] / i_to_m[r1];
      y = i_to_m[r1 + 2];
      a22 = i_to_m[r2 + 2] - s * y;
      for (k = 0; k < 6; k++) {
        b_K_tmp = k + 6 * r1;
        K[b_K_tmp] = b_H[k] / i_to_m[r1];
        K_tmp = k + 6 * r2;
        K[K_tmp] = (b_H[k + 6] - K[b_K_tmp] * y) / a22;
        K[b_K_tmp] -= K[K_tmp] * s;
      }

      /*  calculate corrected (aposteriori) state vector x(+) */
      s = m2[1] / m2[0] * 2.0;
      y = m2[2] / m2[0] * 2.0;
      for (K_tmp = 0; K_tmp < 6; K_tmp++) {
        att_kf_x[K_tmp] += K[K_tmp] * s + K[K_tmp + 6] * y;
      }

      /*  calculate corrected (aposteriori) error covariance matrix P(+) */
      for (K_tmp = 0; K_tmp < 36; K_tmp++) {
        b_I[K_tmp] = 0;
      }

      for (k = 0; k < 6; k++) {
        b_I[k + 6 * k] = 1;
      }

      for (K_tmp = 0; K_tmp < 6; K_tmp++) {
        d = K[K_tmp];
        d1 = K[K_tmp + 6];
        for (r2 = 0; r2 < 6; r2++) {
          r1 = r2 << 1;
          k = K_tmp + 6 * r2;
          d_I[k] = (double)b_I[k] - (d * H[r1] + d1 * H[r1 + 1]);
        }

        for (r2 = 0; r2 < 6; r2++) {
          d = 0.0;
          for (r1 = 0; r1 < 6; r1++) {
            d += d_I[K_tmp + 6 * r1] * att_kf_P[r1 + 6 * r2];
          }

          c_I[K_tmp + 6 * r2] = d;
        }
      }

      memcpy(&att_kf_P[0], &c_I[0], 36U * sizeof(double));
    }
  }
}

/*
 * ATT_KF_PREDICT prediction step (time update)
 * Arguments    : const double w[3]
 * Return Type  : void
 */
void att_kf_predict(const double w[3])
{
  static const double theta[5] = { 0.01495585217958292, 0.253939833006323,
    0.95041789961629319, 2.097847961257068, 5.3719203511481517 };

  static const signed char iv[9] = { -1, 0, 0, 0, -1, 0, 0, 0, -1 };

  static const unsigned char uv[5] = { 3U, 5U, 7U, 9U, 13U };

  double B[144];
  double y[144];
  double F[36];
  double b_F[36];
  double c_F[36];
  double omega[3];
  double absxk;
  double ang_tmp;
  double d;
  double d1;
  double d10;
  double d11;
  double d12;
  double d2;
  double d3;
  double d4;
  double d5;
  double d6;
  double d7;
  double d8;
  double d9;
  double delta_q_ref_idx_0;
  double delta_q_ref_idx_1;
  double delta_q_ref_idx_2;
  double scale;
  double t;
  int F_tmp;
  int eint;
  int i;
  int i1;
  int j;
  bool exitg1;
  if (!isInitialized_att_kf) {
    att_kf_initialize();
  }

  /*  reference attitude quaternion q_ref */
  /*  state vector x */
  /*  process noise covariance matrix Q */
  /*  error covariance matrix P */
  /*  calculate delta time */
  /*  [s] */
  /*  remove gyroscope bias */
  /*  calculate angle of rotation */
  scale = 3.3121686421112381E-170;
  d = w[0] - att_kf_x[3];
  omega[0] = d;
  absxk = fabs(d);
  if (absxk > 3.3121686421112381E-170) {
    ang_tmp = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    ang_tmp = t * t;
  }

  d = w[1] - att_kf_x[4];
  omega[1] = d;
  absxk = fabs(d);
  if (absxk > scale) {
    t = scale / absxk;
    ang_tmp = ang_tmp * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    ang_tmp += t * t;
  }

  d = w[2] - att_kf_x[5];
  omega[2] = d;
  absxk = fabs(d);
  if (absxk > scale) {
    t = scale / absxk;
    ang_tmp = ang_tmp * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    ang_tmp += t * t;
  }

  ang_tmp = scale * sqrt(ang_tmp);
  scale = ang_tmp * 0.01;

  /*  calculate delta quaternion */
  if (scale > 1.0E-6) {
    /*  calculate delata quaternion from angle and axis of rotation */
    absxk = sin(scale / 2.0);
    delta_q_ref_idx_0 = cos(scale / 2.0);
    delta_q_ref_idx_1 = omega[0] / ang_tmp * absxk;
    delta_q_ref_idx_2 = omega[1] / ang_tmp * absxk;
    scale = d / ang_tmp * absxk;
  } else {
    /*  calculate delta quaternion using small angle approximation */
    delta_q_ref_idx_0 = 1.0;
    delta_q_ref_idx_1 = omega[0] * 0.01 / 2.0;
    delta_q_ref_idx_2 = omega[1] * 0.01 / 2.0;
    scale = d * 0.01 / 2.0;
  }

  /*  predict q_ref */
  /* QUAT_MULT quaternion multiplication */
  d = b_att_kf_q_ref[0];
  absxk = b_att_kf_q_ref[1];
  t = b_att_kf_q_ref[2];
  ang_tmp = b_att_kf_q_ref[3];
  d1 = b_att_kf_q_ref[0];
  d2 = b_att_kf_q_ref[1];
  d3 = b_att_kf_q_ref[2];
  d4 = b_att_kf_q_ref[3];
  d5 = b_att_kf_q_ref[0];
  d6 = b_att_kf_q_ref[1];
  d7 = b_att_kf_q_ref[2];
  d8 = b_att_kf_q_ref[3];
  d9 = b_att_kf_q_ref[0];
  d10 = b_att_kf_q_ref[1];
  d11 = b_att_kf_q_ref[2];
  d12 = b_att_kf_q_ref[3];
  b_att_kf_q_ref[0] = ((d * delta_q_ref_idx_0 - absxk * delta_q_ref_idx_1) - t *
                       delta_q_ref_idx_2) - ang_tmp * scale;
  b_att_kf_q_ref[1] = ((d1 * delta_q_ref_idx_1 + d2 * delta_q_ref_idx_0) + d3 *
                       scale) - d4 * delta_q_ref_idx_2;
  b_att_kf_q_ref[2] = ((d5 * delta_q_ref_idx_2 - d6 * scale) + d7 *
                       delta_q_ref_idx_0) + d8 * delta_q_ref_idx_1;
  b_att_kf_q_ref[3] = ((d9 * scale + d10 * delta_q_ref_idx_2) - d11 *
                       delta_q_ref_idx_1) + d12 * delta_q_ref_idx_0;

  /*  calculate linearized state transition matrix Phi */
  /*  Note: The EKF linearizes the underlying model to produce matrix Phi. */
  /* CROSS_PROD_MAT cross product matrix of 3 element vector */
  F[0] = 0.0;
  F[7] = 0.0;
  F[14] = 0.0;
  for (i = 0; i < 3; i++) {
    omega[i] = -omega[i];
    F_tmp = 6 * (i + 3);
    F[F_tmp] = iv[3 * i];
    F[F_tmp + 1] = iv[3 * i + 1];
    F[F_tmp + 2] = iv[3 * i + 2];
  }

  F[6] = -omega[2];
  F[12] = omega[1];
  F[1] = omega[2];
  F[13] = -omega[0];
  F[2] = -omega[1];
  F[8] = omega[0];
  for (i = 0; i < 6; i++) {
    F[6 * i + 3] = 0.0;
    F[6 * i + 4] = 0.0;
    F[6 * i + 5] = 0.0;
    for (i1 = 0; i1 < 6; i1++) {
      F_tmp = i1 + 12 * i;
      y[F_tmp] = -F[i1 + 6 * i] * 0.01;
      y[i1 + 12 * (i + 6)] = att_kf_Q[i + 6 * i1] * 0.01;
      y[F_tmp + 6] = 0.0;
    }
  }

  for (i = 0; i < 6; i++) {
    for (i1 = 0; i1 < 6; i1++) {
      y[(i1 + 12 * (i + 6)) + 6] = F[i + 6 * i1] * 0.01;
    }
  }

  scale = 0.0;
  j = 0;
  exitg1 = false;
  while ((!exitg1) && (j < 12)) {
    absxk = 0.0;
    for (F_tmp = 0; F_tmp < 12; F_tmp++) {
      absxk += fabs(y[F_tmp + 12 * j]);
    }

    if (rtIsNaN(absxk)) {
      scale = rtNaN;
      exitg1 = true;
    } else {
      if (absxk > scale) {
        scale = absxk;
      }

      j++;
    }
  }

  if (scale <= 5.3719203511481517) {
    F_tmp = 0;
    exitg1 = false;
    while ((!exitg1) && (F_tmp < 5)) {
      if (scale <= theta[F_tmp]) {
        PadeApproximantOfDegree(y, uv[F_tmp], B);
        exitg1 = true;
      } else {
        F_tmp++;
      }
    }
  } else {
    scale /= 5.3719203511481517;
    if ((!rtIsInf(scale)) && (!rtIsNaN(scale))) {
      scale = frexp(scale, &eint);
    } else {
      eint = 0;
    }

    absxk = eint;
    if (scale == 0.5) {
      absxk = (double)eint - 1.0;
    }

    scale = rt_powd_snf(2.0, absxk);
    for (i = 0; i < 144; i++) {
      y[i] /= scale;
    }

    PadeApproximantOfDegree(y, 13U, B);
    i = (int)absxk;
    for (j = 0; j < i; j++) {
      for (i1 = 0; i1 < 12; i1++) {
        for (F_tmp = 0; F_tmp < 12; F_tmp++) {
          d = 0.0;
          for (eint = 0; eint < 12; eint++) {
            d += B[i1 + 12 * eint] * B[eint + 12 * F_tmp];
          }

          y[i1 + 12 * F_tmp] = d;
        }
      }

      memcpy(&B[0], &y[0], 144U * sizeof(double));
    }
  }

  for (i = 0; i < 6; i++) {
    for (i1 = 0; i1 < 6; i1++) {
      F[i1 + 6 * i] = B[(i + 12 * (i1 + 6)) + 6];
    }
  }

  /*  calcualte linearized process noise covariance matrix Qs */
  /*  Note: The EKF linearizes the underlying model to produce matrix Qs. */
  /*  Note: The attitude error is transferred to q_ref after every iteration. */
  /*        This ensures, that the predicted (apriori) state vector x(-) is */
  /*        equal to the state vector x of the previous iteration. */
  /*  calculate predicted (apriori) error covariance matrix P(-) */
  for (i = 0; i < 6; i++) {
    for (i1 = 0; i1 < 6; i1++) {
      d = 0.0;
      for (F_tmp = 0; F_tmp < 6; F_tmp++) {
        d += F[i + 6 * F_tmp] * att_kf_P[F_tmp + 6 * i1];
      }

      c_F[i + 6 * i1] = d;
    }

    for (i1 = 0; i1 < 6; i1++) {
      d = 0.0;
      for (F_tmp = 0; F_tmp < 6; F_tmp++) {
        d += c_F[i + 6 * F_tmp] * F[i1 + 6 * F_tmp];
      }

      b_F[i + 6 * i1] = d;
    }

    for (i1 = 0; i1 < 6; i1++) {
      d = 0.0;
      for (F_tmp = 0; F_tmp < 6; F_tmp++) {
        d += F[i + 6 * F_tmp] * B[F_tmp + 12 * (i1 + 6)];
      }

      c_F[i + 6 * i1] = d;
    }
  }

  for (i = 0; i < 36; i++) {
    att_kf_P[i] = b_F[i] + c_F[i];
  }
}

/*
 * ATT_KF_PROPAGATE propagate attitude error
 * Arguments    : void
 * Return Type  : void
 */
void att_kf_propagate(void)
{
  double absxk;
  double d;
  double d1;
  double d10;
  double d11;
  double d2;
  double d3;
  double d4;
  double d5;
  double d6;
  double d7;
  double d8;
  double d9;
  double scale;
  double t;
  double y;
  if (!isInitialized_att_kf) {
    att_kf_initialize();
  }

  /*  reference attitude quaternion q_ref */
  /*  state vector x */
  /*  calculate rotation quaternion from attitude error vector a */
  /*  propagate attitude error to reference attitude quaternion q_ref */
  /* QUAT_MULT quaternion multiplication */
  scale = b_att_kf_q_ref[0];
  absxk = b_att_kf_q_ref[1];
  t = b_att_kf_q_ref[2];
  y = b_att_kf_q_ref[3];
  d = b_att_kf_q_ref[0];
  d1 = b_att_kf_q_ref[1];
  d2 = b_att_kf_q_ref[2];
  d3 = b_att_kf_q_ref[3];
  d4 = b_att_kf_q_ref[0];
  d5 = b_att_kf_q_ref[1];
  d6 = b_att_kf_q_ref[2];
  d7 = b_att_kf_q_ref[3];
  d8 = b_att_kf_q_ref[0];
  d9 = b_att_kf_q_ref[1];
  d10 = b_att_kf_q_ref[2];
  d11 = b_att_kf_q_ref[3];
  b_att_kf_q_ref[0] = ((scale * 2.0 - absxk * att_kf_x[0]) - t * att_kf_x[1]) -
    y * att_kf_x[2];
  b_att_kf_q_ref[1] = ((d * att_kf_x[0] + d1 * 2.0) + d2 * att_kf_x[2]) - d3 *
    att_kf_x[1];
  b_att_kf_q_ref[2] = ((d4 * att_kf_x[1] - d5 * att_kf_x[2]) + d6 * 2.0) + d7 *
    att_kf_x[0];
  b_att_kf_q_ref[3] = ((d8 * att_kf_x[2] + d9 * att_kf_x[1]) - d10 * att_kf_x[0])
    + d11 * 2.0;

  /*  normalize reference attitude quaternion q_ref */
  scale = 3.3121686421112381E-170;
  absxk = fabs(b_att_kf_q_ref[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }

  absxk = fabs(b_att_kf_q_ref[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = fabs(b_att_kf_q_ref[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = fabs(b_att_kf_q_ref[3]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  y = scale * sqrt(y);
  b_att_kf_q_ref[0] /= y;
  b_att_kf_q_ref[1] /= y;
  b_att_kf_q_ref[2] /= y;
  b_att_kf_q_ref[3] /= y;

  /*  reset attitude error vector a */
  att_kf_x[0] = 0.0;
  att_kf_x[1] = 0.0;
  att_kf_x[2] = 0.0;
}

/*
 * ATT_KF_Q_REF reference attitude quaternion
 * Arguments    : double q_ref[4]
 * Return Type  : void
 */
void att_kf_q_ref(double q_ref[4])
{
  if (!isInitialized_att_kf) {
    att_kf_initialize();
  }

  /*  reference attitude quaternion q_ref */
  /*  return reference attitude quaternion */
  q_ref[0] = b_att_kf_q_ref[0];
  q_ref[1] = b_att_kf_q_ref[1];
  q_ref[2] = b_att_kf_q_ref[2];
  q_ref[3] = b_att_kf_q_ref[3];
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void att_kf_terminate(void)
{
  /* (no terminate code required) */
  isInitialized_att_kf = false;
}

/*
 * ROTATE_BY_QUAT rotate 3 element vector by quaternion
 * Arguments    : const double v[3]
 *                const double q[4]
 *                double v_rot[3]
 * Return Type  : void
 */
void rotate_by_quat(const double v[3], const double q[4], double v_rot[3])
{
  double q_idx_0;
  double q_idx_1;
  double q_idx_2;
  double q_idx_3;
  if (!isInitialized_att_kf) {
    att_kf_initialize();
  }

  /* QUAT_MULT quaternion multiplication */
  q_idx_0 = ((q[0] * 0.0 - q[1] * v[0]) - q[2] * v[1]) - q[3] * v[2];
  q_idx_1 = ((q[0] * v[0] + q[1] * 0.0) + q[2] * v[2]) - q[3] * v[1];
  q_idx_2 = ((q[0] * v[1] - q[1] * v[2]) + q[2] * 0.0) + q[3] * v[0];
  q_idx_3 = ((q[0] * v[2] + q[1] * v[1]) - q[2] * v[0]) + q[3] * 0.0;

  /* QUAT_CONJ quaternion conjugate */
  /* QUAT_MULT quaternion multiplication */
  v_rot[0] = ((q_idx_0 * -q[1] + q_idx_1 * q[0]) + q_idx_2 * -q[3]) - q_idx_3 *
    -q[2];
  v_rot[1] = ((q_idx_0 * -q[2] - q_idx_1 * -q[3]) + q_idx_2 * q[0]) + q_idx_3 *
    -q[1];
  v_rot[2] = ((q_idx_0 * -q[3] + q_idx_1 * -q[2]) - q_idx_2 * -q[1]) + q_idx_3 *
    q[0];
}

/*
 * File trailer for att_kf.c
 *
 * [EOF]
 */
