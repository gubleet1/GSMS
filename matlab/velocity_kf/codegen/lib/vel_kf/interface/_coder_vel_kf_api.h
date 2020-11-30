/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_vel_kf_api.h
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 30-Nov-2020 23:53:28
 */

#ifndef _CODER_VEL_KF_API_H
#define _CODER_VEL_KF_API_H

/* Include Files */
#include "emlrt.h"
#include "tmwtypes.h"
#include <string.h>

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

#ifdef __cplusplus

extern "C" {

#endif

  /* Function Declarations */
  void MEXGlobalSyncInFunction(const emlrtStack *sp);
  void MEXGlobalSyncOutFunction(boolean_T skipDirtyCheck);
  void quat_conj(real_T q[4], real_T qc[4]);
  void quat_conj_api(const mxArray * const prhs[1], const mxArray *plhs[1]);
  void rotate_z_by_ang(real_T v[3], real_T ang, real_T v_rot[3]);
  void rotate_z_by_ang_api(const mxArray * const prhs[2], const mxArray *plhs[1]);
  void vel_kf(void);
  void vel_kf_api(void);
  void vel_kf_atexit(void);
  void vel_kf_initialize(void);
  void vel_kf_measure(real_T measured_b[3]);
  void vel_kf_measure_api(const mxArray * const prhs[1]);
  void vel_kf_predict(real_T lin_accel[3]);
  void vel_kf_predict_api(const mxArray * const prhs[1]);
  void vel_kf_terminate(void);
  void vel_kf_vel(real_T vel[3]);
  void vel_kf_vel_api(const mxArray *plhs[1]);
  void vel_kf_xil_shutdown(void);
  void vel_kf_xil_terminate(void);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for _coder_vel_kf_api.h
 *
 * [EOF]
 */
