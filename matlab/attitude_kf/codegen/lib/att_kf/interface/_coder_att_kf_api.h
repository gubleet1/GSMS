/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_att_kf_api.h
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 25-Nov-2020 21:15:25
 */

#ifndef _CODER_ATT_KF_API_H
#define _CODER_ATT_KF_API_H

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
  real_T angle_between_quat(real_T q1[4], real_T q2[4]);
  void angle_between_quat_api(const mxArray * const prhs[2], const mxArray *
    plhs[1]);
  void att_kf(void);
  void att_kf_api(void);
  void att_kf_atexit(void);
  void att_kf_initialize(void);
  void att_kf_measure(real_T expected_i[3], real_T measured_b[3]);
  void att_kf_measure_api(const mxArray * const prhs[2]);
  void att_kf_predict(real_T w[3]);
  void att_kf_predict_api(const mxArray * const prhs[1]);
  void att_kf_propagate(void);
  void att_kf_propagate_api(void);
  void att_kf_q_ref(real_T q_ref[4]);
  void att_kf_q_ref_api(const mxArray *plhs[1]);
  void att_kf_terminate(void);
  void att_kf_xil_shutdown(void);
  void att_kf_xil_terminate(void);
  void rotate_by_quat(real_T v[3], real_T q[4], real_T v_rot[3]);
  void rotate_by_quat_api(const mxArray * const prhs[2], const mxArray *plhs[1]);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for _coder_att_kf_api.h
 *
 * [EOF]
 */
