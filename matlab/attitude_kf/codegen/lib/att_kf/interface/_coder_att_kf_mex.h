/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_att_kf_mex.h
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 25-Nov-2020 21:15:25
 */

#ifndef _CODER_ATT_KF_MEX_H
#define _CODER_ATT_KF_MEX_H

/* Include Files */
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"
#ifdef __cplusplus

extern "C" {

#endif

  /* Function Declarations */
  void angle_between_quat_mexFunction(int32_T nlhs, mxArray *plhs[1], int32_T
    nrhs, const mxArray *prhs[2]);
  void att_kf_measure_mexFunction(int32_T nlhs, int32_T nrhs, const mxArray
    *prhs[2]);
  void att_kf_mexFunction(int32_T nlhs, int32_T nrhs);
  void att_kf_predict_mexFunction(int32_T nlhs, int32_T nrhs, const mxArray
    *prhs[1]);
  void att_kf_propagate_mexFunction(int32_T nlhs, int32_T nrhs);
  void att_kf_q_ref_mexFunction(int32_T nlhs, mxArray *plhs[1], int32_T nrhs);
  MEXFUNCTION_LINKAGE void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T
    nrhs, const mxArray *prhs[]);
  emlrtCTX mexFunctionCreateRootTLS(void);
  void rotate_by_quat_mexFunction(int32_T nlhs, mxArray *plhs[1], int32_T nrhs,
    const mxArray *prhs[2]);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for _coder_att_kf_mex.h
 *
 * [EOF]
 */
