/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_vel_kf_mex.h
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 30-Nov-2020 23:53:28
 */

#ifndef _CODER_VEL_KF_MEX_H
#define _CODER_VEL_KF_MEX_H

/* Include Files */
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"
#ifdef __cplusplus

extern "C" {

#endif

  /* Function Declarations */
  MEXFUNCTION_LINKAGE void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T
    nrhs, const mxArray *prhs[]);
  emlrtCTX mexFunctionCreateRootTLS(void);
  void quat_conj_mexFunction(int32_T nlhs, mxArray *plhs[1], int32_T nrhs, const
    mxArray *prhs[1]);
  void rotate_z_by_ang_mexFunction(int32_T nlhs, mxArray *plhs[1], int32_T nrhs,
    const mxArray *prhs[2]);
  void vel_kf_measure_mexFunction(int32_T nlhs, int32_T nrhs, const mxArray
    *prhs[1]);
  void vel_kf_mexFunction(int32_T nlhs, int32_T nrhs);
  void vel_kf_predict_mexFunction(int32_T nlhs, int32_T nrhs, const mxArray
    *prhs[1]);
  void vel_kf_vel_mexFunction(int32_T nlhs, mxArray *plhs[1], int32_T nrhs);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for _coder_vel_kf_mex.h
 *
 * [EOF]
 */
