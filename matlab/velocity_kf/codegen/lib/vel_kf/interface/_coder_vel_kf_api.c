/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_vel_kf_api.c
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 30-Nov-2020 23:53:28
 */

/* Include Files */
#include "_coder_vel_kf_api.h"
#include "_coder_vel_kf_mex.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;
static real_T vel_kf_x[3];
static uint32_T vel_kf_x_guard;
static real_T vel_kf_Q[9];
static uint32_T vel_kf_Q_guard;
static real_T vel_kf_R[9];
static uint32_T vel_kf_R_guard;
static real_T vel_kf_P[9];
static uint32_T vel_kf_P_guard;
emlrtContext emlrtContextGlobal = { true,/* bFirstTime */
  false,                               /* bInitialized */
  131595U,                             /* fVersionInfo */
  NULL,                                /* fErrorFunction */
  "vel_kf",                            /* fFunctionName */
  NULL,                                /* fRTCallStack */
  false,                               /* bDebugMode */
  { 2045744189U, 2170104910U, 2743257031U, 4284093946U },/* fSigWrd */
  NULL                                 /* fSigMem */
};

/* Function Declarations */
static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[3]);
static const mxArray *b_emlrt_marshallOut(const real_T u[9]);
static void c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *b_vel_kf_Q,
  const char_T *identifier, real_T y[9]);
static const mxArray *c_emlrt_marshallOut(const real_T u[4]);
static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[9]);
static const mxArray *d_emlrt_marshallOut(const real_T u[3]);
static real_T (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *q, const
  char_T *identifier))[4];
static void emlrt_marshallIn(const emlrtStack *sp, const mxArray *b_vel_kf_x,
  const char_T *identifier, real_T y[3]);
static const mxArray *emlrt_marshallOut(const real_T u[3]);
static real_T (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[4];
static real_T (*g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *v, const
  char_T *identifier))[3];
static real_T (*h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[3];
static real_T i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *ang, const
  char_T *identifier);
static real_T j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static void k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[3]);
static void l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[9]);
static real_T (*m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[4];
static real_T (*n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[3];
static real_T o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId);
static void vel_kf_once(void);

/* Function Definitions */
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                real_T y[3]
 * Return Type  : void
 */
static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[3])
{
  k_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const real_T u[9]
 * Return Type  : const mxArray *
 */
static const mxArray *b_emlrt_marshallOut(const real_T u[9])
{
  static const int32_T iv[2] = { 3, 3 };

  const mxArray *m;
  const mxArray *y;
  real_T *pData;
  int32_T b_i;
  int32_T i;
  y = NULL;
  m = emlrtCreateNumericArray(2, &iv[0], mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  i = 0;
  for (b_i = 0; b_i < 3; b_i++) {
    pData[i] = u[3 * b_i];
    i++;
    pData[i] = u[3 * b_i + 1];
    i++;
    pData[i] = u[3 * b_i + 2];
    i++;
  }

  emlrtAssign(&y, m);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *b_vel_kf_Q
 *                const char_T *identifier
 *                real_T y[9]
 * Return Type  : void
 */
static void c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *b_vel_kf_Q,
  const char_T *identifier, real_T y[9])
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  d_emlrt_marshallIn(sp, emlrtAlias(b_vel_kf_Q), &thisId, y);
  emlrtDestroyArray(&b_vel_kf_Q);
}

/*
 * Arguments    : const real_T u[4]
 * Return Type  : const mxArray *
 */
static const mxArray *c_emlrt_marshallOut(const real_T u[4])
{
  static const int32_T iv[1] = { 0 };

  static const int32_T iv1[1] = { 4 };

  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateNumericArray(1, &iv[0], mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, iv1, 1);
  emlrtAssign(&y, m);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                real_T y[9]
 * Return Type  : void
 */
static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[9])
{
  l_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const real_T u[3]
 * Return Type  : const mxArray *
 */
static const mxArray *d_emlrt_marshallOut(const real_T u[3])
{
  static const int32_T iv[1] = { 0 };

  static const int32_T iv1[1] = { 3 };

  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateNumericArray(1, &iv[0], mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, iv1, 1);
  emlrtAssign(&y, m);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *q
 *                const char_T *identifier
 * Return Type  : real_T (*)[4]
 */
static real_T (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *q, const
  char_T *identifier))[4]
{
  emlrtMsgIdentifier thisId;
  real_T (*y)[4];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = f_emlrt_marshallIn(sp, emlrtAlias(q), &thisId);
  emlrtDestroyArray(&q);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *b_vel_kf_x
 *                const char_T *identifier
 *                real_T y[3]
 * Return Type  : void
 */
  static void emlrt_marshallIn(const emlrtStack *sp, const mxArray *b_vel_kf_x,
  const char_T *identifier, real_T y[3])
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  b_emlrt_marshallIn(sp, emlrtAlias(b_vel_kf_x), &thisId, y);
  emlrtDestroyArray(&b_vel_kf_x);
}

/*
 * Arguments    : const real_T u[3]
 * Return Type  : const mxArray *
 */
static const mxArray *emlrt_marshallOut(const real_T u[3])
{
  static const int32_T iv[1] = { 3 };

  const mxArray *m;
  const mxArray *y;
  real_T *pData;
  y = NULL;
  m = emlrtCreateNumericArray(1, &iv[0], mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  pData[0] = u[0];
  pData[1] = u[1];
  pData[2] = u[2];
  emlrtAssign(&y, m);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T (*)[4]
 */
static real_T (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[4]
{
  real_T (*y)[4];
  y = m_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *v
 *                const char_T *identifier
 * Return Type  : real_T (*)[3]
 */
  static real_T (*g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *v,
  const char_T *identifier))[3]
{
  emlrtMsgIdentifier thisId;
  real_T (*y)[3];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = h_emlrt_marshallIn(sp, emlrtAlias(v), &thisId);
  emlrtDestroyArray(&v);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T (*)[3]
 */
static real_T (*h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[3]
{
  real_T (*y)[3];
  y = n_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *ang
 *                const char_T *identifier
 * Return Type  : real_T
 */
  static real_T i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *ang,
  const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  real_T y;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = j_emlrt_marshallIn(sp, emlrtAlias(ang), &thisId);
  emlrtDestroyArray(&ang);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T
 */
static real_T j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = o_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                real_T ret[3]
 * Return Type  : void
 */
static void k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[3])
{
  static const int32_T dims[1] = { 3 };

  real_T (*r)[3];
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 1U, dims);
  r = (real_T (*)[3])emlrtMxGetData(src);
  ret[0] = (*r)[0];
  ret[1] = (*r)[1];
  ret[2] = (*r)[2];
  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                real_T ret[9]
 * Return Type  : void
 */
static void l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[9])
{
  static const int32_T dims[2] = { 3, 3 };

  real_T (*r)[9];
  int32_T i;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, dims);
  r = (real_T (*)[9])emlrtMxGetData(src);
  for (i = 0; i < 9; i++) {
    ret[i] = (*r)[i];
  }

  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T (*)[4]
 */
static real_T (*m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[4]
{
  static const int32_T dims[1] = { 4 };

  real_T (*ret)[4];
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 1U, dims);
  ret = (real_T (*)[4])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T (*)[3]
 */
  static real_T (*n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[3]
{
  static const int32_T dims[1] = { 3 };

  real_T (*ret)[3];
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 1U, dims);
  ret = (real_T (*)[3])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T
 */
static real_T o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId)
{
  static const int32_T dims = 0;
  real_T ret;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 0U, &dims);
  ret = *(real_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
static void vel_kf_once(void)
{
  vel_kf_P_guard = 0U;
  vel_kf_R_guard = 0U;
  vel_kf_Q_guard = 0U;
  vel_kf_x_guard = 0U;
}

/*
 * Arguments    : const emlrtStack *sp
 * Return Type  : void
 */
void MEXGlobalSyncInFunction(const emlrtStack *sp)
{
  const mxArray *tmp;

  /* Marshall in global variables */
  tmp = emlrtGetGlobalVariable("vel_kf_x");
  if (tmp != NULL) {
    emlrt_marshallIn(sp, tmp, "vel_kf_x", vel_kf_x);
    vel_kf_x_guard = 1U;
  }

  tmp = emlrtGetGlobalVariable("vel_kf_Q");
  if (tmp != NULL) {
    c_emlrt_marshallIn(sp, tmp, "vel_kf_Q", vel_kf_Q);
    vel_kf_Q_guard = 1U;
  }

  tmp = emlrtGetGlobalVariable("vel_kf_R");
  if (tmp != NULL) {
    c_emlrt_marshallIn(sp, tmp, "vel_kf_R", vel_kf_R);
    vel_kf_R_guard = 1U;
  }

  tmp = emlrtGetGlobalVariable("vel_kf_P");
  if (tmp != NULL) {
    c_emlrt_marshallIn(sp, tmp, "vel_kf_P", vel_kf_P);
    vel_kf_P_guard = 1U;
  }
}

/*
 * Arguments    : boolean_T skipDirtyCheck
 * Return Type  : void
 */
void MEXGlobalSyncOutFunction(boolean_T skipDirtyCheck)
{
  /* Marshall out global variables */
  if (skipDirtyCheck || (vel_kf_x_guard != 0U)) {
    emlrtPutGlobalVariable("vel_kf_x", emlrt_marshallOut(vel_kf_x));
  }

  if (skipDirtyCheck || (vel_kf_Q_guard != 0U)) {
    emlrtPutGlobalVariable("vel_kf_Q", b_emlrt_marshallOut(vel_kf_Q));
  }

  if (skipDirtyCheck || (vel_kf_R_guard != 0U)) {
    emlrtPutGlobalVariable("vel_kf_R", b_emlrt_marshallOut(vel_kf_R));
  }

  if (skipDirtyCheck || (vel_kf_P_guard != 0U)) {
    emlrtPutGlobalVariable("vel_kf_P", b_emlrt_marshallOut(vel_kf_P));
  }
}

/*
 * Arguments    : const mxArray * const prhs[1]
 *                const mxArray *plhs[1]
 * Return Type  : void
 */
void quat_conj_api(const mxArray * const prhs[1], const mxArray *plhs[1])
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  real_T (*q)[4];
  real_T (*qc)[4];
  st.tls = emlrtRootTLSGlobal;
  qc = (real_T (*)[4])mxMalloc(sizeof(real_T [4]));

  /* Marshall function inputs */
  q = e_emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "q");

  /* Marshall in global variables */
  MEXGlobalSyncInFunction(&st);

  /* Invoke the target function */
  quat_conj(*q, *qc);

  /* Marshall out global variables */
  MEXGlobalSyncOutFunction(true);

  /* Marshall function outputs */
  plhs[0] = c_emlrt_marshallOut(*qc);
}

/*
 * Arguments    : const mxArray * const prhs[2]
 *                const mxArray *plhs[1]
 * Return Type  : void
 */
void rotate_z_by_ang_api(const mxArray * const prhs[2], const mxArray *plhs[1])
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  real_T (*v)[3];
  real_T (*v_rot)[3];
  real_T ang;
  st.tls = emlrtRootTLSGlobal;
  v_rot = (real_T (*)[3])mxMalloc(sizeof(real_T [3]));

  /* Marshall function inputs */
  v = g_emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "v");
  ang = i_emlrt_marshallIn(&st, emlrtAliasP(prhs[1]), "ang");

  /* Marshall in global variables */
  MEXGlobalSyncInFunction(&st);

  /* Invoke the target function */
  rotate_z_by_ang(*v, ang, *v_rot);

  /* Marshall out global variables */
  MEXGlobalSyncOutFunction(true);

  /* Marshall function outputs */
  plhs[0] = d_emlrt_marshallOut(*v_rot);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void vel_kf_api(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Marshall in global variables */
  MEXGlobalSyncInFunction(&st);

  /* Invoke the target function */
  vel_kf();

  /* Marshall out global variables */
  MEXGlobalSyncOutFunction(true);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void vel_kf_atexit(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  vel_kf_xil_terminate();
  vel_kf_xil_shutdown();
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void vel_kf_initialize(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, 0);
  emlrtEnterRtStackR2012b(&st);
  if (emlrtFirstTimeR2012b(emlrtRootTLSGlobal)) {
    vel_kf_once();
  }
}

/*
 * Arguments    : const mxArray * const prhs[1]
 * Return Type  : void
 */
void vel_kf_measure_api(const mxArray * const prhs[1])
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  real_T (*measured_b)[3];
  st.tls = emlrtRootTLSGlobal;

  /* Marshall function inputs */
  measured_b = g_emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "measured_b");

  /* Marshall in global variables */
  MEXGlobalSyncInFunction(&st);

  /* Invoke the target function */
  vel_kf_measure(*measured_b);

  /* Marshall out global variables */
  MEXGlobalSyncOutFunction(true);
}

/*
 * Arguments    : const mxArray * const prhs[1]
 * Return Type  : void
 */
void vel_kf_predict_api(const mxArray * const prhs[1])
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  real_T (*lin_accel)[3];
  st.tls = emlrtRootTLSGlobal;

  /* Marshall function inputs */
  lin_accel = g_emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "lin_accel");

  /* Marshall in global variables */
  MEXGlobalSyncInFunction(&st);

  /* Invoke the target function */
  vel_kf_predict(*lin_accel);

  /* Marshall out global variables */
  MEXGlobalSyncOutFunction(true);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void vel_kf_terminate(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/*
 * Arguments    : const mxArray *plhs[1]
 * Return Type  : void
 */
void vel_kf_vel_api(const mxArray *plhs[1])
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  real_T (*vel)[3];
  st.tls = emlrtRootTLSGlobal;
  vel = (real_T (*)[3])mxMalloc(sizeof(real_T [3]));

  /* Marshall in global variables */
  MEXGlobalSyncInFunction(&st);

  /* Invoke the target function */
  vel_kf_vel(*vel);

  /* Marshall out global variables */
  MEXGlobalSyncOutFunction(true);

  /* Marshall function outputs */
  plhs[0] = d_emlrt_marshallOut(*vel);
}

/*
 * File trailer for _coder_vel_kf_api.c
 *
 * [EOF]
 */
