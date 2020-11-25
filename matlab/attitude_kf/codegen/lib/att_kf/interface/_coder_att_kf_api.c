/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_att_kf_api.c
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 25-Nov-2020 21:15:25
 */

/* Include Files */
#include "_coder_att_kf_api.h"
#include "_coder_att_kf_mex.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;
static real_T b_att_kf_q_ref[4];
static uint32_T att_kf_q_ref_guard;
static real_T att_kf_x[6];
static uint32_T att_kf_x_guard;
static real_T att_kf_Q[36];
static uint32_T att_kf_Q_guard;
static real_T att_kf_R[4];
static uint32_T att_kf_R_guard;
static real_T att_kf_P[36];
static uint32_T att_kf_P_guard;
emlrtContext emlrtContextGlobal = { true,/* bFirstTime */
  false,                               /* bInitialized */
  131595U,                             /* fVersionInfo */
  NULL,                                /* fErrorFunction */
  "att_kf",                            /* fFunctionName */
  NULL,                                /* fRTCallStack */
  false,                               /* bDebugMode */
  { 2045744189U, 2170104910U, 2743257031U, 4284093946U },/* fSigWrd */
  NULL                                 /* fSigMem */
};

/* Function Declarations */
static void att_kf_once(void);
static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[4]);
static const mxArray *b_emlrt_marshallOut(const real_T u[6]);
static void c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *b_att_kf_x,
  const char_T *identifier, real_T y[6]);
static const mxArray *c_emlrt_marshallOut(const real_T u[36]);
static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[6]);
static const mxArray *d_emlrt_marshallOut(const real_T u[4]);
static void e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *b_att_kf_Q,
  const char_T *identifier, real_T y[36]);
static const mxArray *e_emlrt_marshallOut(const real_T u);
static void emlrt_marshallIn(const emlrtStack *sp, const mxArray *c_att_kf_q_ref,
  const char_T *identifier, real_T y[4]);
static const mxArray *emlrt_marshallOut(const real_T u[4]);
static void f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[36]);
static const mxArray *f_emlrt_marshallOut(const real_T u[4]);
static void g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *b_att_kf_R,
  const char_T *identifier, real_T y[4]);
static const mxArray *g_emlrt_marshallOut(const real_T u[3]);
static void h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[4]);
static real_T (*i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *q1,
  const char_T *identifier))[4];
static real_T (*j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[4];
static real_T (*k_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *expected_i, const char_T *identifier))[3];
static real_T (*l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[3];
static void m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[4]);
static void n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[6]);
static void o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[36]);
static void p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[4]);
static real_T (*q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[4];
static real_T (*r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[3];

/* Function Definitions */
/*
 * Arguments    : void
 * Return Type  : void
 */
static void att_kf_once(void)
{
  att_kf_P_guard = 0U;
  att_kf_R_guard = 0U;
  att_kf_Q_guard = 0U;
  att_kf_x_guard = 0U;
  att_kf_q_ref_guard = 0U;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                real_T y[4]
 * Return Type  : void
 */
static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[4])
{
  m_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const real_T u[6]
 * Return Type  : const mxArray *
 */
static const mxArray *b_emlrt_marshallOut(const real_T u[6])
{
  static const int32_T iv[1] = { 6 };

  const mxArray *m;
  const mxArray *y;
  real_T *pData;
  int32_T b_i;
  int32_T i;
  y = NULL;
  m = emlrtCreateNumericArray(1, &iv[0], mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  i = 0;
  for (b_i = 0; b_i < 6; b_i++) {
    pData[i] = u[b_i];
    i++;
  }

  emlrtAssign(&y, m);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *b_att_kf_x
 *                const char_T *identifier
 *                real_T y[6]
 * Return Type  : void
 */
static void c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *b_att_kf_x,
  const char_T *identifier, real_T y[6])
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  d_emlrt_marshallIn(sp, emlrtAlias(b_att_kf_x), &thisId, y);
  emlrtDestroyArray(&b_att_kf_x);
}

/*
 * Arguments    : const real_T u[36]
 * Return Type  : const mxArray *
 */
static const mxArray *c_emlrt_marshallOut(const real_T u[36])
{
  static const int32_T iv[2] = { 6, 6 };

  const mxArray *m;
  const mxArray *y;
  real_T *pData;
  int32_T b_i;
  int32_T c_i;
  int32_T i;
  y = NULL;
  m = emlrtCreateNumericArray(2, &iv[0], mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  i = 0;
  for (b_i = 0; b_i < 6; b_i++) {
    for (c_i = 0; c_i < 6; c_i++) {
      pData[i] = u[c_i + 6 * b_i];
      i++;
    }
  }

  emlrtAssign(&y, m);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                real_T y[6]
 * Return Type  : void
 */
static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[6])
{
  n_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const real_T u[4]
 * Return Type  : const mxArray *
 */
static const mxArray *d_emlrt_marshallOut(const real_T u[4])
{
  static const int32_T iv[2] = { 2, 2 };

  const mxArray *m;
  const mxArray *y;
  real_T *pData;
  y = NULL;
  m = emlrtCreateNumericArray(2, &iv[0], mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  pData[0] = u[0];
  pData[1] = u[1];
  pData[2] = u[2];
  pData[3] = u[3];
  emlrtAssign(&y, m);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *b_att_kf_Q
 *                const char_T *identifier
 *                real_T y[36]
 * Return Type  : void
 */
static void e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *b_att_kf_Q,
  const char_T *identifier, real_T y[36])
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  f_emlrt_marshallIn(sp, emlrtAlias(b_att_kf_Q), &thisId, y);
  emlrtDestroyArray(&b_att_kf_Q);
}

/*
 * Arguments    : const real_T u
 * Return Type  : const mxArray *
 */
static const mxArray *e_emlrt_marshallOut(const real_T u)
{
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateDoubleScalar(u);
  emlrtAssign(&y, m);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *c_att_kf_q_ref
 *                const char_T *identifier
 *                real_T y[4]
 * Return Type  : void
 */
static void emlrt_marshallIn(const emlrtStack *sp, const mxArray *c_att_kf_q_ref,
  const char_T *identifier, real_T y[4])
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  b_emlrt_marshallIn(sp, emlrtAlias(c_att_kf_q_ref), &thisId, y);
  emlrtDestroyArray(&c_att_kf_q_ref);
}

/*
 * Arguments    : const real_T u[4]
 * Return Type  : const mxArray *
 */
static const mxArray *emlrt_marshallOut(const real_T u[4])
{
  static const int32_T iv[1] = { 4 };

  const mxArray *m;
  const mxArray *y;
  real_T *pData;
  y = NULL;
  m = emlrtCreateNumericArray(1, &iv[0], mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  pData[0] = u[0];
  pData[1] = u[1];
  pData[2] = u[2];
  pData[3] = u[3];
  emlrtAssign(&y, m);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                real_T y[36]
 * Return Type  : void
 */
static void f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[36])
{
  o_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const real_T u[4]
 * Return Type  : const mxArray *
 */
static const mxArray *f_emlrt_marshallOut(const real_T u[4])
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
 *                const mxArray *b_att_kf_R
 *                const char_T *identifier
 *                real_T y[4]
 * Return Type  : void
 */
static void g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *b_att_kf_R,
  const char_T *identifier, real_T y[4])
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  h_emlrt_marshallIn(sp, emlrtAlias(b_att_kf_R), &thisId, y);
  emlrtDestroyArray(&b_att_kf_R);
}

/*
 * Arguments    : const real_T u[3]
 * Return Type  : const mxArray *
 */
static const mxArray *g_emlrt_marshallOut(const real_T u[3])
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
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                real_T y[4]
 * Return Type  : void
 */
static void h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[4])
{
  p_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *q1
 *                const char_T *identifier
 * Return Type  : real_T (*)[4]
 */
static real_T (*i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *q1,
  const char_T *identifier))[4]
{
  emlrtMsgIdentifier thisId;
  real_T (*y)[4];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = j_emlrt_marshallIn(sp, emlrtAlias(q1), &thisId);
  emlrtDestroyArray(&q1);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T (*)[4]
 */
  static real_T (*j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId))[4]
{
  real_T (*y)[4];
  y = q_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *expected_i
 *                const char_T *identifier
 * Return Type  : real_T (*)[3]
 */
static real_T (*k_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *expected_i, const char_T *identifier))[3]
{
  emlrtMsgIdentifier thisId;
  real_T (*y)[3];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = l_emlrt_marshallIn(sp, emlrtAlias(expected_i), &thisId);
  emlrtDestroyArray(&expected_i);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T (*)[3]
 */
  static real_T (*l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId))[3]
{
  real_T (*y)[3];
  y = r_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                real_T ret[4]
 * Return Type  : void
 */
static void m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[4])
{
  static const int32_T dims[1] = { 4 };

  real_T (*r)[4];
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 1U, dims);
  r = (real_T (*)[4])emlrtMxGetData(src);
  ret[0] = (*r)[0];
  ret[1] = (*r)[1];
  ret[2] = (*r)[2];
  ret[3] = (*r)[3];
  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                real_T ret[6]
 * Return Type  : void
 */
static void n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[6])
{
  static const int32_T dims[1] = { 6 };

  real_T (*r)[6];
  int32_T i;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 1U, dims);
  r = (real_T (*)[6])emlrtMxGetData(src);
  for (i = 0; i < 6; i++) {
    ret[i] = (*r)[i];
  }

  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                real_T ret[36]
 * Return Type  : void
 */
static void o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[36])
{
  static const int32_T dims[2] = { 6, 6 };

  real_T (*r)[36];
  int32_T i;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, dims);
  r = (real_T (*)[36])emlrtMxGetData(src);
  for (i = 0; i < 36; i++) {
    ret[i] = (*r)[i];
  }

  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                real_T ret[4]
 * Return Type  : void
 */
static void p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[4])
{
  static const int32_T dims[2] = { 2, 2 };

  real_T (*r)[4];
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, dims);
  r = (real_T (*)[4])emlrtMxGetData(src);
  ret[0] = (*r)[0];
  ret[1] = (*r)[1];
  ret[2] = (*r)[2];
  ret[3] = (*r)[3];
  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T (*)[4]
 */
static real_T (*q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
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
  static real_T (*r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
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
 * Return Type  : void
 */
void MEXGlobalSyncInFunction(const emlrtStack *sp)
{
  const mxArray *tmp;

  /* Marshall in global variables */
  tmp = emlrtGetGlobalVariable("att_kf_q_ref");
  if (tmp != NULL) {
    emlrt_marshallIn(sp, tmp, "att_kf_q_ref", b_att_kf_q_ref);
    att_kf_q_ref_guard = 1U;
  }

  tmp = emlrtGetGlobalVariable("att_kf_x");
  if (tmp != NULL) {
    c_emlrt_marshallIn(sp, tmp, "att_kf_x", att_kf_x);
    att_kf_x_guard = 1U;
  }

  tmp = emlrtGetGlobalVariable("att_kf_Q");
  if (tmp != NULL) {
    e_emlrt_marshallIn(sp, tmp, "att_kf_Q", att_kf_Q);
    att_kf_Q_guard = 1U;
  }

  tmp = emlrtGetGlobalVariable("att_kf_R");
  if (tmp != NULL) {
    g_emlrt_marshallIn(sp, tmp, "att_kf_R", att_kf_R);
    att_kf_R_guard = 1U;
  }

  tmp = emlrtGetGlobalVariable("att_kf_P");
  if (tmp != NULL) {
    e_emlrt_marshallIn(sp, tmp, "att_kf_P", att_kf_P);
    att_kf_P_guard = 1U;
  }
}

/*
 * Arguments    : boolean_T skipDirtyCheck
 * Return Type  : void
 */
void MEXGlobalSyncOutFunction(boolean_T skipDirtyCheck)
{
  /* Marshall out global variables */
  if (skipDirtyCheck || (att_kf_q_ref_guard != 0U)) {
    emlrtPutGlobalVariable("att_kf_q_ref", emlrt_marshallOut(b_att_kf_q_ref));
  }

  if (skipDirtyCheck || (att_kf_x_guard != 0U)) {
    emlrtPutGlobalVariable("att_kf_x", b_emlrt_marshallOut(att_kf_x));
  }

  if (skipDirtyCheck || (att_kf_Q_guard != 0U)) {
    emlrtPutGlobalVariable("att_kf_Q", c_emlrt_marshallOut(att_kf_Q));
  }

  if (skipDirtyCheck || (att_kf_R_guard != 0U)) {
    emlrtPutGlobalVariable("att_kf_R", d_emlrt_marshallOut(att_kf_R));
  }

  if (skipDirtyCheck || (att_kf_P_guard != 0U)) {
    emlrtPutGlobalVariable("att_kf_P", c_emlrt_marshallOut(att_kf_P));
  }
}

/*
 * Arguments    : const mxArray * const prhs[2]
 *                const mxArray *plhs[1]
 * Return Type  : void
 */
void angle_between_quat_api(const mxArray * const prhs[2], const mxArray *plhs[1])
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  real_T (*q1)[4];
  real_T (*q2)[4];
  real_T r;
  st.tls = emlrtRootTLSGlobal;

  /* Marshall function inputs */
  q1 = i_emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "q1");
  q2 = i_emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "q2");

  /* Marshall in global variables */
  MEXGlobalSyncInFunction(&st);

  /* Invoke the target function */
  r = angle_between_quat(*q1, *q2);

  /* Marshall out global variables */
  MEXGlobalSyncOutFunction(true);

  /* Marshall function outputs */
  plhs[0] = e_emlrt_marshallOut(r);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void att_kf_api(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Marshall in global variables */
  MEXGlobalSyncInFunction(&st);

  /* Invoke the target function */
  att_kf();

  /* Marshall out global variables */
  MEXGlobalSyncOutFunction(true);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void att_kf_atexit(void)
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
  att_kf_xil_terminate();
  att_kf_xil_shutdown();
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void att_kf_initialize(void)
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
    att_kf_once();
  }
}

/*
 * Arguments    : const mxArray * const prhs[2]
 * Return Type  : void
 */
void att_kf_measure_api(const mxArray * const prhs[2])
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  const mxArray *prhs_copy_idx_0;
  const mxArray *prhs_copy_idx_1;
  real_T (*expected_i)[3];
  real_T (*measured_b)[3];
  st.tls = emlrtRootTLSGlobal;
  prhs_copy_idx_0 = emlrtProtectR2012b(prhs[0], 0, false, -1);
  prhs_copy_idx_1 = emlrtProtectR2012b(prhs[1], 1, false, -1);

  /* Marshall function inputs */
  expected_i = k_emlrt_marshallIn(&st, emlrtAlias(prhs_copy_idx_0), "expected_i");
  measured_b = k_emlrt_marshallIn(&st, emlrtAlias(prhs_copy_idx_1), "measured_b");

  /* Marshall in global variables */
  MEXGlobalSyncInFunction(&st);

  /* Invoke the target function */
  att_kf_measure(*expected_i, *measured_b);

  /* Marshall out global variables */
  MEXGlobalSyncOutFunction(true);
}

/*
 * Arguments    : const mxArray * const prhs[1]
 * Return Type  : void
 */
void att_kf_predict_api(const mxArray * const prhs[1])
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  real_T (*w)[3];
  st.tls = emlrtRootTLSGlobal;

  /* Marshall function inputs */
  w = k_emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "w");

  /* Marshall in global variables */
  MEXGlobalSyncInFunction(&st);

  /* Invoke the target function */
  att_kf_predict(*w);

  /* Marshall out global variables */
  MEXGlobalSyncOutFunction(true);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void att_kf_propagate_api(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Marshall in global variables */
  MEXGlobalSyncInFunction(&st);

  /* Invoke the target function */
  att_kf_propagate();

  /* Marshall out global variables */
  MEXGlobalSyncOutFunction(true);
}

/*
 * Arguments    : const mxArray *plhs[1]
 * Return Type  : void
 */
void att_kf_q_ref_api(const mxArray *plhs[1])
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  real_T (*q_ref)[4];
  st.tls = emlrtRootTLSGlobal;
  q_ref = (real_T (*)[4])mxMalloc(sizeof(real_T [4]));

  /* Marshall in global variables */
  MEXGlobalSyncInFunction(&st);

  /* Invoke the target function */
  att_kf_q_ref(*q_ref);

  /* Marshall out global variables */
  MEXGlobalSyncOutFunction(true);

  /* Marshall function outputs */
  plhs[0] = f_emlrt_marshallOut(*q_ref);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void att_kf_terminate(void)
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
 * Arguments    : const mxArray * const prhs[2]
 *                const mxArray *plhs[1]
 * Return Type  : void
 */
void rotate_by_quat_api(const mxArray * const prhs[2], const mxArray *plhs[1])
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  real_T (*q)[4];
  real_T (*v)[3];
  real_T (*v_rot)[3];
  st.tls = emlrtRootTLSGlobal;
  v_rot = (real_T (*)[3])mxMalloc(sizeof(real_T [3]));

  /* Marshall function inputs */
  v = k_emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "v");
  q = i_emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "q");

  /* Marshall in global variables */
  MEXGlobalSyncInFunction(&st);

  /* Invoke the target function */
  rotate_by_quat(*v, *q, *v_rot);

  /* Marshall out global variables */
  MEXGlobalSyncOutFunction(true);

  /* Marshall function outputs */
  plhs[0] = g_emlrt_marshallOut(*v_rot);
}

/*
 * File trailer for _coder_att_kf_api.c
 *
 * [EOF]
 */
