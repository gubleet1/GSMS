/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: main.c
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 25-Nov-2020 21:15:25
 */

/*************************************************************************/
/* This automatically generated example C main file shows how to call    */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/

/* Include Files */
#include "main.h"
#include "att_kf.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static void argInit_3x1_real_T(double result[3]);
static void argInit_4x1_real_T(double result[4]);
static double argInit_real_T(void);
static void main_angle_between_quat(void);
static void main_att_kf(void);
static void main_att_kf_measure(void);
static void main_att_kf_predict(void);
static void main_att_kf_propagate(void);
static void main_att_kf_q_ref(void);
static void main_rotate_by_quat(void);

/* Function Definitions */
/*
 * Arguments    : double result[3]
 * Return Type  : void
 */
static void argInit_3x1_real_T(double result[3])
{
  int idx0;

  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 3; idx0++) {
    /* Set the value of the array element.
       Change this value to the value that the application requires. */
    result[idx0] = argInit_real_T();
  }
}

/*
 * Arguments    : double result[4]
 * Return Type  : void
 */
static void argInit_4x1_real_T(double result[4])
{
  int idx0;

  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 4; idx0++) {
    /* Set the value of the array element.
       Change this value to the value that the application requires. */
    result[idx0] = argInit_real_T();
  }
}

/*
 * Arguments    : void
 * Return Type  : double
 */
static double argInit_real_T(void)
{
  return 0.0;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
static void main_angle_between_quat(void)
{
  double q1_tmp[4];
  double r;

  /* Initialize function 'angle_between_quat' input arguments. */
  /* Initialize function input argument 'q1'. */
  argInit_4x1_real_T(q1_tmp);

  /* Initialize function input argument 'q2'. */
  /* Call the entry-point 'angle_between_quat'. */
  r = angle_between_quat(q1_tmp, q1_tmp);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
static void main_att_kf(void)
{
  /* Call the entry-point 'att_kf'. */
  att_kf();
}

/*
 * Arguments    : void
 * Return Type  : void
 */
static void main_att_kf_measure(void)
{
  double b_expected_i_tmp[3];
  double expected_i_tmp[3];

  /* Initialize function 'att_kf_measure' input arguments. */
  /* Initialize function input argument 'expected_i'. */
  argInit_3x1_real_T(expected_i_tmp);

  /* Initialize function input argument 'measured_b'. */
  /* Call the entry-point 'att_kf_measure'. */
  b_expected_i_tmp[0] = expected_i_tmp[0];
  b_expected_i_tmp[1] = expected_i_tmp[1];
  b_expected_i_tmp[2] = expected_i_tmp[2];
  att_kf_measure(expected_i_tmp, b_expected_i_tmp);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
static void main_att_kf_predict(void)
{
  double dv[3];

  /* Initialize function 'att_kf_predict' input arguments. */
  /* Initialize function input argument 'w'. */
  /* Call the entry-point 'att_kf_predict'. */
  argInit_3x1_real_T(dv);
  att_kf_predict(dv);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
static void main_att_kf_propagate(void)
{
  /* Call the entry-point 'att_kf_propagate'. */
  att_kf_propagate();
}

/*
 * Arguments    : void
 * Return Type  : void
 */
static void main_att_kf_q_ref(void)
{
  double q_ref[4];

  /* Call the entry-point 'att_kf_q_ref'. */
  att_kf_q_ref(q_ref);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
static void main_rotate_by_quat(void)
{
  double dv1[4];
  double dv[3];
  double v_rot[3];

  /* Initialize function 'rotate_by_quat' input arguments. */
  /* Initialize function input argument 'v'. */
  /* Initialize function input argument 'q'. */
  /* Call the entry-point 'rotate_by_quat'. */
  argInit_3x1_real_T(dv);
  argInit_4x1_real_T(dv1);
  rotate_by_quat(dv, dv1, v_rot);
}

/*
 * Arguments    : int argc
 *                const char * const argv[]
 * Return Type  : int
 */
int main(int argc, const char * const argv[])
{
  (void)argc;
  (void)argv;

  /* The initialize function is being called automatically from your entry-point function. So, a call to initialize is not included here. */
  /* Invoke the entry-point functions.
     You can call entry-point functions multiple times. */
  main_angle_between_quat();
  main_att_kf();
  main_att_kf_measure();
  main_att_kf_predict();
  main_att_kf_propagate();
  main_att_kf_q_ref();
  main_rotate_by_quat();

  /* Terminate the application.
     You do not need to do this more than one time. */
  att_kf_terminate();
  return 0;
}

/*
 * File trailer for main.c
 *
 * [EOF]
 */
