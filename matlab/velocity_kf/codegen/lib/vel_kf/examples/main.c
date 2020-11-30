/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: main.c
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 30-Nov-2020 23:53:28
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
#include "vel_kf.h"

/* Function Declarations */
static void argInit_3x1_real_T(double result[3]);
static void argInit_4x1_real_T(double result[4]);
static double argInit_real_T(void);
static void main_quat_conj(void);
static void main_rotate_z_by_ang(void);
static void main_vel_kf(void);
static void main_vel_kf_measure(void);
static void main_vel_kf_predict(void);
static void main_vel_kf_vel(void);

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
static void main_quat_conj(void)
{
  double dv[4];
  double qc[4];

  /* Initialize function 'quat_conj' input arguments. */
  /* Initialize function input argument 'q'. */
  /* Call the entry-point 'quat_conj'. */
  argInit_4x1_real_T(dv);
  quat_conj(dv, qc);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
static void main_rotate_z_by_ang(void)
{
  double dv[3];
  double v_rot[3];

  /* Initialize function 'rotate_z_by_ang' input arguments. */
  /* Initialize function input argument 'v'. */
  /* Call the entry-point 'rotate_z_by_ang'. */
  argInit_3x1_real_T(dv);
  rotate_z_by_ang(dv, argInit_real_T(), v_rot);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
static void main_vel_kf(void)
{
  /* Call the entry-point 'vel_kf'. */
  vel_kf();
}

/*
 * Arguments    : void
 * Return Type  : void
 */
static void main_vel_kf_measure(void)
{
  double dv[3];

  /* Initialize function 'vel_kf_measure' input arguments. */
  /* Initialize function input argument 'measured_b'. */
  /* Call the entry-point 'vel_kf_measure'. */
  argInit_3x1_real_T(dv);
  vel_kf_measure(dv);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
static void main_vel_kf_predict(void)
{
  double dv[3];

  /* Initialize function 'vel_kf_predict' input arguments. */
  /* Initialize function input argument 'lin_accel'. */
  /* Call the entry-point 'vel_kf_predict'. */
  argInit_3x1_real_T(dv);
  vel_kf_predict(dv);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
static void main_vel_kf_vel(void)
{
  double vel[3];

  /* Call the entry-point 'vel_kf_vel'. */
  vel_kf_vel(vel);
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
  main_quat_conj();
  main_rotate_z_by_ang();
  main_vel_kf();
  main_vel_kf_measure();
  main_vel_kf_predict();
  main_vel_kf_vel();

  /* Terminate the application.
     You do not need to do this more than one time. */
  vel_kf_terminate();
  return 0;
}

/*
 * File trailer for main.c
 *
 * [EOF]
 */
