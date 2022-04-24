/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: main.c
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 14-Apr-2021 16:16:29
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
#include "LinTrajGenRed.h"
#include "LinTrajGenRed_emxAPI.h"
#include "LinTrajGenRed_terminate.h"
#include "LinTrajGenRed_types.h"
#include "rt_nonfinite.h"
#include "simple_trajectory.h"

/* Function Declarations */
static void argInit_5x1_real_T(double result[5]);
static emxArray_real_T *argInit_Unboundedx1_real_T(void);
static int argInit_int32_T(void);
static double argInit_real_T(void);
static void main_LinTrajGenRed(void);
static void main_simple_trajectory(void);

/* Function Definitions */
/*
 * Arguments    : double result[5]
 * Return Type  : void
 */
static void argInit_5x1_real_T(double result[5])
{
  int idx0;

  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 5; idx0++) {
    /* Set the value of the array element.
       Change this value to the value that the application requires. */
    result[idx0] = argInit_real_T();
  }
}

/*
 * Arguments    : void
 * Return Type  : emxArray_real_T *
 */
static emxArray_real_T *argInit_Unboundedx1_real_T(void)
{
  static const int iv[1] = { 2 };

  emxArray_real_T *result;
  int idx0;

  /* Set the size of the array.
     Change this size to the value that the application requires. */
  result = emxCreateND_real_T(1, iv);

  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < result->size[0U]; idx0++) {
    /* Set the value of the array element.
       Change this value to the value that the application requires. */
    result->data[idx0] = argInit_real_T();
  }

  return result;
}

/*
 * Arguments    : void
 * Return Type  : int
 */
static int argInit_int32_T(void)
{
  return 0;
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
static void main_LinTrajGenRed(void)
{
  double out[6];
  double dv[5];

  /* Initialize function 'LinTrajGenRed' input arguments. */
  /* Initialize function input argument 'inp'. */
  /* Call the entry-point 'LinTrajGenRed'. */
  argInit_5x1_real_T(dv);
  LinTrajGenRed(dv, out);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
static void main_simple_trajectory(void)
{
  emxArray_real_T *Xend;
  emxArray_real_T *Xini;
  emxArray_real_T *aq;
  int Ndof_tmp;
  emxInitArray_real_T(&aq, 2);

  /* Initialize function 'simple_trajectory' input arguments. */
  /* Initialize function input argument 'Xini'. */
  Xini = argInit_Unboundedx1_real_T();

  /* Initialize function input argument 'Xend'. */
  Xend = argInit_Unboundedx1_real_T();
  Ndof_tmp = argInit_int32_T();

  /* Call the entry-point 'simple_trajectory'. */
  simple_trajectory(Xini, Xend, argInit_real_T(), Ndof_tmp, Ndof_tmp, aq);
  emxDestroyArray_real_T(aq);
  emxDestroyArray_real_T(Xend);
  emxDestroyArray_real_T(Xini);
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
  main_LinTrajGenRed();
  main_simple_trajectory();

  /* Terminate the application.
     You do not need to do this more than one time. */
  LinTrajGenRed_terminate();
  return 0;
}

/*
 * File trailer for main.c
 *
 * [EOF]
 */
