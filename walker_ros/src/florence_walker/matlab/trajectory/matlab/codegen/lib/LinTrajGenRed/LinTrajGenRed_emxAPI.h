/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: LinTrajGenRed_emxAPI.h
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 14-Apr-2021 16:16:29
 */

#ifndef LINTRAJGENRED_EMXAPI_H
#define LINTRAJGENRED_EMXAPI_H

/* Include Files */
#include "LinTrajGenRed_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>
#ifdef __cplusplus

extern "C" {

#endif

  /* Function Declarations */
  extern emxArray_real_T *emxCreateND_real_T(int numDimensions, const int *size);
  extern emxArray_real_T *emxCreateWrapperND_real_T(double *data, int
    numDimensions, const int *size);
  extern emxArray_real_T *emxCreateWrapper_real_T(double *data, int rows, int
    cols);
  extern emxArray_real_T *emxCreate_real_T(int rows, int cols);
  extern void emxDestroyArray_real_T(emxArray_real_T *emxArray);
  extern void emxInitArray_real_T(emxArray_real_T **pEmxArray, int numDimensions);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for LinTrajGenRed_emxAPI.h
 *
 * [EOF]
 */
