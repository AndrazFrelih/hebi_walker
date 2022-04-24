/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: LinTrajGenRed_emxutil.h
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 14-Apr-2021 16:16:29
 */

#ifndef LINTRAJGENRED_EMXUTIL_H
#define LINTRAJGENRED_EMXUTIL_H

/* Include Files */
#include "LinTrajGenRed_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>
#ifdef __cplusplus

extern "C" {

#endif

  /* Function Declarations */
  extern void emxEnsureCapacity_real_T(emxArray_real_T *emxArray, int oldNumel);
  extern void emxFree_real_T(emxArray_real_T **pEmxArray);
  extern void emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for LinTrajGenRed_emxutil.h
 *
 * [EOF]
 */
