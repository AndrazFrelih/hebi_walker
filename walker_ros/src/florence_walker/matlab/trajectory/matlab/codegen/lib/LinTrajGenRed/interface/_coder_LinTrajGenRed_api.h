/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_LinTrajGenRed_api.h
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 14-Apr-2021 16:16:29
 */

#ifndef _CODER_LINTRAJGENRED_API_H
#define _CODER_LINTRAJGENRED_API_H

/* Include Files */
#include "emlrt.h"
#include "tmwtypes.h"
#include <string.h>

/* Type Definitions */
#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T

struct emxArray_real_T
{
  real_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray_real_T*/

#ifndef typedef_emxArray_real_T
#define typedef_emxArray_real_T

typedef struct emxArray_real_T emxArray_real_T;

#endif                                 /*typedef_emxArray_real_T*/

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

#ifdef __cplusplus

extern "C" {

#endif

  /* Function Declarations */
  void LinTrajGenRed(real_T inp[5], real_T out[6]);
  void LinTrajGenRed_api(const mxArray * const prhs[1], const mxArray *plhs[1]);
  void LinTrajGenRed_atexit(void);
  void LinTrajGenRed_initialize(void);
  void LinTrajGenRed_terminate(void);
  void LinTrajGenRed_xil_shutdown(void);
  void LinTrajGenRed_xil_terminate(void);
  void simple_trajectory(emxArray_real_T *Xini, emxArray_real_T *Xend, real_T
    Tfin, int32_T Ndof, int32_T Npoly, emxArray_real_T *aq);
  void simple_trajectory_api(const mxArray * const prhs[5], const mxArray *plhs
    [1]);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for _coder_LinTrajGenRed_api.h
 *
 * [EOF]
 */
