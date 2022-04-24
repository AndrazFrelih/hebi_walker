/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: simple_trajectory.h
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 14-Apr-2021 16:16:29
 */

#ifndef SIMPLE_TRAJECTORY_H
#define SIMPLE_TRAJECTORY_H

/* Include Files */
#include "LinTrajGenRed_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>
#ifdef __cplusplus

extern "C" {

#endif

  /* Function Declarations */
  extern void simple_trajectory(const emxArray_real_T *Xini, const
    emxArray_real_T *Xend, double Tfin, int Ndof, int Npoly, emxArray_real_T *aq);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for simple_trajectory.h
 *
 * [EOF]
 */
