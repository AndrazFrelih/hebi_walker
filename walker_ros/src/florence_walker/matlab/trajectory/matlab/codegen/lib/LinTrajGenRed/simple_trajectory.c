/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: simple_trajectory.c
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 14-Apr-2021 16:16:29
 */

/* Include Files */
#include "simple_trajectory.h"
#include "LinTrajGenRed_emxutil.h"
#include "LinTrajGenRed_rtwutil.h"
#include "LinTrajGenRed_types.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : const emxArray_real_T *Xini
 *                const emxArray_real_T *Xend
 *                double Tfin
 *                int Ndof
 *                int Npoly
 *                emxArray_real_T *aq
 * Return Type  : void
 */
void simple_trajectory(const emxArray_real_T *Xini, const emxArray_real_T *Xend,
  double Tfin, int Ndof, int Npoly, emxArray_real_T *aq)
{
  double out[6];
  int b_aq;
  int b_i;
  int i;
  i = aq->size[0] * aq->size[1];
  aq->size[0] = Ndof;
  aq->size[1] = Npoly;
  emxEnsureCapacity_real_T(aq, i);
  if (0 <= Ndof - 1) {
    out[1] = 0.0;
    out[2] = 0.0;
  }

  for (b_i = 0; b_i < Ndof; b_i++) {
    out[0] = Xini->data[b_i];
    out[3] = -(2.0 * (((5.0 * Xini->data[b_i] - 5.0 * Xend->data[b_i]) + 2.0 *
                       Tfin * 0.0) + 3.0 * Tfin * 0.0)) / rt_powd_snf(Tfin, 3.0);
    out[4] = (((15.0 * Xini->data[b_i] - 15.0 * Xend->data[b_i]) + 7.0 * Tfin *
               0.0) + 8.0 * Tfin * 0.0) / rt_powd_snf(Tfin, 4.0);
    out[5] = -(3.0 * (((2.0 * Xini->data[b_i] - 2.0 * Xend->data[b_i]) + Tfin *
                       0.0) + Tfin * 0.0)) / rt_powd_snf(Tfin, 5.0);
    b_aq = aq->size[1];
    for (i = 0; i < b_aq; i++) {
      aq->data[b_i + aq->size[0] * i] = out[i];
    }
  }
}

/*
 * File trailer for simple_trajectory.c
 *
 * [EOF]
 */
