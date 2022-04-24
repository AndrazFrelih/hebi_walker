/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: LinTrajGenRed.c
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 14-Apr-2021 16:16:29
 */

/* Include Files */
#include "LinTrajGenRed.h"
#include "LinTrajGenRed_rtwutil.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : const double inp[5]
 *                double out[6]
 * Return Type  : void
 */
void LinTrajGenRed(const double inp[5], double out[6])
{
  out[0] = inp[1];
  out[1] = inp[3];
  out[2] = 0.0;
  out[3] = -(2.0 * (((5.0 * inp[1] - 5.0 * inp[2]) + 2.0 * inp[0] * inp[4]) +
                    3.0 * inp[0] * inp[3])) / rt_powd_snf(inp[0], 3.0);
  out[4] = (((15.0 * inp[1] - 15.0 * inp[2]) + 7.0 * inp[0] * inp[4]) + 8.0 *
            inp[0] * inp[3]) / rt_powd_snf(inp[0], 4.0);
  out[5] = -(3.0 * (((2.0 * inp[1] - 2.0 * inp[2]) + inp[0] * inp[4]) + inp[0] *
                    inp[3])) / rt_powd_snf(inp[0], 5.0);
}

/*
 * File trailer for LinTrajGenRed.c
 *
 * [EOF]
 */
