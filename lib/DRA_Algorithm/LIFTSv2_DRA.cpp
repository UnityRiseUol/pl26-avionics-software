//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: LIFTSv2_DRA.cpp
//
// Code generated for Simulink model 'LIFTSv2_DRA'.
//
// Model version                  : 1.23
// Simulink Coder version         : 26.1 (R2026a) 20-Nov-2025
// C/C++ source code generated on : Sun Jun 14 20:30:55 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Windows64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "LIFTSv2_DRA.h"
#include "rtwtypes.h"
#include <cmath>
#include <cstring>

// Function for MATLAB Function: '<Root>/MATLAB Function'
real32_T LIFTSv2_DRA::LIFTSv2_DRA_norm(const real32_T x[3])
{
  real32_T absxk;
  real32_T scale;
  real32_T t;
  real32_T y;
  scale = 1.2924697E-26F;
  absxk = std::abs(x[0]);
  if (absxk > 1.2924697E-26F) {
    y = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.2924697E-26F;
    y = t * t;
  }

  absxk = std::abs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = std::abs(x[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  y = scale * std::sqrt(y);
  if (std::isnan(y)) {
    int32_T b_k;
    b_k = 0;
    int32_T exitg1;
    do {
      exitg1 = 0;
      if (b_k < 3) {
        if (std::isnan(x[b_k])) {
          exitg1 = 1;
        } else {
          b_k++;
        }
      } else {
        y = (rtInfF);
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  return y;
}

// Model step function
void LIFTSv2_DRA::step()
{
  int32_T b_k;
  int32_T e_k;
  int32_T ijA;
  int32_T jA;
  int32_T jBcol;
  int32_T jj;
  int32_T kBcol;
  real32_T b_I_0[81];
  real32_T b_a[81];
  real32_T A_tmp[72];
  real32_T X[72];
  real32_T A[64];
  real32_T R[64];
  real32_T yhat_tmp_0[9];
  real32_T v[8];
  real32_T tmp_c[3];
  real32_T dq_idx_0;
  real32_T dq_idx_1;
  real32_T dq_idx_2;
  real32_T dq_idx_3;
  real32_T tmp;
  real32_T tmp_0;
  real32_T tmp_1;
  real32_T tmp_2;
  real32_T tmp_3;
  real32_T tmp_4;
  real32_T tmp_5;
  real32_T tmp_6;
  real32_T tmp_7;
  real32_T tmp_8;
  real32_T tmp_9;
  real32_T tmp_a;
  real32_T tmp_b;
  real32_T w_mag;
  real32_T yhat_tmp;
  real32_T yhat_tmp_tmp;
  int8_T b_I[81];
  int8_T ipiv[8];
  int8_T ipiv_0;
  boolean_T gps_is_healthy;
  static const real32_T e[3]{ 0.0F, 0.0F, 9.80665F };

  static const real32_T b_a_0[81]{ 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.01F, 0.0F, 0.0F, 1.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.01F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.01F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F };

  static const real32_T f[81]{ 1.0F, 0.0F, 0.0F, 0.01F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.01F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    1.0F, 0.0F, 0.0F, 0.01F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F };

  static const real32_T g[81]{ 0.01F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.01F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.01F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.01F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.01F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.01F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.01F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.01F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.01F };

  static const real32_T d_b[3]{ 0.2F, -0.01F, 0.45F };

  static const real_T h[8]{ 0.25, 0.25, 1.0, 4.0, 0.1, 5.0, 5.0, 5.0 };

  static const real_T m[8]{ 1.0E+9, 1.0E+9, 1.0E+9, 4.0, 0.1, 5.0, 5.0, 5.0 };

  static const int8_T l[72]{ 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1 };

  static const int8_T c_a[72]{ 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 1 };

  int32_T exitg1;

  // MATLAB Function: '<Root>/MATLAB Function' incorporates:
  //   Inport: '<Root>/Accel_XYZ'
  //   Inport: '<Root>/Bmp_Alt'
  //   Inport: '<Root>/Bmp_Vspeed'
  //   Inport: '<Root>/Gps_Alt'
  //   Inport: '<Root>/Gps_Lat'
  //   Inport: '<Root>/Gps_Lon'
  //   Inport: '<Root>/Gps_Valid'
  //   Inport: '<Root>/Gyro_XYZ'
  //   Inport: '<Root>/Launch_LLA'
  //   Inport: '<Root>/Mag_XYZ'
  //   Outport: '<Root>/Predicted_LLA'

  LIFTSv2_DRA_DW.t_accum += 0.01F;
  w_mag = LIFTSv2_DRA_norm(LIFTSv2_DRA_U.Gyro_XYZ);
  if (w_mag > 1.0E-6F) {
    dq_idx_0 = w_mag * 0.01F / 2.0F;
    dq_idx_3 = std::sin(dq_idx_0);
    dq_idx_0 = std::cos(dq_idx_0);
    dq_idx_1 = LIFTSv2_DRA_U.Gyro_XYZ[0] / w_mag * dq_idx_3;
    dq_idx_2 = LIFTSv2_DRA_U.Gyro_XYZ[1] / w_mag * dq_idx_3;
    dq_idx_3 *= LIFTSv2_DRA_U.Gyro_XYZ[2] / w_mag;
    w_mag = LIFTSv2_DRA_DW.q_nav[0];
    yhat_tmp = LIFTSv2_DRA_DW.q_nav[1];
    yhat_tmp_tmp = LIFTSv2_DRA_DW.q_nav[2];
    tmp = LIFTSv2_DRA_DW.q_nav[3];
    tmp_0 = LIFTSv2_DRA_DW.q_nav[0];
    tmp_1 = LIFTSv2_DRA_DW.q_nav[1];
    tmp_2 = LIFTSv2_DRA_DW.q_nav[2];
    tmp_3 = LIFTSv2_DRA_DW.q_nav[3];
    tmp_4 = LIFTSv2_DRA_DW.q_nav[0];
    tmp_5 = LIFTSv2_DRA_DW.q_nav[1];
    tmp_6 = LIFTSv2_DRA_DW.q_nav[2];
    tmp_7 = LIFTSv2_DRA_DW.q_nav[3];
    tmp_8 = LIFTSv2_DRA_DW.q_nav[0];
    tmp_9 = LIFTSv2_DRA_DW.q_nav[1];
    tmp_a = LIFTSv2_DRA_DW.q_nav[2];
    tmp_b = LIFTSv2_DRA_DW.q_nav[3];
    LIFTSv2_DRA_DW.q_nav[0] = ((w_mag * dq_idx_0 - yhat_tmp * dq_idx_1) -
      yhat_tmp_tmp * dq_idx_2) - tmp * dq_idx_3;
    LIFTSv2_DRA_DW.q_nav[1] = ((tmp_0 * dq_idx_1 + dq_idx_0 * tmp_1) + tmp_2 *
      dq_idx_3) - dq_idx_2 * tmp_3;
    LIFTSv2_DRA_DW.q_nav[2] = ((tmp_4 * dq_idx_2 - tmp_5 * dq_idx_3) + dq_idx_0 *
      tmp_6) + dq_idx_1 * tmp_7;
    LIFTSv2_DRA_DW.q_nav[3] = ((tmp_8 * dq_idx_3 + tmp_9 * dq_idx_2) - dq_idx_1 *
      tmp_a) + dq_idx_0 * tmp_b;
  }

  w_mag = 1.2924697E-26F;
  dq_idx_0 = std::abs(LIFTSv2_DRA_DW.q_nav[0]);
  if (dq_idx_0 > 1.2924697E-26F) {
    dq_idx_2 = 1.0F;
    w_mag = dq_idx_0;
  } else {
    dq_idx_1 = dq_idx_0 / 1.2924697E-26F;
    dq_idx_2 = dq_idx_1 * dq_idx_1;
  }

  dq_idx_0 = std::abs(LIFTSv2_DRA_DW.q_nav[1]);
  if (dq_idx_0 > w_mag) {
    dq_idx_1 = w_mag / dq_idx_0;
    dq_idx_2 = dq_idx_2 * dq_idx_1 * dq_idx_1 + 1.0F;
    w_mag = dq_idx_0;
  } else {
    dq_idx_1 = dq_idx_0 / w_mag;
    dq_idx_2 += dq_idx_1 * dq_idx_1;
  }

  dq_idx_0 = std::abs(LIFTSv2_DRA_DW.q_nav[2]);
  if (dq_idx_0 > w_mag) {
    dq_idx_1 = w_mag / dq_idx_0;
    dq_idx_2 = dq_idx_2 * dq_idx_1 * dq_idx_1 + 1.0F;
    w_mag = dq_idx_0;
  } else {
    dq_idx_1 = dq_idx_0 / w_mag;
    dq_idx_2 += dq_idx_1 * dq_idx_1;
  }

  dq_idx_0 = std::abs(LIFTSv2_DRA_DW.q_nav[3]);
  if (dq_idx_0 > w_mag) {
    dq_idx_1 = w_mag / dq_idx_0;
    dq_idx_2 = dq_idx_2 * dq_idx_1 * dq_idx_1 + 1.0F;
    w_mag = dq_idx_0;
  } else {
    dq_idx_1 = dq_idx_0 / w_mag;
    dq_idx_2 += dq_idx_1 * dq_idx_1;
  }

  dq_idx_2 = w_mag * std::sqrt(dq_idx_2);
  if (std::isnan(dq_idx_2)) {
    b_k = 0;
    do {
      exitg1 = 0;
      if (b_k < 4) {
        if (std::isnan(LIFTSv2_DRA_DW.q_nav[b_k])) {
          exitg1 = 1;
        } else {
          b_k++;
        }
      } else {
        dq_idx_2 = (rtInfF);
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  LIFTSv2_DRA_DW.q_nav[0] /= dq_idx_2;
  LIFTSv2_DRA_DW.q_nav[1] /= dq_idx_2;
  LIFTSv2_DRA_DW.q_nav[2] /= dq_idx_2;
  LIFTSv2_DRA_DW.q_nav[3] /= dq_idx_2;
  w_mag = LIFTSv2_DRA_DW.q_nav[0] * LIFTSv2_DRA_DW.q_nav[0];
  yhat_tmp = LIFTSv2_DRA_DW.q_nav[1] * LIFTSv2_DRA_DW.q_nav[1];
  yhat_tmp_tmp = LIFTSv2_DRA_DW.q_nav[2] * LIFTSv2_DRA_DW.q_nav[2];
  tmp = LIFTSv2_DRA_DW.q_nav[3] * LIFTSv2_DRA_DW.q_nav[3];
  yhat_tmp_0[0] = ((w_mag + yhat_tmp) - yhat_tmp_tmp) - tmp;
  tmp_0 = LIFTSv2_DRA_DW.q_nav[1] * LIFTSv2_DRA_DW.q_nav[2];
  tmp_1 = LIFTSv2_DRA_DW.q_nav[0] * LIFTSv2_DRA_DW.q_nav[3];
  yhat_tmp_0[3] = (tmp_0 - tmp_1) * 2.0F;
  tmp_2 = LIFTSv2_DRA_DW.q_nav[1] * LIFTSv2_DRA_DW.q_nav[3];
  tmp_3 = LIFTSv2_DRA_DW.q_nav[0] * LIFTSv2_DRA_DW.q_nav[2];
  yhat_tmp_0[6] = (tmp_2 + tmp_3) * 2.0F;
  yhat_tmp_0[1] = (tmp_0 + tmp_1) * 2.0F;
  w_mag -= yhat_tmp;
  yhat_tmp_0[4] = (w_mag + yhat_tmp_tmp) - tmp;
  yhat_tmp = LIFTSv2_DRA_DW.q_nav[2] * LIFTSv2_DRA_DW.q_nav[3];
  tmp_0 = LIFTSv2_DRA_DW.q_nav[0] * LIFTSv2_DRA_DW.q_nav[1];
  yhat_tmp_0[7] = (yhat_tmp - tmp_0) * 2.0F;
  yhat_tmp_0[2] = (tmp_2 - tmp_3) * 2.0F;
  yhat_tmp_0[5] = (yhat_tmp + tmp_0) * 2.0F;
  yhat_tmp_0[8] = (w_mag - yhat_tmp_tmp) + tmp;
  w_mag = LIFTSv2_DRA_U.Accel_XYZ[1];
  dq_idx_0 = LIFTSv2_DRA_U.Accel_XYZ[0];
  dq_idx_1 = LIFTSv2_DRA_U.Accel_XYZ[2];
  for (jj = 0; jj < 3; jj++) {
    dq_idx_2 = ((yhat_tmp_0[jj + 3] * w_mag + yhat_tmp_0[jj] * dq_idx_0) +
                yhat_tmp_0[jj + 6] * dq_idx_1) - e[jj];
    dq_idx_3 = dq_idx_2 * 0.01F + LIFTSv2_DRA_DW.vel_nav[jj];
    LIFTSv2_DRA_DW.vel_nav[jj] = dq_idx_3;
    LIFTSv2_DRA_DW.pos_nav[jj] = (dq_idx_3 * 0.01F + LIFTSv2_DRA_DW.pos_nav[jj])
      + 0.5F * dq_idx_2 * 0.0001F;
  }

  LIFTSv2_DRA_DW.xhat[0] += LIFTSv2_DRA_DW.xhat[3] * 0.01F;
  LIFTSv2_DRA_DW.xhat[1] += LIFTSv2_DRA_DW.xhat[4] * 0.01F;
  LIFTSv2_DRA_DW.xhat[2] += LIFTSv2_DRA_DW.xhat[5] * 0.01F;
  for (jj = 0; jj < 9; jj++) {
    for (jBcol = 0; jBcol < 9; jBcol++) {
      b_a[jBcol + 9 * jj] = 0.0F;
    }

    for (jBcol = 0; jBcol < 9; jBcol++) {
      w_mag = LIFTSv2_DRA_DW.P[9 * jj + jBcol];
      for (jA = 0; jA < 9; jA++) {
        b_k = 9 * jj + jA;
        b_a[b_k] += b_a_0[9 * jBcol + jA] * w_mag;
      }
    }
  }

  for (jj = 0; jj < 9; jj++) {
    for (jBcol = 0; jBcol < 9; jBcol++) {
      w_mag = 0.0F;
      for (jA = 0; jA < 9; jA++) {
        w_mag += b_a[9 * jA + jj] * f[9 * jBcol + jA];
      }

      b_k = 9 * jBcol + jj;
      LIFTSv2_DRA_DW.P[b_k] = g[b_k] + w_mag;
    }
  }

  w_mag = std::cos(LIFTSv2_DRA_DW.xhat[7]);
  dq_idx_0 = std::sin(LIFTSv2_DRA_DW.xhat[7]);
  dq_idx_1 = std::cos(LIFTSv2_DRA_DW.xhat[8]);
  dq_idx_2 = std::sin(LIFTSv2_DRA_DW.xhat[8]);
  dq_idx_3 = std::sin(LIFTSv2_DRA_DW.xhat[6]);
  yhat_tmp = std::cos(LIFTSv2_DRA_DW.xhat[6]);
  yhat_tmp_0[0] = w_mag * dq_idx_1;
  yhat_tmp_0[3] = w_mag * dq_idx_2;
  yhat_tmp_0[6] = -dq_idx_0;
  yhat_tmp_tmp = dq_idx_3 * dq_idx_0;
  yhat_tmp_0[1] = yhat_tmp_tmp * dq_idx_1 - yhat_tmp * dq_idx_2;
  yhat_tmp_0[4] = yhat_tmp_tmp * dq_idx_2 + yhat_tmp * dq_idx_1;
  yhat_tmp_0[7] = dq_idx_3 * w_mag;
  yhat_tmp_tmp = yhat_tmp * dq_idx_0;
  yhat_tmp_0[2] = yhat_tmp_tmp * dq_idx_1 + dq_idx_3 * dq_idx_2;
  yhat_tmp_0[5] = yhat_tmp_tmp * dq_idx_2 - dq_idx_3 * dq_idx_1;
  yhat_tmp_0[8] = yhat_tmp * w_mag;
  dq_idx_0 = 0.0F;
  dq_idx_1 = 0.0F;
  dq_idx_2 = 0.0F;
  for (jj = 0; jj < 3; jj++) {
    w_mag = d_b[jj];
    dq_idx_0 += yhat_tmp_0[3 * jj] * w_mag;
    dq_idx_1 += yhat_tmp_0[3 * jj + 1] * w_mag;
    dq_idx_2 += yhat_tmp_0[3 * jj + 2] * w_mag;
  }

  if (LIFTSv2_DRA_U.Gps_Valid) {
    tmp_c[0] = LIFTSv2_DRA_DW.pos_nav[0] - LIFTSv2_DRA_DW.xhat[0];
    tmp_c[1] = LIFTSv2_DRA_DW.pos_nav[1] - LIFTSv2_DRA_DW.xhat[1];
    tmp_c[2] = LIFTSv2_DRA_DW.pos_nav[2] - LIFTSv2_DRA_DW.xhat[2];
    gps_is_healthy = (LIFTSv2_DRA_norm(tmp_c) < 100.0F);
  } else {
    gps_is_healthy = false;
  }

  for (jj = 0; jj < 8; jj++) {
    if (gps_is_healthy) {
      v[jj] = static_cast<real32_T>(h[jj]);
    } else {
      v[jj] = static_cast<real32_T>(m[jj]);
    }
  }

  std::memset(&R[0], 0, sizeof(real32_T) << 6U);
  for (b_k = 0; b_k < 8; b_k++) {
    R[b_k + (b_k << 3)] = v[b_k];
  }

  for (jj = 0; jj < 9; jj++) {
    for (jBcol = 0; jBcol < 8; jBcol++) {
      A_tmp[jBcol + (jj << 3)] = 0.0F;
    }

    for (jBcol = 0; jBcol < 9; jBcol++) {
      w_mag = LIFTSv2_DRA_DW.P[9 * jj + jBcol];
      for (jA = 0; jA < 8; jA++) {
        b_k = (jj << 3) + jA;
        A_tmp[b_k] += static_cast<real32_T>((&c_a[0])[(jBcol << 3) + jA]) *
          w_mag;
      }
    }
  }

  for (jj = 0; jj < 8; jj++) {
    for (jBcol = 0; jBcol < 8; jBcol++) {
      w_mag = 0.0F;
      for (jA = 0; jA < 9; jA++) {
        w_mag += A_tmp[(jA << 3) + jj] * static_cast<real32_T>((&l[0])[9 * jBcol
          + jA]);
      }

      jA = (jBcol << 3) + jj;
      A[jA] = R[jA] + w_mag;
    }

    for (jBcol = 0; jBcol < 9; jBcol++) {
      X[jBcol + 9 * jj] = 0.0F;
    }

    for (jBcol = 0; jBcol < 9; jBcol++) {
      jA = (&l[0])[9 * jj + jBcol];
      for (e_k = 0; e_k < 9; e_k++) {
        b_k = 9 * jj + e_k;
        X[b_k] += LIFTSv2_DRA_DW.P[9 * jBcol + e_k] * static_cast<real32_T>(jA);
      }
    }

    ipiv[jj] = static_cast<int8_T>(jj + 1);
  }

  for (b_k = 0; b_k < 7; b_k++) {
    jj = b_k * 9;
    jBcol = 9 - b_k;
    jA = 0;
    w_mag = std::abs(A[jj]);
    for (e_k = 2; e_k < jBcol; e_k++) {
      dq_idx_3 = std::abs(A[(jj + e_k) - 1]);
      if (dq_idx_3 > w_mag) {
        jA = e_k - 1;
        w_mag = dq_idx_3;
      }
    }

    if (A[jj + jA] != 0.0F) {
      if (jA != 0) {
        jBcol = b_k + jA;
        ipiv[b_k] = static_cast<int8_T>(jBcol + 1);
        for (e_k = 0; e_k < 8; e_k++) {
          jA = e_k << 3;
          kBcol = jA + b_k;
          w_mag = A[kBcol];
          jA += jBcol;
          A[kBcol] = A[jA];
          A[jA] = w_mag;
        }
      }

      jBcol = (jj - b_k) + 8;
      for (jA = jj + 2; jA <= jBcol; jA++) {
        A[jA - 1] /= A[jj];
      }
    }

    jBcol = 6 - b_k;
    jA = jj + 10;
    for (e_k = 0; e_k <= jBcol; e_k++) {
      w_mag = A[((e_k << 3) + jj) + 8];
      if (w_mag != 0.0F) {
        kBcol = (jA - b_k) + 6;
        for (ijA = jA; ijA <= kBcol; ijA++) {
          A[ijA - 1] += A[((jj + ijA) - jA) + 1] * -w_mag;
        }
      }

      jA += 8;
    }
  }

  for (jj = 0; jj < 8; jj++) {
    jBcol = 9 * jj;
    jA = jj << 3;
    for (e_k = 0; e_k < jj; e_k++) {
      kBcol = 9 * e_k;
      w_mag = A[e_k + jA];
      if (w_mag != 0.0F) {
        for (ijA = 0; ijA < 9; ijA++) {
          b_k = ijA + jBcol;
          X[b_k] -= X[ijA + kBcol] * w_mag;
        }
      }
    }

    w_mag = 1.0F / A[jj + jA];
    for (jA = 0; jA < 9; jA++) {
      b_k = jA + jBcol;
      X[b_k] *= w_mag;
    }
  }

  for (jj = 7; jj >= 0; jj--) {
    jBcol = 9 * jj;
    jA = (jj << 3) - 1;
    for (e_k = jj + 2; e_k < 9; e_k++) {
      kBcol = (e_k - 1) * 9;
      w_mag = A[e_k + jA];
      if (w_mag != 0.0F) {
        for (ijA = 0; ijA < 9; ijA++) {
          b_k = ijA + jBcol;
          X[b_k] -= X[ijA + kBcol] * w_mag;
        }
      }
    }
  }

  for (jj = 6; jj >= 0; jj--) {
    ipiv_0 = ipiv[jj];
    if (jj + 1 != ipiv_0) {
      for (jBcol = 0; jBcol < 9; jBcol++) {
        jA = 9 * jj + jBcol;
        w_mag = X[jA];
        b_k = (ipiv_0 - 1) * 9 + jBcol;
        X[jA] = X[b_k];
        X[b_k] = w_mag;
      }
    }
  }

  dq_idx_3 = std::cos(LIFTSv2_DRA_U.Launch_LLA[0] * 0.017453292F);
  v[0] = (LIFTSv2_DRA_U.Gps_Lon - LIFTSv2_DRA_U.Launch_LLA[1]) * 0.017453292F *
    6.378137E+6F * dq_idx_3 - LIFTSv2_DRA_DW.xhat[0];
  v[1] = (LIFTSv2_DRA_U.Gps_Lat - LIFTSv2_DRA_U.Launch_LLA[0]) * 0.017453292F *
    6.378137E+6F - LIFTSv2_DRA_DW.xhat[1];
  v[2] = (LIFTSv2_DRA_U.Gps_Alt - LIFTSv2_DRA_U.Launch_LLA[2]) -
    LIFTSv2_DRA_DW.xhat[2];
  v[3] = LIFTSv2_DRA_U.Bmp_Alt - LIFTSv2_DRA_DW.xhat[2];
  v[4] = LIFTSv2_DRA_U.Bmp_Vspeed - LIFTSv2_DRA_DW.xhat[5];
  v[5] = LIFTSv2_DRA_U.Mag_XYZ[0] - dq_idx_0;
  v[6] = LIFTSv2_DRA_U.Mag_XYZ[1] - dq_idx_1;
  v[7] = LIFTSv2_DRA_U.Mag_XYZ[2] - dq_idx_2;
  for (jj = 0; jj < 9; jj++) {
    w_mag = 0.0F;
    for (jBcol = 0; jBcol < 8; jBcol++) {
      w_mag += X[9 * jBcol + jj] * v[jBcol];
    }

    LIFTSv2_DRA_DW.xhat[jj] += w_mag;
  }

  std::memset(&b_I[0], 0, 81U * sizeof(int8_T));
  for (b_k = 0; b_k < 9; b_k++) {
    b_I[b_k + 9 * b_k] = 1;
  }

  for (jj = 0; jj < 9; jj++) {
    for (jBcol = 0; jBcol < 9; jBcol++) {
      w_mag = 0.0F;
      for (jA = 0; jA < 8; jA++) {
        w_mag += X[9 * jA + jj] * static_cast<real32_T>((&c_a[0])[(jBcol << 3) +
          jA]);
      }

      b_k = 9 * jBcol + jj;
      b_a[b_k] = static_cast<real32_T>(b_I[b_k]) - w_mag;
      b_I_0[jBcol + 9 * jj] = 0.0F;
    }
  }

  for (jj = 0; jj < 9; jj++) {
    for (jBcol = 0; jBcol < 9; jBcol++) {
      w_mag = LIFTSv2_DRA_DW.P[9 * jj + jBcol];
      for (jA = 0; jA < 9; jA++) {
        b_k = 9 * jj + jA;
        b_I_0[b_k] += b_a[9 * jBcol + jA] * w_mag;
      }
    }
  }

  std::memcpy(&LIFTSv2_DRA_DW.P[0], &b_I_0[0], 81U * sizeof(real32_T));
  LIFTSv2_DRA_Y.Predicted_LLA[0] = LIFTSv2_DRA_DW.xhat[1] / 111319.49F +
    LIFTSv2_DRA_U.Launch_LLA[0];
  LIFTSv2_DRA_Y.Predicted_LLA[1] = LIFTSv2_DRA_DW.xhat[0] / (6.378137E+6F *
    dq_idx_3 * 0.017453292F) + LIFTSv2_DRA_U.Launch_LLA[1];
  LIFTSv2_DRA_Y.Predicted_LLA[2] = LIFTSv2_DRA_U.Launch_LLA[2] +
    LIFTSv2_DRA_DW.xhat[2];

  // Outport: '<Root>/Orientation_Quaternion' incorporates:
  //   MATLAB Function: '<Root>/MATLAB Function'

  LIFTSv2_DRA_Y.Orientation_Quaternion[0] = LIFTSv2_DRA_DW.q_nav[0];
  LIFTSv2_DRA_Y.Orientation_Quaternion[1] = LIFTSv2_DRA_DW.q_nav[1];
  LIFTSv2_DRA_Y.Orientation_Quaternion[2] = LIFTSv2_DRA_DW.q_nav[2];
  LIFTSv2_DRA_Y.Orientation_Quaternion[3] = LIFTSv2_DRA_DW.q_nav[3];

  // Outport: '<Root>/Displacement_XYZ' incorporates:
  //   MATLAB Function: '<Root>/MATLAB Function'

  LIFTSv2_DRA_Y.Displacement_XYZ[0] = LIFTSv2_DRA_DW.xhat[0];
  LIFTSv2_DRA_Y.Displacement_XYZ[1] = LIFTSv2_DRA_DW.xhat[1];
  LIFTSv2_DRA_Y.Displacement_XYZ[2] = LIFTSv2_DRA_DW.xhat[2];
}

// Model initialize function
void LIFTSv2_DRA::initialize()
{
  {
    int32_T i;
    static const int8_T tmp[81]{ 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10 };

    // SystemInitialize for MATLAB Function: '<Root>/MATLAB Function'
    for (i = 0; i < 81; i++) {
      LIFTSv2_DRA_DW.P[i] = tmp[i];
    }

    LIFTSv2_DRA_DW.q_nav[0] = 1.0F;
    LIFTSv2_DRA_DW.q_nav[1] = 0.0F;
    LIFTSv2_DRA_DW.q_nav[2] = 0.0F;
    LIFTSv2_DRA_DW.q_nav[3] = 0.0F;

    // End of SystemInitialize for MATLAB Function: '<Root>/MATLAB Function'
  }
}

// Model terminate function
void LIFTSv2_DRA::terminate()
{
  // (no terminate code required)
}

const char_T* LIFTSv2_DRA::RT_MODEL_LIFTSv2_DRA_T::getErrorStatus() const
{
  return (errorStatus);
}

void LIFTSv2_DRA::RT_MODEL_LIFTSv2_DRA_T::setErrorStatus(const char_T* const
  volatile aErrorStatus)
{
  (errorStatus = aErrorStatus);
}

// Constructor
LIFTSv2_DRA::LIFTSv2_DRA() :
  LIFTSv2_DRA_U(),
  LIFTSv2_DRA_Y(),
  LIFTSv2_DRA_DW(),
  LIFTSv2_DRA_M()
{
  // Currently there is no constructor body generated.
}

// Destructor
// Currently there is no destructor body generated.
LIFTSv2_DRA::~LIFTSv2_DRA() = default;

// Real-Time Model get method
LIFTSv2_DRA::RT_MODEL_LIFTSv2_DRA_T * LIFTSv2_DRA::getRTM()
{
  return (&LIFTSv2_DRA_M);
}

//
// File trailer for generated code.
//
// [EOF]
//
