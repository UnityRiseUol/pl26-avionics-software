//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: LIFTSv2_DRA.h
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
#ifndef LIFTSV2_DRA_H_
#define LIFTSV2_DRA_H_
#include <cmath>
#include "rtwtypes.h"
//#include "rtw_continuous.h"
//#include "rtw_solver.h"
#include "rt_nonfinite.h"
#include "LIFTSv2_DRA_types.h"

extern "C"
{

#include "rtGetInf.h"

}

// Class declaration for model LIFTSv2_DRA
class LIFTSv2_DRA final
{
  // public data and function members
 public:
  // Block states (default storage) for system '<Root>'
  struct DW_LIFTSv2_DRA_T {
    real32_T xhat[9];                  // '<Root>/MATLAB Function'
    real32_T P[81];                    // '<Root>/MATLAB Function'
    real32_T q_nav[4];                 // '<Root>/MATLAB Function'
    real32_T pos_nav[3];               // '<Root>/MATLAB Function'
    real32_T vel_nav[3];               // '<Root>/MATLAB Function'
    real32_T t_accum;                  // '<Root>/MATLAB Function'
  };

  // External inputs (root inport signals with default storage)
  struct ExtU_LIFTSv2_DRA_T {
    real32_T Accel_XYZ[3];             // '<Root>/Accel_XYZ'
    real32_T Gyro_XYZ[3];              // '<Root>/Gyro_XYZ'
    real32_T Mag_XYZ[3];               // '<Root>/Mag_XYZ'
    boolean_T Gps_Valid;               // '<Root>/Gps_Valid'
    real32_T Gps_Lat;                  // '<Root>/Gps_Lat'
    real32_T Gps_Lon;                  // '<Root>/Gps_Lon'
    real32_T Gps_Alt;                  // '<Root>/Gps_Alt'
    real32_T Launch_LLA[3];            // '<Root>/Launch_LLA'
    real32_T Bmp_Alt;                  // '<Root>/Bmp_Alt'
    real32_T Bmp_Vspeed;               // '<Root>/Bmp_Vspeed'
  };

  // External outputs (root outports fed by signals with default storage)
  struct ExtY_LIFTSv2_DRA_T {
    real32_T Displacement_XYZ[3];      // '<Root>/Displacement_XYZ'
    real32_T Orientation_Quaternion[4];// '<Root>/Orientation_Quaternion'
    real32_T Predicted_LLA[3];         // '<Root>/Predicted_LLA'
  };

  // Real-time Model Data Structure
  struct RT_MODEL_LIFTSv2_DRA_T {
    const char_T * volatile errorStatus;
    const char_T* getErrorStatus() const;
    void setErrorStatus(const char_T* const volatile aErrorStatus);
  };

  // Copy Constructor
  LIFTSv2_DRA(LIFTSv2_DRA const&) = delete;

  // Assignment Operator
  LIFTSv2_DRA& operator= (LIFTSv2_DRA const&) & = delete;

  // Move Constructor
  LIFTSv2_DRA(LIFTSv2_DRA &&) = delete;

  // Move Assignment Operator
  LIFTSv2_DRA& operator= (LIFTSv2_DRA &&) = delete;

  // Real-Time Model get method
  LIFTSv2_DRA::RT_MODEL_LIFTSv2_DRA_T * getRTM();

  // Root inports set method
  void setExternalInputs(const ExtU_LIFTSv2_DRA_T *pExtU_LIFTSv2_DRA_T)
  {
    LIFTSv2_DRA_U = *pExtU_LIFTSv2_DRA_T;
  }

  // Root outports get method
  const ExtY_LIFTSv2_DRA_T &getExternalOutputs() const
  {
    return LIFTSv2_DRA_Y;
  }

  // model initialize function
  void initialize();

  // model step function
  void step();

  // model terminate function
  static void terminate();

  // Constructor
  LIFTSv2_DRA();

  // Destructor
  ~LIFTSv2_DRA();

  // private data and function members
 private:
  // External inputs
  ExtU_LIFTSv2_DRA_T LIFTSv2_DRA_U;

  // External outputs
  ExtY_LIFTSv2_DRA_T LIFTSv2_DRA_Y;

  // Block states
  DW_LIFTSv2_DRA_T LIFTSv2_DRA_DW;

  // private member function(s) for subsystem '<Root>'
  real32_T LIFTSv2_DRA_norm(const real32_T x[3]);

  // Real-Time Model
  RT_MODEL_LIFTSv2_DRA_T LIFTSv2_DRA_M;
};

//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'LIFTSv2_DRA'
//  '<S1>'   : 'LIFTSv2_DRA/MATLAB Function'

#endif                                 // LIFTSV2_DRA_H_

//
// File trailer for generated code.
//
// [EOF]
//
