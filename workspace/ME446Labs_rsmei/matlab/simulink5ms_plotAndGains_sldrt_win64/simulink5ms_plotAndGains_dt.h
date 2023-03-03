/*
 * simulink5ms_plotAndGains_dt.h
 *
 * Classroom License -- for classroom instructional use only.  Not for
 * government, commercial, academic research, or other organizational use.
 *
 * Code generation for model "simulink5ms_plotAndGains".
 *
 * Model version              : 9.2
 * Simulink Coder version : 9.7 (R2022a) 13-Nov-2021
 * C source code generated on : Fri Mar  3 11:12:32 2023
 *
 * Target selection: sldrt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "ext_types.h"

/* data type size table */
static uint_T rtDataTypeSizes[] = {
  sizeof(real_T),
  sizeof(real32_T),
  sizeof(int8_T),
  sizeof(uint8_T),
  sizeof(int16_T),
  sizeof(uint16_T),
  sizeof(int32_T),
  sizeof(uint32_T),
  sizeof(boolean_T),
  sizeof(fcn_call_T),
  sizeof(int_T),
  sizeof(pointer_T),
  sizeof(action_T),
  2*sizeof(uint32_T),
  sizeof(int32_T),
  sizeof(int64_T),
  sizeof(uint64_T),
  sizeof(uint64_T),
  sizeof(int64_T),
  sizeof(char_T),
  sizeof(uchar_T),
  sizeof(time_T)
};

/* data type name table */
static const char_T * rtDataTypeNames[] = {
  "real_T",
  "real32_T",
  "int8_T",
  "uint8_T",
  "int16_T",
  "uint16_T",
  "int32_T",
  "uint32_T",
  "boolean_T",
  "fcn_call_T",
  "int_T",
  "pointer_T",
  "action_T",
  "timer_uint32_pair_T",
  "physical_connection",
  "int64_T",
  "uint64_T",
  "uint64_T",
  "int64_T",
  "char_T",
  "uchar_T",
  "time_T"
};

/* data type transitions for block I/O structure */
static DataTypeTransition rtBTransitions[] = {
  { (char_T *)(&simulink5ms_plotAndGains_B.plot1), 0, 0, 4 },

  { (char_T *)(&simulink5ms_plotAndGains_B.PacketInput1_o1[0]), 6, 0, 5 },

  { (char_T *)(&simulink5ms_plotAndGains_B.Gain1), 4, 0, 7 }
  ,

  { (char_T *)(&simulink5ms_plotAndGains_DW.PacketInput1_PWORK), 11, 0, 7 }
};

/* data type transition table for block I/O structure */
static DataTypeTransitionTable rtBTransTable = {
  4U,
  rtBTransitions
};

/* data type transitions for Parameters structure */
static DataTypeTransition rtPTransitions[] = {
  { (char_T *)(&simulink5ms_plotAndGains_P.PacketInput1_MaxMissedTicks), 0, 0, 4
  },

  { (char_T *)(&simulink5ms_plotAndGains_P.PacketInput1_PacketID), 6, 0, 2 },

  { (char_T *)(&simulink5ms_plotAndGains_P.Gain1_Gain), 0, 0, 29 },

  { (char_T *)(&simulink5ms_plotAndGains_P.ConstantMustbeThisValue0x7fff_Value),
    4, 0, 1 }
};

/* data type transition table for Parameters structure */
static DataTypeTransitionTable rtPTransTable = {
  4U,
  rtPTransitions
};

/* [EOF] simulink5ms_plotAndGains_dt.h */
