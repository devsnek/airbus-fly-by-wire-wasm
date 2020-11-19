#ifndef RTW_HEADER_FlyByWire_h_
#define RTW_HEADER_FlyByWire_h_
#include <cmath>
#include <cstring>
#ifndef FlyByWire_COMMON_INCLUDES_
# define FlyByWire_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif

#include "FlyByWire_types.h"

typedef struct {
  real_T flare_Theta_c_deg;
  real_T flare_Theta_c_rate_deg_s;
} BlockIO_FlyByWire_T;

typedef struct {
  real_T DiscreteTransferFcn_states;
  real_T DelayInput2_DSTATE;
  real_T UD_DSTATE;
  real_T Integration_DSTATE;
  real_T Integration_DSTATE_e;
  real_T DelayInput2_DSTATE_h;
  real_T DiscreteTimeIntegrator_DSTATE;
  real_T DiscreteTransferFcn2_states;
  real_T DiscreteTransferFcn1_states;
  real_T PrevY;
  real_T PrevY_c;
  real_T PrevY_f;
  real_T PrevY_h;
  real_T PrevY_j;
  real_T PrevY_p;
  real_T PrevY_d;
  real_T PrevY_jg;
  real_T PrevY_g;
  int32_T chartAbsoluteTimeCounter;
  int32_T durationLastReferenceTick_1;
  int8_T Integration_PrevResetState;
  int8_T Integration_PrevResetState_p;
  int8_T DiscreteTimeIntegrator_PrevResetState;
  uint8_T Integration_IC_LOADING;
  uint8_T Integration_IC_LOADING_a;
  uint8_T DiscreteTimeIntegrator_IC_LOADING;
  uint8_T is_active_c5_FlyByWire;
  uint8_T is_c5_FlyByWire;
  uint8_T is_active_c7_FlyByWire;
  uint8_T is_c7_FlyByWire;
  uint8_T is_active_c8_FlyByWire;
  uint8_T is_c8_FlyByWire;
  uint8_T is_active_c3_FlyByWire;
  uint8_T is_c3_FlyByWire;
  uint8_T is_active_c9_FlyByWire;
  uint8_T is_c9_FlyByWire;
  uint8_T is_active_c2_FlyByWire;
  uint8_T is_c2_FlyByWire;
  uint8_T is_active_c1_FlyByWire;
  uint8_T is_c1_FlyByWire;
  boolean_T condWasTrueAtLastTimeStep_1;
} D_Work_FlyByWire_T;

typedef struct {
  fbw_input in;
} ExternalInputs_FlyByWire_T;

typedef struct {
  fbw_output out;
} ExternalOutputs_FlyByWire_T;

struct Parameters_FlyByWire_T_ {
  fbw_output fbw_output_MATLABStruct;
  real_T PID_DifferentiatorICPrevScaledInput;
  real_T PID_LowerSaturationLimit;
  real_T PID_UpperSaturationLimit;
  real_T Constant_Value;
  real_T Constant_Value_m;
  real_T Gain_Gain;
  real_T Saturation_UpperSat;
  real_T Saturation_LowerSat;
  real_T Theta_max3_Value;
  real_T Gain3_Gain;
  real_T Saturation2_UpperSat;
  real_T Saturation2_LowerSat;
  real_T GainTheta_Gain;
  real_T GainPhi_Gain;
  real_T Gain_Gain_n;
  real_T Gainqk_Gain;
  real_T Gain_Gain_l;
  real_T Gain_Gain_a;
  real_T Gainpk_Gain;
  real_T Gain_Gain_e;
  real_T Gainqk1_Gain;
  real_T Gain_Gain_aw;
  real_T Gain_Gain_nm;
  real_T Gainpk1_Gain;
  real_T Gainpk2_Gain;
  real_T Gainpk3_Gain;
  real_T Gain_Gain_i;
  real_T Constant_Value_g;
  real_T Saturation_UpperSat_e;
  real_T Saturation_LowerSat_e;
  real_T Gain1_Gain;
  real_T Saturation1_UpperSat;
  real_T Saturation1_LowerSat;
  real_T Gain2_Gain;
  real_T Saturation2_UpperSat_b;
  real_T Saturation2_LowerSat_g;
  real_T RateLimitereta_RisingLim;
  real_T RateLimitereta_FallingLim;
  real_T RateLimitereta_IC;
  real_T Gaineta_Gain;
  real_T RateLimiterxi_RisingLim;
  real_T RateLimiterxi_FallingLim;
  real_T RateLimiterxi_IC;
  real_T Gainxi_Gain;
  real_T RateLimiterxi1_RisingLim;
  real_T RateLimiterxi1_FallingLim;
  real_T RateLimiterxi1_IC;
  real_T Gainxi1_Gain;
  real_T Gain_Gain_d;
  real_T DiscreteTransferFcn_NumCoef;
  real_T DiscreteTransferFcn_DenCoef[2];
  real_T DiscreteTransferFcn_InitialStates;
  real_T Constant1_Value;
  real_T Constant_Value_j;
  real_T Saturation_UpperSat_er;
  real_T Saturation_LowerSat_a;
  real_T RateLimit_RisingLim;
  real_T RateLimit_FallingLim;
  real_T RateLimit_IC;
  real_T DelayInput2_InitialCondition;
  real_T sampletime_WtEt;
  real_T K7857_Gain;
  real_T K7958_Gain;
  real_T u1_Value;
  real_T K7659_Gain;
  real_T F193LUT_tableData[8];
  real_T F193LUT_bp01Data[8];
  real_T u7_Value;
  real_T K7765_Gain;
  real_T Gain1_Gain_j;
  real_T uDLookupTable_tableData[5];
  real_T uDLookupTable_bp01Data[5];
  real_T Gain1_Gain_p;
  real_T Saturation_UpperSat_d;
  real_T Saturation_LowerSat_p;
  real_T Gain1_Gain_b;
  real_T Theta_max1_Value;
  real_T Gain2_Gain_g;
  real_T Gain1_Gain_d;
  real_T Saturation1_UpperSat_h;
  real_T Saturation1_LowerSat_o;
  real_T Loaddemand_tableData[3];
  real_T Loaddemand_bp01Data[3];
  real_T Switch_Threshold;
  real_T PLUT_tableData[2];
  real_T PLUT_bp01Data[2];
  real_T DLUT_tableData[2];
  real_T DLUT_bp01Data[2];
  real_T TSamp_WtEt;
  real_T Integration_gainval;
  real_T Integration_UpperSat;
  real_T Integration_LowerSat;
  real_T Saturation_UpperSat_j;
  real_T Saturation_LowerSat_c;
  real_T Constant_Value_o;
  real_T Integration_gainval_c;
  real_T Integration_UpperSat_f;
  real_T Integration_LowerSat_f;
  real_T DelayInput2_InitialCondition_d;
  real_T sampletime_WtEt_f;
  real_T Gain_Gain_c;
  real_T Gain1_Gain_jh;
  real_T Saturation_UpperSat_p;
  real_T Saturation_LowerSat_h;
  real_T RateLimit_RisingLim_b;
  real_T RateLimit_FallingLim_m;
  real_T RateLimit_IC_d;
  real_T Gain1_Gain_m;
  real_T BankAngleProtection_tableData[7];
  real_T BankAngleProtection_bp01Data[7];
  real_T Saturation_UpperSat_n;
  real_T Saturation_LowerSat_o;
  real_T DiscreteTimeIntegrator_gainval;
  real_T DiscreteTimeIntegrator_UpperSat;
  real_T DiscreteTimeIntegrator_LowerSat;
  real_T Gain2_Gain_i;
  real_T Gain1_Gain_mg;
  real_T pKp_Gain;
  real_T RateLimiter2_RisingLim;
  real_T RateLimiter2_FallingLim;
  real_T RateLimiter2_IC;
  real_T Gain5_Gain;
  real_T Constant2_Value;
  real_T Gain1_Gain_br;
  real_T Gain1_Gain_c;
  real_T Saturation_UpperSat_l;
  real_T Saturation_LowerSat_l;
  real_T Gain6_Gain;
  real_T Gain_Gain_cd;
  real_T DiscreteTransferFcn2_NumCoef;
  real_T DiscreteTransferFcn2_DenCoef[2];
  real_T DiscreteTransferFcn2_InitialStates;
  real_T Gain_Gain_h;
  real_T Saturation1_UpperSat_ho;
  real_T Saturation1_LowerSat_g;
  real_T DiscreteTransferFcn1_NumCoef[2];
  real_T DiscreteTransferFcn1_DenCoef[2];
  real_T DiscreteTransferFcn1_InitialStates;
  real_T Gain6_Gain_k;
  real_T Saturation2_UpperSat_e;
  real_T Saturation2_LowerSat_gp;
  real_T Saturation_UpperSat_i;
  real_T Saturation_LowerSat_al;
  real_T Constant_Value_c;
  real_T Saturation_UpperSat_id;
  real_T Saturation_LowerSat_n;
  real_T Constant_Value_f;
  real_T RateLimitereta_RisingLim_j;
  real_T RateLimitereta_FallingLim_m;
  real_T RateLimitereta_IC_c;
  real_T Gaineta_Gain_d;
  real_T Limitereta_UpperSat;
  real_T Limitereta_LowerSat;
  real_T GainiH_Gain;
  real_T LimiteriH_UpperSat;
  real_T LimiteriH_LowerSat;
  real_T RateLimiterxi_RisingLim_n;
  real_T RateLimiterxi_FallingLim_f;
  real_T RateLimiterxi_IC_c;
  real_T Gainxi_Gain_n;
  real_T Limiterxi_UpperSat;
  real_T Limiterxi_LowerSat;
  real_T RateLimiterxi1_RisingLim_k;
  real_T RateLimiterxi1_FallingLim_o;
  real_T RateLimiterxi1_IC_i;
  real_T Gainxi1_Gain_e;
  real_T Limiterxi1_UpperSat;
  real_T Limiterxi1_LowerSat;
  real_T Gainxi2_Gain;
  real_T Limiterxi2_UpperSat;
  real_T Limiterxi2_LowerSat;
  real_T Gain_Gain_ip;
  uint8_T ManualSwitch_CurrentSetting;
  uint8_T ManualSwitch1_CurrentSetting;
};

extern const fbw_input FlyByWire_rtZfbw_input;
extern const fbw_output FlyByWire_rtZfbw_output;
class FlyByWireModelClass {
 public:
  ExternalInputs_FlyByWire_T FlyByWire_U;
  ExternalOutputs_FlyByWire_T FlyByWire_Y;
  void initialize();
  void step();
  void terminate();
  FlyByWireModelClass();
  ~FlyByWireModelClass();
 private:
  static Parameters_FlyByWire_T FlyByWire_P;
  BlockIO_FlyByWire_T FlyByWire_B;
  D_Work_FlyByWire_T FlyByWire_DWork;
};

#endif

