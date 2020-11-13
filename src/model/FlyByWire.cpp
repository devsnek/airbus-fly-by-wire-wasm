#include "FlyByWire.h"
#include "FlyByWire_private.h"

const uint8_T FlyByWire_IN_InAir = 1U;
const uint8_T FlyByWire_IN_NO_ACTIVE_CHILD = 0U;
const uint8_T FlyByWire_IN_OnGround = 2U;
const uint8_T FlyByWire_IN_Flare_Reduce_Theta_c = 1U;
const uint8_T FlyByWire_IN_Flare_Set_Rate = 2U;
const uint8_T FlyByWire_IN_Flare_Store_Theta_c_deg = 3U;
const uint8_T FlyByWire_IN_Flight_High = 4U;
const uint8_T FlyByWire_IN_Flight_Low = 5U;
const uint8_T FlyByWire_IN_Ground = 6U;
const uint8_T FlyByWire_IN_frozen = 1U;
const uint8_T FlyByWire_IN_running = 2U;
const uint8_T FlyByWire_IN_Flight = 1U;
const uint8_T FlyByWire_IN_Ground_p = 2U;
const uint8_T FlyByWire_IN_automatic = 1U;
const uint8_T FlyByWire_IN_manual = 2U;
const uint8_T FlyByWire_IN_reset = 3U;
const uint8_T FlyByWire_IN_flight_clean = 1U;
const uint8_T FlyByWire_IN_flight_flaps = 2U;
const uint8_T FlyByWire_IN_ground = 3U;
const uint8_T FlyByWire_IN_FlightMode = 1U;
const uint8_T FlyByWire_IN_GroundMode = 2U;
const fbw_output FlyByWire_rtZfbw_output = {
  {
    {
      {
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0
      },

      {
        0.0,
        0.0,
        0.0
      },

      {
        0.0,
        0.0,
        false,
        0.0,
        0.0,
        0.0
      }
    },

    {
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0
    },

    {
      0.0
    },

    {
      0.0,
      0.0,
      0.0
    }
  },

  {
    {
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      false,
      false,
      0.0,
      false,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0
    },

    {
      0.0,
      0.0,
      0.0
    },

    {
      {
        0.0
      },

      {
        0.0
      }
    },

    {
      0.0
    },

    {
      0.0
    },

    {
      0.0,
      0.0
    }
  },

  {
    {
      0.0,
      0.0
    },

    {
      0.0,
      0.0,
      0.0,
      0.0
    },

    {
      0.0,
      0.0
    }
  }
} ;

const fbw_input FlyByWire_rtZfbw_input = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 } };

real_T look1_binlxpw(real_T u0, const real_T bp0[], const real_T table[], uint32_T maxIndex)
{
  real_T frac;
  uint32_T iRght;
  uint32_T iLeft;
  uint32_T bpIdx;
  if (u0 <= bp0[0U]) {
    iLeft = 0U;
    frac = (u0 - bp0[0U]) / (bp0[1U] - bp0[0U]);
  } else if (u0 < bp0[maxIndex]) {
    bpIdx = maxIndex >> 1U;
    iLeft = 0U;
    iRght = maxIndex;
    while (iRght - iLeft > 1U) {
      if (u0 < bp0[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = (iRght + iLeft) >> 1U;
    }

    frac = (u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]);
  } else {
    iLeft = maxIndex - 1U;
    frac = (u0 - bp0[maxIndex - 1U]) / (bp0[maxIndex] - bp0[maxIndex - 1U]);
  }

  return (table[iLeft + 1U] - table[iLeft]) * frac + table[iLeft];
}

void FlyByWireModelClass::step()
{
  real_T numAccum;
  real_T Phi;
  real_T result[3];
  real_T result_0[3];
  real_T rtb_DiscreteTimeIntegrator;
  real_T rtb_Product;
  real_T rtb_Integration_m;
  real_T rtb_u8;
  real_T rtb_LimiteriH;
  real_T rtb_Limiterxi;
  real_T rtb_PProdOut;
  real_T rtb_BusAssignment_mk_roll_law_normal_xi_pos;
  real_T rtb_GainTheta;
  real_T rtb_GainPhi;
  real_T rtb_Gainqk;
  real_T rtb_Gain;
  real_T rtb_Gainpk;
  real_T rtb_Gaineta;
  real_T rtb_Gainxi;
  int32_T rtb_on_ground;
  int32_T rtb_in_flight;
  real_T rtb_BusAssignment_f_roll_output_xi_pos;
  int32_T rtb_in_flight_o;
  real_T rtb_UkYk1;
  int32_T rtb_in_flare;
  real_T rtb_nz_limit_up_g;
  int32_T rtb_nz_limit_lo_g;
  boolean_T rtb_eta_trim_deg_should_freeze;
  boolean_T rtb_eta_trim_deg_reset;
  real_T rtb_eta_trim_deg_reset_deg;
  boolean_T rtb_eta_trim_deg_should_write;
  real_T rtb_eta_trim_deg_rate_limit_up_deg_s;
  real_T rtb_eta_trim_deg_rate_limit_lo_deg_s;
  real_T rtb_BusAssignment_c_pitch_data_computed_in_flight_gain;
  real_T rtb_TSamp;
  real_T BusAssignment_sim_input_delta_zeta_pos;
  real_T result_tmp;
  real_T tmp[9];
  rtb_GainTheta = FlyByWire_P.GainTheta_Gain * FlyByWire_U.in.data.Theta_deg;
  rtb_GainPhi = FlyByWire_P.GainPhi_Gain * FlyByWire_U.in.data.Phi_deg;
  rtb_Gainqk = FlyByWire_P.Gain_Gain_n * FlyByWire_U.in.data.q_rad_s * FlyByWire_P.Gainqk_Gain;
  rtb_Gain = FlyByWire_P.Gain_Gain_l * FlyByWire_U.in.data.r_rad_s;
  rtb_Gainpk = FlyByWire_P.Gain_Gain_a * FlyByWire_U.in.data.p_rad_s * FlyByWire_P.Gainpk_Gain;
  rtb_Gaineta = 0.017453292519943295 * rtb_GainTheta;
  Phi = 0.017453292519943295 * rtb_GainPhi;
  rtb_Gainxi = std::tan(rtb_Gaineta);
  result_tmp = std::sin(Phi);
  Phi = std::cos(Phi);
  tmp[0] = 1.0;
  tmp[3] = result_tmp * rtb_Gainxi;
  tmp[6] = Phi * rtb_Gainxi;
  tmp[1] = 0.0;
  tmp[4] = Phi;
  tmp[7] = -result_tmp;
  tmp[2] = 0.0;
  rtb_Integration_m = 1.0 / std::cos(rtb_Gaineta);
  tmp[5] = rtb_Integration_m * result_tmp;
  tmp[8] = rtb_Integration_m * Phi;
  for (rtb_on_ground = 0; rtb_on_ground < 3; rtb_on_ground++) {
    result[rtb_on_ground] = tmp[rtb_on_ground + 6] * rtb_Gain + (tmp[rtb_on_ground + 3] * rtb_Gainqk + tmp[rtb_on_ground]
      * rtb_Gainpk);
  }

  rtb_Gaineta = 0.017453292519943295 * rtb_GainTheta;
  Phi = 0.017453292519943295 * rtb_GainPhi;
  rtb_Gainxi = std::tan(rtb_Gaineta);
  result_tmp = std::sin(Phi);
  Phi = std::cos(Phi);
  tmp[0] = 1.0;
  tmp[3] = result_tmp * rtb_Gainxi;
  tmp[6] = Phi * rtb_Gainxi;
  tmp[1] = 0.0;
  tmp[4] = Phi;
  tmp[7] = -result_tmp;
  tmp[2] = 0.0;
  rtb_Integration_m = 1.0 / std::cos(rtb_Gaineta);
  tmp[5] = rtb_Integration_m * result_tmp;
  tmp[8] = rtb_Integration_m * Phi;
  result_tmp = FlyByWire_P.Gain_Gain_nm * FlyByWire_U.in.data.p_dot_rad_s2 * FlyByWire_P.Gainpk1_Gain;
  rtb_Gaineta = FlyByWire_P.Gain_Gain_e * FlyByWire_U.in.data.q_dot_rad_s2 * FlyByWire_P.Gainqk1_Gain;
  rtb_Gainxi = FlyByWire_P.Gain_Gain_aw * FlyByWire_U.in.data.r_dot_rad_s2;
  for (rtb_on_ground = 0; rtb_on_ground < 3; rtb_on_ground++) {
    result_0[rtb_on_ground] = tmp[rtb_on_ground + 6] * rtb_Gainxi + (tmp[rtb_on_ground + 3] * rtb_Gaineta +
      tmp[rtb_on_ground] * result_tmp);
  }

  rtb_Limiterxi = FlyByWire_P.Gainpk2_Gain * FlyByWire_U.in.data.eta_trim_deg;
  rtb_Integration_m = FlyByWire_P.Gain1_Gain * FlyByWire_U.in.data.gear_animation_pos_1 - FlyByWire_P.Constant_Value_g;
  if (rtb_Integration_m > FlyByWire_P.Saturation1_UpperSat) {
    rtb_Integration_m = FlyByWire_P.Saturation1_UpperSat;
  } else {
    if (rtb_Integration_m < FlyByWire_P.Saturation1_LowerSat) {
      rtb_Integration_m = FlyByWire_P.Saturation1_LowerSat;
    }
  }

  result_tmp = FlyByWire_P.Gain2_Gain * FlyByWire_U.in.data.gear_animation_pos_2 - FlyByWire_P.Constant_Value_g;
  if (result_tmp > FlyByWire_P.Saturation2_UpperSat) {
    result_tmp = FlyByWire_P.Saturation2_UpperSat;
  } else {
    if (result_tmp < FlyByWire_P.Saturation2_LowerSat) {
      result_tmp = FlyByWire_P.Saturation2_LowerSat;
    }
  }

  rtb_PProdOut = FlyByWire_U.in.input.delta_eta_pos - FlyByWire_DWork.PrevY;
  if (rtb_PProdOut > FlyByWire_P.RateLimitereta_RisingLim) {
    rtb_DiscreteTimeIntegrator = FlyByWire_DWork.PrevY + FlyByWire_P.RateLimitereta_RisingLim;
  } else if (rtb_PProdOut < FlyByWire_P.RateLimitereta_FallingLim) {
    rtb_DiscreteTimeIntegrator = FlyByWire_DWork.PrevY + FlyByWire_P.RateLimitereta_FallingLim;
  } else {
    rtb_DiscreteTimeIntegrator = FlyByWire_U.in.input.delta_eta_pos;
  }

  FlyByWire_DWork.PrevY = rtb_DiscreteTimeIntegrator;
  rtb_Gaineta = FlyByWire_P.Gaineta_Gain * rtb_DiscreteTimeIntegrator;
  rtb_PProdOut = FlyByWire_U.in.input.delta_xi_pos - FlyByWire_DWork.PrevY_c;
  if (rtb_PProdOut > FlyByWire_P.RateLimiterxi_RisingLim) {
    rtb_DiscreteTimeIntegrator = FlyByWire_DWork.PrevY_c + FlyByWire_P.RateLimiterxi_RisingLim;
  } else if (rtb_PProdOut < FlyByWire_P.RateLimiterxi_FallingLim) {
    rtb_DiscreteTimeIntegrator = FlyByWire_DWork.PrevY_c + FlyByWire_P.RateLimiterxi_FallingLim;
  } else {
    rtb_DiscreteTimeIntegrator = FlyByWire_U.in.input.delta_xi_pos;
  }

  FlyByWire_DWork.PrevY_c = rtb_DiscreteTimeIntegrator;
  rtb_Gainxi = FlyByWire_P.Gainxi_Gain * rtb_DiscreteTimeIntegrator;
  rtb_PProdOut = FlyByWire_U.in.input.delta_zeta_pos - FlyByWire_DWork.PrevY_f;
  if (rtb_PProdOut > FlyByWire_P.RateLimiterxi1_RisingLim) {
    rtb_DiscreteTimeIntegrator = FlyByWire_DWork.PrevY_f + FlyByWire_P.RateLimiterxi1_RisingLim;
  } else if (rtb_PProdOut < FlyByWire_P.RateLimiterxi1_FallingLim) {
    rtb_DiscreteTimeIntegrator = FlyByWire_DWork.PrevY_f + FlyByWire_P.RateLimiterxi1_FallingLim;
  } else {
    rtb_DiscreteTimeIntegrator = FlyByWire_U.in.input.delta_zeta_pos;
  }

  FlyByWire_DWork.PrevY_f = rtb_DiscreteTimeIntegrator;
  if (FlyByWire_DWork.is_active_c1_FlyByWire == 0U) {
    FlyByWire_DWork.is_active_c1_FlyByWire = 1U;
    FlyByWire_DWork.is_c1_FlyByWire = FlyByWire_IN_OnGround;
    rtb_on_ground = 1;
  } else if (FlyByWire_DWork.is_c1_FlyByWire == FlyByWire_IN_InAir) {
    if ((rtb_Integration_m > 0.1) || (result_tmp > 0.1)) {
      FlyByWire_DWork.is_c1_FlyByWire = FlyByWire_IN_OnGround;
      rtb_on_ground = 1;
    } else {
      rtb_on_ground = 0;
    }
  } else {
    if ((rtb_Integration_m == 0.0) && (result_tmp == 0.0)) {
      FlyByWire_DWork.is_c1_FlyByWire = FlyByWire_IN_InAir;
      rtb_on_ground = 0;
    } else {
      rtb_on_ground = 1;
    }
  }

  FlyByWire_Y.out.sim.data.eta_trim_deg = rtb_Limiterxi;
  Phi = FlyByWire_P.Gainpk3_Gain * FlyByWire_U.in.data.zeta_trim_pos;
  FlyByWire_Y.out.sim.data.gear_strut_compression_1 = rtb_Integration_m;
  BusAssignment_sim_input_delta_zeta_pos = rtb_DiscreteTimeIntegrator;
  FlyByWire_DWork.chartAbsoluteTimeCounter++;
  rtb_eta_trim_deg_should_freeze = ((rtb_on_ground == 1) && (rtb_GainTheta < 2.5));
  if ((!rtb_eta_trim_deg_should_freeze) || (!FlyByWire_DWork.condWasTrueAtLastTimeStep_1)) {
    FlyByWire_DWork.durationLastReferenceTick_1 = FlyByWire_DWork.chartAbsoluteTimeCounter;
  }

  FlyByWire_DWork.condWasTrueAtLastTimeStep_1 = rtb_eta_trim_deg_should_freeze;
  if (FlyByWire_DWork.is_active_c3_FlyByWire == 0U) {
    FlyByWire_DWork.chartAbsoluteTimeCounter = 0;
    FlyByWire_DWork.is_active_c3_FlyByWire = 1U;
    FlyByWire_DWork.is_c3_FlyByWire = FlyByWire_IN_Ground_p;
    rtb_in_flight_o = 0;
  } else if (FlyByWire_DWork.is_c3_FlyByWire == FlyByWire_IN_Flight) {
    rtb_eta_trim_deg_should_freeze = ((rtb_on_ground == 1) && (rtb_GainTheta < 2.5));
    if ((!rtb_eta_trim_deg_should_freeze) || (!FlyByWire_DWork.condWasTrueAtLastTimeStep_1)) {
      FlyByWire_DWork.durationLastReferenceTick_1 = FlyByWire_DWork.chartAbsoluteTimeCounter;
    }

    FlyByWire_DWork.condWasTrueAtLastTimeStep_1 = rtb_eta_trim_deg_should_freeze;
    if (FlyByWire_DWork.chartAbsoluteTimeCounter - FlyByWire_DWork.durationLastReferenceTick_1 >= 250) {
      FlyByWire_DWork.is_c3_FlyByWire = FlyByWire_IN_Ground_p;
      rtb_in_flight_o = 0;
    } else {
      rtb_in_flight_o = 1;
    }
  } else {
    if (((rtb_on_ground == 0) && (rtb_GainTheta > 8.0)) || (FlyByWire_U.in.data.H_radio_ft > 400.0)) {
      FlyByWire_DWork.durationLastReferenceTick_1 = FlyByWire_DWork.chartAbsoluteTimeCounter;
      FlyByWire_DWork.is_c3_FlyByWire = FlyByWire_IN_Flight;
      rtb_in_flight_o = 1;
      FlyByWire_DWork.condWasTrueAtLastTimeStep_1 = ((rtb_on_ground == 1) && (rtb_GainTheta < 2.5));
    } else {
      rtb_in_flight_o = 0;
    }
  }

  numAccum = FlyByWire_P.DiscreteTransferFcn_NumCoef * FlyByWire_DWork.DiscreteTransferFcn_states;
  if (FlyByWire_P.ManualSwitch_CurrentSetting == 1) {
    rtb_DiscreteTimeIntegrator = FlyByWire_P.Constant1_Value;
  } else {
    rtb_DiscreteTimeIntegrator = FlyByWire_P.Constant_Value_j;
  }

  if (FlyByWire_DWork.is_active_c2_FlyByWire == 0U) {
    FlyByWire_DWork.is_active_c2_FlyByWire = 1U;
    FlyByWire_DWork.is_c2_FlyByWire = FlyByWire_IN_Ground;
    rtb_in_flare = 0;
    FlyByWire_B.flare_Theta_c_deg = numAccum;
    FlyByWire_B.flare_Theta_c_rate_deg_s = -3.0;
  } else {
    switch (FlyByWire_DWork.is_c2_FlyByWire) {
     case FlyByWire_IN_Flare_Reduce_Theta_c:
      if (rtb_in_flight_o == 0) {
        FlyByWire_DWork.is_c2_FlyByWire = FlyByWire_IN_Ground;
        rtb_in_flare = 0;
        FlyByWire_B.flare_Theta_c_deg = numAccum;
        FlyByWire_B.flare_Theta_c_rate_deg_s = -3.0;
      } else if (((FlyByWire_U.in.data.H_radio_ft > 50.0) && (rtb_GainTheta > 8.0)) || ((FlyByWire_U.in.data.H_radio_ft >
        50.0) && (rtb_DiscreteTimeIntegrator == 0.0))) {
        FlyByWire_DWork.is_c2_FlyByWire = FlyByWire_IN_Flight_Low;
        rtb_in_flare = 0;
        FlyByWire_B.flare_Theta_c_deg = numAccum;
        FlyByWire_B.flare_Theta_c_rate_deg_s = -3.0;
      } else {
        rtb_in_flare = 1;
        FlyByWire_B.flare_Theta_c_deg = -2.0;
      }
      break;

     case FlyByWire_IN_Flare_Set_Rate:
      if (FlyByWire_P.ManualSwitch1_CurrentSetting == 1) {
        rtb_Integration_m = FlyByWire_P.Constant1_Value;
      } else {
        rtb_Integration_m = FlyByWire_P.Constant_Value_j;
      }

      if ((FlyByWire_U.in.data.H_radio_ft <= 30.0) || (rtb_Integration_m == 1.0)) {
        FlyByWire_DWork.is_c2_FlyByWire = FlyByWire_IN_Flare_Reduce_Theta_c;
        rtb_in_flare = 1;
        FlyByWire_B.flare_Theta_c_deg = -2.0;
      } else if (((FlyByWire_U.in.data.H_radio_ft > 50.0) && (rtb_GainTheta > 8.0)) || ((FlyByWire_U.in.data.H_radio_ft >
        50.0) && (rtb_DiscreteTimeIntegrator == 0.0))) {
        FlyByWire_DWork.is_c2_FlyByWire = FlyByWire_IN_Flight_Low;
        rtb_in_flare = 0;
        FlyByWire_B.flare_Theta_c_deg = numAccum;
        FlyByWire_B.flare_Theta_c_rate_deg_s = -3.0;
      } else {
        rtb_in_flare = 1;
      }
      break;

     case FlyByWire_IN_Flare_Store_Theta_c_deg:
      if (((FlyByWire_U.in.data.H_radio_ft > 50.0) && (rtb_GainTheta > 8.0)) || ((FlyByWire_U.in.data.H_radio_ft > 50.0)
           && (rtb_DiscreteTimeIntegrator == 0.0))) {
        FlyByWire_DWork.is_c2_FlyByWire = FlyByWire_IN_Flight_Low;
        rtb_in_flare = 0;
        FlyByWire_B.flare_Theta_c_deg = numAccum;
        FlyByWire_B.flare_Theta_c_rate_deg_s = -3.0;
      } else {
        FlyByWire_B.flare_Theta_c_rate_deg_s = -(numAccum + 2.0) / 8.0;
        FlyByWire_DWork.is_c2_FlyByWire = FlyByWire_IN_Flare_Set_Rate;
        rtb_in_flare = 1;
      }
      break;

     case FlyByWire_IN_Flight_High:
      if ((FlyByWire_U.in.data.H_radio_ft <= 50.0) || (rtb_DiscreteTimeIntegrator == 1.0)) {
        FlyByWire_B.flare_Theta_c_deg = numAccum;
        FlyByWire_DWork.is_c2_FlyByWire = FlyByWire_IN_Flare_Store_Theta_c_deg;
        rtb_in_flare = 1;
      } else {
        rtb_in_flare = 0;
        FlyByWire_B.flare_Theta_c_deg = numAccum;
        FlyByWire_B.flare_Theta_c_rate_deg_s = -3.0;
      }
      break;

     case FlyByWire_IN_Flight_Low:
      if (FlyByWire_U.in.data.H_radio_ft > 50.0) {
        FlyByWire_DWork.is_c2_FlyByWire = FlyByWire_IN_Flight_High;
        rtb_in_flare = 0;
        FlyByWire_B.flare_Theta_c_deg = numAccum;
        FlyByWire_B.flare_Theta_c_rate_deg_s = -3.0;
      } else {
        rtb_in_flare = 0;
        FlyByWire_B.flare_Theta_c_deg = numAccum;
        FlyByWire_B.flare_Theta_c_rate_deg_s = -3.0;
      }
      break;

     default:
      if (rtb_in_flight_o == 1) {
        FlyByWire_DWork.is_c2_FlyByWire = FlyByWire_IN_Flight_Low;
        rtb_in_flare = 0;
        FlyByWire_B.flare_Theta_c_deg = numAccum;
        FlyByWire_B.flare_Theta_c_rate_deg_s = -3.0;
      } else {
        rtb_in_flare = 0;
        FlyByWire_B.flare_Theta_c_deg = numAccum;
        FlyByWire_B.flare_Theta_c_rate_deg_s = -3.0;
      }
      break;
    }
  }

  if (FlyByWire_DWork.is_active_c7_FlyByWire == 0U) {
    FlyByWire_DWork.is_active_c7_FlyByWire = 1U;
    FlyByWire_DWork.is_c7_FlyByWire = FlyByWire_IN_ground;
    rtb_eta_trim_deg_rate_limit_up_deg_s = 0.7;
    rtb_eta_trim_deg_rate_limit_lo_deg_s = -0.7;
    rtb_nz_limit_up_g = 2.0;
    rtb_nz_limit_lo_g = 0;
  } else {
    switch (FlyByWire_DWork.is_c7_FlyByWire) {
     case FlyByWire_IN_flight_clean:
      if (FlyByWire_U.in.data.flaps_handle_index != 0.0) {
        FlyByWire_DWork.is_c7_FlyByWire = FlyByWire_IN_flight_flaps;
        rtb_eta_trim_deg_rate_limit_up_deg_s = 0.7;
        rtb_eta_trim_deg_rate_limit_lo_deg_s = -0.7;
        rtb_nz_limit_up_g = 2.0;
        rtb_nz_limit_lo_g = 0;
      } else if ((rtb_in_flight_o == 0) && (!(FlyByWire_U.in.data.flaps_handle_index != 0.0))) {
        FlyByWire_DWork.is_c7_FlyByWire = FlyByWire_IN_ground;
        rtb_eta_trim_deg_rate_limit_up_deg_s = 0.7;
        rtb_eta_trim_deg_rate_limit_lo_deg_s = -0.7;
        rtb_nz_limit_up_g = 2.0;
        rtb_nz_limit_lo_g = 0;
      } else {
        rtb_eta_trim_deg_rate_limit_up_deg_s = 0.3;
        rtb_eta_trim_deg_rate_limit_lo_deg_s = -0.3;
        rtb_nz_limit_up_g = 2.5;
        rtb_nz_limit_lo_g = -1;
      }
      break;

     case FlyByWire_IN_flight_flaps:
      if (!(FlyByWire_U.in.data.flaps_handle_index != 0.0)) {
        FlyByWire_DWork.is_c7_FlyByWire = FlyByWire_IN_flight_clean;
        rtb_eta_trim_deg_rate_limit_up_deg_s = 0.3;
        rtb_eta_trim_deg_rate_limit_lo_deg_s = -0.3;
        rtb_nz_limit_up_g = 2.5;
        rtb_nz_limit_lo_g = -1;
      } else if (rtb_in_flight_o == 0) {
        FlyByWire_DWork.is_c7_FlyByWire = FlyByWire_IN_ground;
        rtb_eta_trim_deg_rate_limit_up_deg_s = 0.7;
        rtb_eta_trim_deg_rate_limit_lo_deg_s = -0.7;
        rtb_nz_limit_up_g = 2.0;
        rtb_nz_limit_lo_g = 0;
      } else {
        rtb_eta_trim_deg_rate_limit_up_deg_s = 0.7;
        rtb_eta_trim_deg_rate_limit_lo_deg_s = -0.7;
        rtb_nz_limit_up_g = 2.0;
        rtb_nz_limit_lo_g = 0;
      }
      break;

     default:
      if ((rtb_in_flight_o != 0) && (!(FlyByWire_U.in.data.flaps_handle_index != 0.0))) {
        FlyByWire_DWork.is_c7_FlyByWire = FlyByWire_IN_flight_clean;
        rtb_eta_trim_deg_rate_limit_up_deg_s = 0.3;
        rtb_eta_trim_deg_rate_limit_lo_deg_s = -0.3;
        rtb_nz_limit_up_g = 2.5;
        rtb_nz_limit_lo_g = -1;
      } else if ((rtb_in_flight_o != 0) && (FlyByWire_U.in.data.flaps_handle_index != 0.0)) {
        FlyByWire_DWork.is_c7_FlyByWire = FlyByWire_IN_flight_flaps;
        rtb_eta_trim_deg_rate_limit_up_deg_s = 0.7;
        rtb_eta_trim_deg_rate_limit_lo_deg_s = -0.7;
        rtb_nz_limit_up_g = 2.0;
        rtb_nz_limit_lo_g = 0;
      } else {
        rtb_eta_trim_deg_rate_limit_up_deg_s = 0.7;
        rtb_eta_trim_deg_rate_limit_lo_deg_s = -0.7;
        rtb_nz_limit_up_g = 2.0;
        rtb_nz_limit_lo_g = 0;
      }
      break;
    }
  }

  if (FlyByWire_DWork.is_active_c9_FlyByWire == 0U) {
    FlyByWire_DWork.is_active_c9_FlyByWire = 1U;
    FlyByWire_DWork.is_c9_FlyByWire = FlyByWire_IN_running;
    rtb_eta_trim_deg_should_freeze = false;
  } else if (FlyByWire_DWork.is_c9_FlyByWire == FlyByWire_IN_frozen) {
    if ((rtb_in_flare == 0) && (FlyByWire_U.in.data.nz_g < 1.25) && (FlyByWire_U.in.data.nz_g > 0.5) && (std::abs
         (rtb_GainPhi) <= 30.0)) {
      FlyByWire_DWork.is_c9_FlyByWire = FlyByWire_IN_running;
      rtb_eta_trim_deg_should_freeze = false;
    } else {
      rtb_eta_trim_deg_should_freeze = true;
    }
  } else {
    if ((rtb_in_flare != 0) || (FlyByWire_U.in.data.nz_g >= 1.25) || (FlyByWire_U.in.data.nz_g <= 0.5) || (std::abs
         (rtb_GainPhi) > 30.0)) {
      FlyByWire_DWork.is_c9_FlyByWire = FlyByWire_IN_frozen;
      rtb_eta_trim_deg_should_freeze = true;
    } else {
      rtb_eta_trim_deg_should_freeze = false;
    }
  }

  if (FlyByWire_DWork.is_active_c8_FlyByWire == 0U) {
    FlyByWire_DWork.is_active_c8_FlyByWire = 1U;
    FlyByWire_DWork.is_c8_FlyByWire = FlyByWire_IN_manual;
    rtb_eta_trim_deg_reset = true;
    rtb_eta_trim_deg_reset_deg = rtb_Limiterxi;
    rtb_eta_trim_deg_should_write = false;
  } else {
    switch (FlyByWire_DWork.is_c8_FlyByWire) {
     case FlyByWire_IN_automatic:
      if (rtb_in_flight_o == 0) {
        FlyByWire_DWork.is_c8_FlyByWire = FlyByWire_IN_reset;
        rtb_eta_trim_deg_reset = true;
        rtb_eta_trim_deg_reset_deg = 0.0;
        rtb_eta_trim_deg_should_write = true;
      } else {
        rtb_eta_trim_deg_reset = false;
        rtb_eta_trim_deg_reset_deg = rtb_Limiterxi;
        rtb_eta_trim_deg_should_write = true;
      }
      break;

     case FlyByWire_IN_manual:
      if (rtb_in_flight_o != 0) {
        FlyByWire_DWork.is_c8_FlyByWire = FlyByWire_IN_automatic;
        rtb_eta_trim_deg_reset = false;
        rtb_eta_trim_deg_reset_deg = rtb_Limiterxi;
        rtb_eta_trim_deg_should_write = true;
      } else {
        rtb_eta_trim_deg_reset = true;
        rtb_eta_trim_deg_reset_deg = rtb_Limiterxi;
        rtb_eta_trim_deg_should_write = false;
      }
      break;

     default:
      if ((rtb_in_flight_o == 0) && (rtb_Limiterxi == 0.0)) {
        FlyByWire_DWork.is_c8_FlyByWire = FlyByWire_IN_manual;
        rtb_eta_trim_deg_reset = true;
        rtb_eta_trim_deg_reset_deg = rtb_Limiterxi;
        rtb_eta_trim_deg_should_write = false;
      } else {
        rtb_eta_trim_deg_reset = true;
        rtb_eta_trim_deg_reset_deg = 0.0;
        rtb_eta_trim_deg_should_write = true;
      }
      break;
    }
  }

  if (FlyByWire_DWork.is_active_c5_FlyByWire == 0U) {
    FlyByWire_DWork.is_active_c5_FlyByWire = 1U;
    FlyByWire_DWork.is_c5_FlyByWire = FlyByWire_IN_GroundMode;
    rtb_in_flight = 0;
  } else if (FlyByWire_DWork.is_c5_FlyByWire == FlyByWire_IN_FlightMode) {
    if (rtb_on_ground == 1) {
      FlyByWire_DWork.is_c5_FlyByWire = FlyByWire_IN_GroundMode;
      rtb_in_flight = 0;
    } else {
      rtb_in_flight = 1;
    }
  } else {
    if (((rtb_on_ground == 0) && (rtb_GainTheta > 8.0)) || (FlyByWire_U.in.data.H_radio_ft > 400.0)) {
      FlyByWire_DWork.is_c5_FlyByWire = FlyByWire_IN_FlightMode;
      rtb_in_flight = 1;
    } else {
      rtb_in_flight = 0;
    }
  }

  if (rtb_in_flight > FlyByWire_P.Saturation_UpperSat_p) {
    rtb_DiscreteTimeIntegrator = FlyByWire_P.Saturation_UpperSat_p;
  } else if (rtb_in_flight < FlyByWire_P.Saturation_LowerSat_h) {
    rtb_DiscreteTimeIntegrator = FlyByWire_P.Saturation_LowerSat_h;
  } else {
    rtb_DiscreteTimeIntegrator = rtb_in_flight;
  }

  rtb_PProdOut = rtb_DiscreteTimeIntegrator - FlyByWire_DWork.PrevY_j;
  if (rtb_PProdOut > FlyByWire_P.RateLimit_RisingLim) {
    rtb_DiscreteTimeIntegrator = FlyByWire_DWork.PrevY_j + FlyByWire_P.RateLimit_RisingLim;
  } else {
    if (rtb_PProdOut < FlyByWire_P.RateLimit_FallingLim) {
      rtb_DiscreteTimeIntegrator = FlyByWire_DWork.PrevY_j + FlyByWire_P.RateLimit_FallingLim;
    }
  }

  FlyByWire_DWork.PrevY_j = rtb_DiscreteTimeIntegrator;
  rtb_Limiterxi = FlyByWire_P.Gain1_Gain_m * rtb_Gainxi + look1_binlxpw(rtb_GainPhi,
    FlyByWire_P.BankAngleProtection_bp01Data, FlyByWire_P.BankAngleProtection_tableData, 6U);
  if (rtb_Limiterxi > FlyByWire_P.Saturation_UpperSat_n) {
    rtb_Limiterxi = FlyByWire_P.Saturation_UpperSat_n;
  } else {
    if (rtb_Limiterxi < FlyByWire_P.Saturation_LowerSat_o) {
      rtb_Limiterxi = FlyByWire_P.Saturation_LowerSat_o;
    }
  }

  rtb_Product = rtb_Limiterxi * rtb_DiscreteTimeIntegrator;
  if (FlyByWire_DWork.DiscreteTimeIntegrator_IC_LOADING != 0) {
    FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE = rtb_GainPhi;
    if (FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE >= FlyByWire_P.DiscreteTimeIntegrator_UpperSat) {
      FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE = FlyByWire_P.DiscreteTimeIntegrator_UpperSat;
    } else {
      if (FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE <= FlyByWire_P.DiscreteTimeIntegrator_LowerSat) {
        FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE = FlyByWire_P.DiscreteTimeIntegrator_LowerSat;
      }
    }
  }

  if ((rtb_DiscreteTimeIntegrator > 0.0) && (FlyByWire_DWork.DiscreteTimeIntegrator_PrevResetState <= 0)) {
    FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE = rtb_GainPhi;
    if (FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE >= FlyByWire_P.DiscreteTimeIntegrator_UpperSat) {
      FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE = FlyByWire_P.DiscreteTimeIntegrator_UpperSat;
    } else {
      if (FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE <= FlyByWire_P.DiscreteTimeIntegrator_LowerSat) {
        FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE = FlyByWire_P.DiscreteTimeIntegrator_LowerSat;
      }
    }
  }

  if (FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE >= FlyByWire_P.DiscreteTimeIntegrator_UpperSat) {
    FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE = FlyByWire_P.DiscreteTimeIntegrator_UpperSat;
  } else {
    if (FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE <= FlyByWire_P.DiscreteTimeIntegrator_LowerSat) {
      FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE = FlyByWire_P.DiscreteTimeIntegrator_LowerSat;
    }
  }

  rtb_BusAssignment_mk_roll_law_normal_xi_pos = (FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE - rtb_GainPhi) *
    FlyByWire_P.Gain2_Gain_i + FlyByWire_P.Gain1_Gain_mg * result[0] * FlyByWire_P.pKp_Gain;
  if (rtb_DiscreteTimeIntegrator > FlyByWire_P.Saturation_UpperSat_f) {
    rtb_Integration_m = FlyByWire_P.Saturation_UpperSat_f;
  } else if (rtb_DiscreteTimeIntegrator < FlyByWire_P.Saturation_LowerSat_ea) {
    rtb_Integration_m = FlyByWire_P.Saturation_LowerSat_ea;
  } else {
    rtb_Integration_m = rtb_DiscreteTimeIntegrator;
  }

  rtb_Limiterxi = rtb_BusAssignment_mk_roll_law_normal_xi_pos * rtb_Integration_m;
  rtb_Integration_m -= FlyByWire_P.Constant_Value_f;
  rtb_Integration_m *= rtb_Gainxi;
  rtb_BusAssignment_f_roll_output_xi_pos = rtb_Limiterxi + rtb_Integration_m;
  if (rtb_in_flight_o > FlyByWire_P.Saturation_UpperSat_er) {
    rtb_Integration_m = FlyByWire_P.Saturation_UpperSat_er;
  } else if (rtb_in_flight_o < FlyByWire_P.Saturation_LowerSat_a) {
    rtb_Integration_m = FlyByWire_P.Saturation_LowerSat_a;
  } else {
    rtb_Integration_m = rtb_in_flight_o;
  }

  rtb_PProdOut = rtb_Integration_m - FlyByWire_DWork.PrevY_h;
  if (rtb_PProdOut > FlyByWire_P.RateLimit_RisingLim_b) {
    rtb_Integration_m = FlyByWire_DWork.PrevY_h + FlyByWire_P.RateLimit_RisingLim_b;
  } else {
    if (rtb_PProdOut < FlyByWire_P.RateLimit_FallingLim_e) {
      rtb_Integration_m = FlyByWire_DWork.PrevY_h + FlyByWire_P.RateLimit_FallingLim_e;
    }
  }

  FlyByWire_DWork.PrevY_h = rtb_Integration_m;
  rtb_Limiterxi = std::abs(FlyByWire_B.flare_Theta_c_rate_deg_s) * FlyByWire_P.sampletime_WtEt;
  rtb_UkYk1 = FlyByWire_B.flare_Theta_c_deg - FlyByWire_DWork.DelayInput2_DSTATE;
  if (!(rtb_UkYk1 > rtb_Limiterxi)) {
    rtb_Limiterxi = FlyByWire_B.flare_Theta_c_rate_deg_s * FlyByWire_P.sampletime_WtEt;
    if (!(rtb_UkYk1 < rtb_Limiterxi)) {
      rtb_Limiterxi = rtb_UkYk1;
    }
  }

  FlyByWire_DWork.DelayInput2_DSTATE += rtb_Limiterxi;
  rtb_BusAssignment_c_pitch_data_computed_in_flight_gain = rtb_Integration_m;
  rtb_Integration_m = FlyByWire_P.K7857_Gain * result_0[1] + FlyByWire_P.K7958_Gain * result[1];
  rtb_Limiterxi = look1_binlxpw(FlyByWire_U.in.data.V_tas_kn, FlyByWire_P.F193LUT_bp01Data,
    FlyByWire_P.F193LUT_tableData, 7U);
  rtb_UkYk1 = ((FlyByWire_P.u1_Value + rtb_GainTheta) * FlyByWire_P.K7659_Gain + rtb_Integration_m) * rtb_Limiterxi;
  rtb_u8 = ((rtb_GainTheta - FlyByWire_P.u7_Value) * FlyByWire_P.K7765_Gain + rtb_Integration_m) * rtb_Limiterxi;
  rtb_Integration_m = FlyByWire_P.Gain1_Gain_j * result[1] * look1_binlxpw(FlyByWire_U.in.data.V_tas_kn,
    FlyByWire_P.uDLookupTable_bp01Data, FlyByWire_P.uDLookupTable_tableData, 4U) + FlyByWire_U.in.data.nz_g;
  rtb_Limiterxi = look1_binlxpw(rtb_GainTheta, FlyByWire_P.uDLookupTable_bp01Data_c,
    FlyByWire_P.uDLookupTable_tableData_a, 3U);
  if (!(rtb_Gaineta > rtb_Limiterxi)) {
    rtb_Limiterxi = look1_binlxpw(rtb_GainTheta, FlyByWire_P.uDLookupTable2_bp01Data,
      FlyByWire_P.uDLookupTable2_tableData, 3U);
    if (!(rtb_Gaineta < rtb_Limiterxi)) {
      rtb_Limiterxi = rtb_Gaineta;
    }
  }

  rtb_Limiterxi = look1_binlxpw(rtb_Limiterxi, FlyByWire_P.Loaddemand_bp01Data, FlyByWire_P.Loaddemand_tableData, 2U);
  if (rtb_in_flare > FlyByWire_P.Switch_Threshold) {
    rtb_LimiteriH = (FlyByWire_DWork.DelayInput2_DSTATE - rtb_GainTheta) * FlyByWire_P.Gain_Gain;
    if (rtb_LimiteriH > FlyByWire_P.Saturation_UpperSat) {
      rtb_LimiteriH = FlyByWire_P.Saturation_UpperSat;
    } else {
      if (rtb_LimiteriH < FlyByWire_P.Saturation_LowerSat) {
        rtb_LimiteriH = FlyByWire_P.Saturation_LowerSat;
      }
    }
  } else {
    rtb_LimiteriH = FlyByWire_P.Constant_Value_m;
  }

  if (rtb_GainPhi > FlyByWire_P.Saturation_UpperSat_d) {
    rtb_TSamp = FlyByWire_P.Saturation_UpperSat_d;
  } else if (rtb_GainPhi < FlyByWire_P.Saturation_LowerSat_p) {
    rtb_TSamp = FlyByWire_P.Saturation_LowerSat_p;
  } else {
    rtb_TSamp = rtb_GainPhi;
  }

  rtb_Limiterxi = ((std::cos(FlyByWire_P.Gain1_Gain_p * rtb_GainTheta) / std::cos(FlyByWire_P.Gain1_Gain_b * rtb_TSamp)
                    + rtb_Limiterxi) + rtb_LimiteriH) + look1_binlxpw(rtb_GainTheta, FlyByWire_P.uDLookupTable1_bp01Data,
    FlyByWire_P.uDLookupTable1_tableData, 8U);
  if (rtb_Limiterxi > rtb_nz_limit_up_g) {
    rtb_Limiterxi = rtb_nz_limit_up_g;
  } else {
    if (rtb_Limiterxi < rtb_nz_limit_lo_g) {
      rtb_Limiterxi = rtb_nz_limit_lo_g;
    }
  }

  rtb_LimiteriH = rtb_Integration_m - rtb_Limiterxi;
  rtb_PProdOut = rtb_LimiteriH * look1_binlxpw(FlyByWire_U.in.data.V_tas_kn, FlyByWire_P.PLUT_bp01Data,
    FlyByWire_P.PLUT_tableData, 1U);
  rtb_LimiteriH *= look1_binlxpw(FlyByWire_U.in.data.V_tas_kn, FlyByWire_P.DLUT_bp01Data, FlyByWire_P.DLUT_tableData, 1U);
  rtb_TSamp = rtb_LimiteriH * FlyByWire_P.TSamp_WtEt;
  rtb_LimiteriH = (rtb_TSamp - FlyByWire_DWork.UD_DSTATE) + rtb_PProdOut;
  if (rtb_LimiteriH > FlyByWire_P.PID_UpperSaturationLimit) {
    rtb_LimiteriH = FlyByWire_P.PID_UpperSaturationLimit;
  } else {
    if (rtb_LimiteriH < FlyByWire_P.PID_LowerSaturationLimit) {
      rtb_LimiteriH = FlyByWire_P.PID_LowerSaturationLimit;
    }
  }

  FlyByWire_Y.out.pitch.law_normal.Cstar_g = rtb_Integration_m;
  rtb_PProdOut = rtb_LimiteriH - FlyByWire_DWork.PrevY_i;
  if (rtb_PProdOut > FlyByWire_P.Limiter_RisingLim) {
    rtb_Integration_m = FlyByWire_DWork.PrevY_i + FlyByWire_P.Limiter_RisingLim;
  } else if (rtb_PProdOut < FlyByWire_P.Limiter_FallingLim) {
    rtb_Integration_m = FlyByWire_DWork.PrevY_i + FlyByWire_P.Limiter_FallingLim;
  } else {
    rtb_Integration_m = rtb_LimiteriH;
  }

  FlyByWire_DWork.PrevY_i = rtb_Integration_m;
  if (rtb_Integration_m > FlyByWire_P.Saturation_UpperSat_h) {
    rtb_Integration_m = FlyByWire_P.Saturation_UpperSat_h;
  } else {
    if (rtb_Integration_m < FlyByWire_P.Saturation_LowerSat_l) {
      rtb_Integration_m = FlyByWire_P.Saturation_LowerSat_l;
    }
  }

  if (rtIsNaN(rtb_Integration_m)) {
    rtb_Integration_m = (rtNaN);
  }

  if (FlyByWire_DWork.Integration_IC_LOADING != 0) {
    FlyByWire_DWork.Integration_DSTATE = rtb_Gaineta;
    if (FlyByWire_DWork.Integration_DSTATE >= FlyByWire_P.Integration_UpperSat) {
      FlyByWire_DWork.Integration_DSTATE = FlyByWire_P.Integration_UpperSat;
    } else {
      if (FlyByWire_DWork.Integration_DSTATE <= FlyByWire_P.Integration_LowerSat) {
        FlyByWire_DWork.Integration_DSTATE = FlyByWire_P.Integration_LowerSat;
      }
    }
  }

  if ((rtb_in_flight_o > 0) && (FlyByWire_DWork.Integration_PrevResetState <= 0)) {
    FlyByWire_DWork.Integration_DSTATE = rtb_Gaineta;
    if (FlyByWire_DWork.Integration_DSTATE >= FlyByWire_P.Integration_UpperSat) {
      FlyByWire_DWork.Integration_DSTATE = FlyByWire_P.Integration_UpperSat;
    } else {
      if (FlyByWire_DWork.Integration_DSTATE <= FlyByWire_P.Integration_LowerSat) {
        FlyByWire_DWork.Integration_DSTATE = FlyByWire_P.Integration_LowerSat;
      }
    }
  }

  if (FlyByWire_DWork.Integration_DSTATE >= FlyByWire_P.Integration_UpperSat) {
    FlyByWire_DWork.Integration_DSTATE = FlyByWire_P.Integration_UpperSat;
  } else {
    if (FlyByWire_DWork.Integration_DSTATE <= FlyByWire_P.Integration_LowerSat) {
      FlyByWire_DWork.Integration_DSTATE = FlyByWire_P.Integration_LowerSat;
    }
  }

  FlyByWire_Y.out.pitch.law_normal.Cstar_c_g = rtb_Limiterxi;
  FlyByWire_Y.out.pitch.law_normal.eta_dot_pos_s = rtb_LimiteriH;
  if (rtb_BusAssignment_c_pitch_data_computed_in_flight_gain > FlyByWire_P.Saturation_UpperSat_pr) {
    rtb_Limiterxi = FlyByWire_P.Saturation_UpperSat_pr;
  } else if (rtb_BusAssignment_c_pitch_data_computed_in_flight_gain < FlyByWire_P.Saturation_LowerSat_k) {
    rtb_Limiterxi = FlyByWire_P.Saturation_LowerSat_k;
  } else {
    rtb_Limiterxi = rtb_BusAssignment_c_pitch_data_computed_in_flight_gain;
  }

  rtb_LimiteriH = FlyByWire_DWork.Integration_DSTATE * rtb_Limiterxi;
  rtb_Limiterxi -= FlyByWire_P.Constant_Value_k;
  rtb_Limiterxi *= rtb_Gaineta;
  rtb_LimiteriH += rtb_Limiterxi;
  rtb_Limiterxi = rtb_eta_trim_deg_rate_limit_up_deg_s * FlyByWire_P.sampletime_WtEt_f;
  if (FlyByWire_DWork.Integration_IC_LOADING_a != 0) {
    FlyByWire_DWork.Integration_DSTATE_e = rtb_eta_trim_deg_reset_deg;
    if (FlyByWire_DWork.Integration_DSTATE_e >= FlyByWire_P.Integration_UpperSat_f) {
      FlyByWire_DWork.Integration_DSTATE_e = FlyByWire_P.Integration_UpperSat_f;
    } else {
      if (FlyByWire_DWork.Integration_DSTATE_e <= FlyByWire_P.Integration_LowerSat_f) {
        FlyByWire_DWork.Integration_DSTATE_e = FlyByWire_P.Integration_LowerSat_f;
      }
    }
  }

  if (rtb_eta_trim_deg_reset || (FlyByWire_DWork.Integration_PrevResetState_p != 0)) {
    FlyByWire_DWork.Integration_DSTATE_e = rtb_eta_trim_deg_reset_deg;
    if (FlyByWire_DWork.Integration_DSTATE_e >= FlyByWire_P.Integration_UpperSat_f) {
      FlyByWire_DWork.Integration_DSTATE_e = FlyByWire_P.Integration_UpperSat_f;
    } else {
      if (FlyByWire_DWork.Integration_DSTATE_e <= FlyByWire_P.Integration_LowerSat_f) {
        FlyByWire_DWork.Integration_DSTATE_e = FlyByWire_P.Integration_LowerSat_f;
      }
    }
  }

  if (FlyByWire_DWork.Integration_DSTATE_e >= FlyByWire_P.Integration_UpperSat_f) {
    FlyByWire_DWork.Integration_DSTATE_e = FlyByWire_P.Integration_UpperSat_f;
  } else {
    if (FlyByWire_DWork.Integration_DSTATE_e <= FlyByWire_P.Integration_LowerSat_f) {
      FlyByWire_DWork.Integration_DSTATE_e = FlyByWire_P.Integration_LowerSat_f;
    }
  }

  rtb_PProdOut = FlyByWire_DWork.Integration_DSTATE_e - FlyByWire_DWork.DelayInput2_DSTATE_h;
  if (!(rtb_PProdOut > rtb_Limiterxi)) {
    rtb_Limiterxi = rtb_eta_trim_deg_rate_limit_lo_deg_s * FlyByWire_P.sampletime_WtEt_f;
    if (!(rtb_PProdOut < rtb_Limiterxi)) {
      rtb_Limiterxi = rtb_PProdOut;
    }
  }

  FlyByWire_DWork.DelayInput2_DSTATE_h += rtb_Limiterxi;
  rtb_PProdOut = rtb_LimiteriH - FlyByWire_DWork.PrevY_d;
  if (rtb_PProdOut > FlyByWire_P.RateLimitereta_RisingLim_j) {
    rtb_Limiterxi = FlyByWire_DWork.PrevY_d + FlyByWire_P.RateLimitereta_RisingLim_j;
  } else if (rtb_PProdOut < FlyByWire_P.RateLimitereta_FallingLim_m) {
    rtb_Limiterxi = FlyByWire_DWork.PrevY_d + FlyByWire_P.RateLimitereta_FallingLim_m;
  } else {
    rtb_Limiterxi = rtb_LimiteriH;
  }

  FlyByWire_DWork.PrevY_d = rtb_Limiterxi;
  rtb_Limiterxi *= FlyByWire_P.Gaineta_Gain_d;
  if (rtb_Limiterxi > FlyByWire_P.Limitereta_UpperSat) {
    FlyByWire_Y.out.sim.raw.output.eta_pos = FlyByWire_P.Limitereta_UpperSat;
  } else if (rtb_Limiterxi < FlyByWire_P.Limitereta_LowerSat) {
    FlyByWire_Y.out.sim.raw.output.eta_pos = FlyByWire_P.Limitereta_LowerSat;
  } else {
    FlyByWire_Y.out.sim.raw.output.eta_pos = rtb_Limiterxi;
  }

  rtb_PProdOut = rtb_BusAssignment_f_roll_output_xi_pos - FlyByWire_DWork.PrevY_jg;
  if (rtb_PProdOut > FlyByWire_P.RateLimiterxi_RisingLim_n) {
    rtb_Limiterxi = FlyByWire_DWork.PrevY_jg + FlyByWire_P.RateLimiterxi_RisingLim_n;
  } else if (rtb_PProdOut < FlyByWire_P.RateLimiterxi_FallingLim_f) {
    rtb_Limiterxi = FlyByWire_DWork.PrevY_jg + FlyByWire_P.RateLimiterxi_FallingLim_f;
  } else {
    rtb_Limiterxi = rtb_BusAssignment_f_roll_output_xi_pos;
  }

  FlyByWire_DWork.PrevY_jg = rtb_Limiterxi;
  rtb_Limiterxi *= FlyByWire_P.Gainxi_Gain_n;
  if (rtb_Limiterxi > FlyByWire_P.Limiterxi_UpperSat) {
    FlyByWire_Y.out.sim.raw.output.xi_pos = FlyByWire_P.Limiterxi_UpperSat;
  } else if (rtb_Limiterxi < FlyByWire_P.Limiterxi_LowerSat) {
    FlyByWire_Y.out.sim.raw.output.xi_pos = FlyByWire_P.Limiterxi_LowerSat;
  } else {
    FlyByWire_Y.out.sim.raw.output.xi_pos = rtb_Limiterxi;
  }

  rtb_PProdOut = BusAssignment_sim_input_delta_zeta_pos - FlyByWire_DWork.PrevY_g;
  if (rtb_PProdOut > FlyByWire_P.RateLimiterxi1_RisingLim_k) {
    rtb_Limiterxi = FlyByWire_DWork.PrevY_g + FlyByWire_P.RateLimiterxi1_RisingLim_k;
  } else if (rtb_PProdOut < FlyByWire_P.RateLimiterxi1_FallingLim_o) {
    rtb_Limiterxi = FlyByWire_DWork.PrevY_g + FlyByWire_P.RateLimiterxi1_FallingLim_o;
  } else {
    rtb_Limiterxi = BusAssignment_sim_input_delta_zeta_pos;
  }

  FlyByWire_DWork.PrevY_g = rtb_Limiterxi;
  rtb_Limiterxi *= FlyByWire_P.Gainxi1_Gain;
  if (rtb_Limiterxi > FlyByWire_P.Limiterxi1_UpperSat) {
    FlyByWire_Y.out.sim.raw.output.zeta_pos = FlyByWire_P.Limiterxi1_UpperSat;
  } else if (rtb_Limiterxi < FlyByWire_P.Limiterxi1_LowerSat) {
    FlyByWire_Y.out.sim.raw.output.zeta_pos = FlyByWire_P.Limiterxi1_LowerSat;
  } else {
    FlyByWire_Y.out.sim.raw.output.zeta_pos = rtb_Limiterxi;
  }

  FlyByWire_Y.out.sim.raw.data = FlyByWire_U.in.data;
  FlyByWire_Y.out.sim.raw.input = FlyByWire_U.in.input;
  rtb_Limiterxi = FlyByWire_P.GainiH_Gain * FlyByWire_DWork.DelayInput2_DSTATE_h;
  if (rtb_Limiterxi > FlyByWire_P.LimiteriH_UpperSat) {
    FlyByWire_Y.out.sim.raw.output.eta_trim_deg = FlyByWire_P.LimiteriH_UpperSat;
  } else if (rtb_Limiterxi < FlyByWire_P.LimiteriH_LowerSat) {
    FlyByWire_Y.out.sim.raw.output.eta_trim_deg = FlyByWire_P.LimiteriH_LowerSat;
  } else {
    FlyByWire_Y.out.sim.raw.output.eta_trim_deg = rtb_Limiterxi;
  }

  FlyByWire_Y.out.sim.raw.output.eta_trim_deg_should_write = rtb_eta_trim_deg_should_write;
  rtb_Limiterxi = FlyByWire_P.Gainxi2_Gain * Phi;
  if (rtb_Limiterxi > FlyByWire_P.Limiterxi2_UpperSat) {
    FlyByWire_Y.out.sim.raw.output.zeta_trim_pos = FlyByWire_P.Limiterxi2_UpperSat;
  } else if (rtb_Limiterxi < FlyByWire_P.Limiterxi2_LowerSat) {
    FlyByWire_Y.out.sim.raw.output.zeta_trim_pos = FlyByWire_P.Limiterxi2_LowerSat;
  } else {
    FlyByWire_Y.out.sim.raw.output.zeta_trim_pos = rtb_Limiterxi;
  }

  FlyByWire_Y.out.sim.data.nz_g = FlyByWire_U.in.data.nz_g;
  FlyByWire_Y.out.sim.data.Theta_deg = rtb_GainTheta;
  FlyByWire_Y.out.sim.data.Phi_deg = rtb_GainPhi;
  FlyByWire_Y.out.sim.data.q_deg_s = rtb_Gainqk;
  FlyByWire_Y.out.sim.data.r_deg_s = rtb_Gain;
  FlyByWire_Y.out.sim.data.p_deg_s = rtb_Gainpk;
  FlyByWire_Y.out.sim.data.qk_deg_s = result[1];
  FlyByWire_Y.out.sim.data.rk_deg_s = result[2];
  FlyByWire_Y.out.sim.data.pk_deg_s = result[0];
  FlyByWire_Y.out.sim.data.qk_dot_deg_s2 = result_0[1];
  FlyByWire_Y.out.sim.data.rk_dot_deg_s2 = result_0[2];
  FlyByWire_Y.out.sim.data.pk_dot_deg_s2 = result_0[0];
  FlyByWire_Y.out.sim.data.zeta_trim_pos = Phi;
  FlyByWire_Y.out.sim.data.alpha_deg = FlyByWire_U.in.data.alpha_deg;
  FlyByWire_Y.out.sim.data.beta_deg = FlyByWire_U.in.data.beta_deg;
  FlyByWire_Y.out.sim.data.V_ias_kn = FlyByWire_U.in.data.V_ias_kn;
  FlyByWire_Y.out.sim.data.V_tas_kn = FlyByWire_U.in.data.V_tas_kn;
  FlyByWire_Y.out.sim.data.V_mach = FlyByWire_U.in.data.V_mach;
  FlyByWire_Y.out.sim.data.H_ft = FlyByWire_U.in.data.H_ft;
  FlyByWire_Y.out.sim.data.H_ind_ft = FlyByWire_U.in.data.H_ind_ft;
  FlyByWire_Y.out.sim.data.H_radio_ft = FlyByWire_U.in.data.H_radio_ft;
  FlyByWire_Y.out.sim.data.CG_percent_MAC = FlyByWire_U.in.data.CG_percent_MAC;
  rtb_Limiterxi = FlyByWire_P.Gain_Gain_i * FlyByWire_U.in.data.gear_animation_pos_0 - FlyByWire_P.Constant_Value_g;
  if (rtb_Limiterxi > FlyByWire_P.Saturation_UpperSat_e) {
    FlyByWire_Y.out.sim.data.gear_strut_compression_0 = FlyByWire_P.Saturation_UpperSat_e;
  } else if (rtb_Limiterxi < FlyByWire_P.Saturation_LowerSat_e) {
    FlyByWire_Y.out.sim.data.gear_strut_compression_0 = FlyByWire_P.Saturation_LowerSat_e;
  } else {
    FlyByWire_Y.out.sim.data.gear_strut_compression_0 = rtb_Limiterxi;
  }

  FlyByWire_Y.out.sim.data.gear_strut_compression_2 = result_tmp;
  FlyByWire_Y.out.sim.data.flaps_handle_index = FlyByWire_U.in.data.flaps_handle_index;
  FlyByWire_Y.out.sim.data.autopilot_master_on = FlyByWire_U.in.data.autopilot_master_on;
  FlyByWire_Y.out.sim.data_computed.on_ground = rtb_on_ground;
  FlyByWire_Y.out.sim.input.delta_eta_pos = rtb_Gaineta;
  FlyByWire_Y.out.sim.input.delta_xi_pos = rtb_Gainxi;
  FlyByWire_Y.out.sim.input.delta_zeta_pos = BusAssignment_sim_input_delta_zeta_pos;
  FlyByWire_Y.out.pitch.data_computed.in_flight = rtb_in_flight_o;
  FlyByWire_Y.out.pitch.data_computed.in_flare = rtb_in_flare;
  FlyByWire_Y.out.pitch.data_computed.in_flight_gain = rtb_BusAssignment_c_pitch_data_computed_in_flight_gain;
  FlyByWire_Y.out.pitch.data_computed.nz_limit_up_g = rtb_nz_limit_up_g;
  FlyByWire_Y.out.pitch.data_computed.nz_limit_lo_g = rtb_nz_limit_lo_g;
  FlyByWire_Y.out.pitch.data_computed.eta_trim_deg_should_freeze = rtb_eta_trim_deg_should_freeze;
  FlyByWire_Y.out.pitch.data_computed.eta_trim_deg_reset = rtb_eta_trim_deg_reset;
  FlyByWire_Y.out.pitch.data_computed.eta_trim_deg_reset_deg = rtb_eta_trim_deg_reset_deg;
  FlyByWire_Y.out.pitch.data_computed.eta_trim_deg_should_write = rtb_eta_trim_deg_should_write;
  FlyByWire_Y.out.pitch.data_computed.eta_trim_deg_rate_limit_up_deg_s = rtb_eta_trim_deg_rate_limit_up_deg_s;
  FlyByWire_Y.out.pitch.data_computed.eta_trim_deg_rate_limit_lo_deg_s = rtb_eta_trim_deg_rate_limit_lo_deg_s;
  FlyByWire_Y.out.pitch.data_computed.flare_Theta_deg = numAccum;
  FlyByWire_Y.out.pitch.data_computed.flare_Theta_c_deg = FlyByWire_DWork.DelayInput2_DSTATE;
  FlyByWire_Y.out.pitch.data_computed.flare_Theta_c_rate_deg_s = FlyByWire_B.flare_Theta_c_rate_deg_s;
  FlyByWire_Y.out.pitch.law_protection.attitude_min.eta_dot_pos_s = FlyByWire_P.Gain_Gain_c * rtb_UkYk1;
  FlyByWire_Y.out.pitch.law_protection.attitude_max.eta_dot_pos_s = FlyByWire_P.Gain1_Gain_e * rtb_u8;
  FlyByWire_Y.out.pitch.vote.eta_dot_pos_s = rtb_Integration_m;
  FlyByWire_Y.out.pitch.integrated.eta_pos = FlyByWire_DWork.Integration_DSTATE;
  FlyByWire_Y.out.pitch.output.eta_pos = rtb_LimiteriH;
  FlyByWire_Y.out.pitch.output.eta_trim_deg = FlyByWire_DWork.DelayInput2_DSTATE_h;
  FlyByWire_Y.out.roll.data_computed.in_flight = rtb_in_flight;
  FlyByWire_Y.out.roll.data_computed.in_flight_gain = rtb_DiscreteTimeIntegrator;
  FlyByWire_Y.out.roll.law_normal.pk_c_deg_s = rtb_Product;
  FlyByWire_Y.out.roll.law_normal.Phi_c_deg = FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE;
  FlyByWire_Y.out.roll.law_normal.xi_pos = rtb_BusAssignment_mk_roll_law_normal_xi_pos;
  FlyByWire_Y.out.roll.law_normal.zeta_pos = BusAssignment_sim_input_delta_zeta_pos;
  FlyByWire_Y.out.roll.output.xi_pos = rtb_BusAssignment_f_roll_output_xi_pos;
  FlyByWire_Y.out.roll.output.zeta_pos = BusAssignment_sim_input_delta_zeta_pos;
  if (rtb_eta_trim_deg_should_freeze) {
    rtb_GainPhi = FlyByWire_P.Constant_Value;
  } else {
    rtb_GainPhi = FlyByWire_P.Convertto_Gain * FlyByWire_DWork.Integration_DSTATE;
  }

  FlyByWire_DWork.DiscreteTransferFcn_states = (rtb_GainTheta - FlyByWire_P.DiscreteTransferFcn_DenCoef[1] *
    FlyByWire_DWork.DiscreteTransferFcn_states) / FlyByWire_P.DiscreteTransferFcn_DenCoef[0];
  FlyByWire_DWork.DiscreteTimeIntegrator_IC_LOADING = 0U;
  FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE += FlyByWire_P.DiscreteTimeIntegrator_gainval * rtb_Product;
  if (FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE >= FlyByWire_P.DiscreteTimeIntegrator_UpperSat) {
    FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE = FlyByWire_P.DiscreteTimeIntegrator_UpperSat;
  } else {
    if (FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE <= FlyByWire_P.DiscreteTimeIntegrator_LowerSat) {
      FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE = FlyByWire_P.DiscreteTimeIntegrator_LowerSat;
    }
  }

  if (rtb_DiscreteTimeIntegrator > 0.0) {
    FlyByWire_DWork.DiscreteTimeIntegrator_PrevResetState = 1;
  } else if (rtb_DiscreteTimeIntegrator < 0.0) {
    FlyByWire_DWork.DiscreteTimeIntegrator_PrevResetState = -1;
  } else if (rtb_DiscreteTimeIntegrator == 0.0) {
    FlyByWire_DWork.DiscreteTimeIntegrator_PrevResetState = 0;
  } else {
    FlyByWire_DWork.DiscreteTimeIntegrator_PrevResetState = 2;
  }

  FlyByWire_DWork.UD_DSTATE = rtb_TSamp;
  FlyByWire_DWork.Integration_IC_LOADING = 0U;
  FlyByWire_DWork.Integration_DSTATE += FlyByWire_P.Integration_gainval * rtb_Integration_m;
  if (FlyByWire_DWork.Integration_DSTATE >= FlyByWire_P.Integration_UpperSat) {
    FlyByWire_DWork.Integration_DSTATE = FlyByWire_P.Integration_UpperSat;
  } else {
    if (FlyByWire_DWork.Integration_DSTATE <= FlyByWire_P.Integration_LowerSat) {
      FlyByWire_DWork.Integration_DSTATE = FlyByWire_P.Integration_LowerSat;
    }
  }

  if (rtb_in_flight_o > 0) {
    FlyByWire_DWork.Integration_PrevResetState = 1;
  } else {
    FlyByWire_DWork.Integration_PrevResetState = 0;
  }

  FlyByWire_DWork.Integration_IC_LOADING_a = 0U;
  FlyByWire_DWork.Integration_DSTATE_e += FlyByWire_P.Gain_Gain_ip * rtb_GainPhi * FlyByWire_P.Integration_gainval_c;
  if (FlyByWire_DWork.Integration_DSTATE_e >= FlyByWire_P.Integration_UpperSat_f) {
    FlyByWire_DWork.Integration_DSTATE_e = FlyByWire_P.Integration_UpperSat_f;
  } else {
    if (FlyByWire_DWork.Integration_DSTATE_e <= FlyByWire_P.Integration_LowerSat_f) {
      FlyByWire_DWork.Integration_DSTATE_e = FlyByWire_P.Integration_LowerSat_f;
    }
  }

  FlyByWire_DWork.Integration_PrevResetState_p = static_cast<int8_T>(rtb_eta_trim_deg_reset);
}

void FlyByWireModelClass::initialize()
{
  rt_InitInfAndNaN(sizeof(real_T));
  (void) std::memset((static_cast<void *>(&FlyByWire_B)), 0,
                     sizeof(BlockIO_FlyByWire_T));
  (void) std::memset(static_cast<void *>(&FlyByWire_DWork), 0,
                     sizeof(D_Work_FlyByWire_T));
  FlyByWire_U.in = FlyByWire_rtZfbw_input;
  FlyByWire_Y.out = FlyByWire_rtZfbw_output;
  FlyByWire_DWork.PrevY = FlyByWire_P.RateLimitereta_IC;
  FlyByWire_DWork.PrevY_c = FlyByWire_P.RateLimiterxi_IC;
  FlyByWire_DWork.PrevY_f = FlyByWire_P.RateLimiterxi1_IC;
  FlyByWire_DWork.DiscreteTransferFcn_states = FlyByWire_P.DiscreteTransferFcn_InitialStates;
  FlyByWire_DWork.PrevY_j = FlyByWire_P.RateLimit_IC;
  FlyByWire_DWork.DiscreteTimeIntegrator_IC_LOADING = 1U;
  FlyByWire_DWork.DiscreteTimeIntegrator_PrevResetState = 2;
  FlyByWire_DWork.PrevY_h = FlyByWire_P.RateLimit_IC_d;
  FlyByWire_DWork.DelayInput2_DSTATE = FlyByWire_P.DelayInput2_InitialCondition;
  FlyByWire_DWork.UD_DSTATE = FlyByWire_P.PID_DifferentiatorICPrevScaledInput;
  FlyByWire_DWork.PrevY_i = FlyByWire_P.Limiter_IC;
  FlyByWire_DWork.Integration_IC_LOADING = 1U;
  FlyByWire_DWork.Integration_PrevResetState = 2;
  FlyByWire_DWork.DelayInput2_DSTATE_h = FlyByWire_P.DelayInput2_InitialCondition_d;
  FlyByWire_DWork.Integration_IC_LOADING_a = 1U;
  FlyByWire_DWork.Integration_PrevResetState_p = 0;
  FlyByWire_DWork.PrevY_d = FlyByWire_P.RateLimitereta_IC_c;
  FlyByWire_DWork.PrevY_jg = FlyByWire_P.RateLimiterxi_IC_c;
  FlyByWire_DWork.PrevY_g = FlyByWire_P.RateLimiterxi1_IC_i;
  FlyByWire_DWork.is_active_c1_FlyByWire = 0U;
  FlyByWire_DWork.is_c1_FlyByWire = FlyByWire_IN_NO_ACTIVE_CHILD;
  FlyByWire_DWork.is_active_c3_FlyByWire = 0U;
  FlyByWire_DWork.is_c3_FlyByWire = FlyByWire_IN_NO_ACTIVE_CHILD;
  FlyByWire_DWork.chartAbsoluteTimeCounter = 0;
  FlyByWire_DWork.is_active_c2_FlyByWire = 0U;
  FlyByWire_DWork.is_c2_FlyByWire = FlyByWire_IN_NO_ACTIVE_CHILD;
  FlyByWire_DWork.is_active_c7_FlyByWire = 0U;
  FlyByWire_DWork.is_c7_FlyByWire = FlyByWire_IN_NO_ACTIVE_CHILD;
  FlyByWire_DWork.is_active_c9_FlyByWire = 0U;
  FlyByWire_DWork.is_c9_FlyByWire = FlyByWire_IN_NO_ACTIVE_CHILD;
  FlyByWire_DWork.is_active_c8_FlyByWire = 0U;
  FlyByWire_DWork.is_c8_FlyByWire = FlyByWire_IN_NO_ACTIVE_CHILD;
  FlyByWire_DWork.is_active_c5_FlyByWire = 0U;
  FlyByWire_DWork.is_c5_FlyByWire = FlyByWire_IN_NO_ACTIVE_CHILD;
}

void FlyByWireModelClass::terminate()
{
}

FlyByWireModelClass::FlyByWireModelClass()
{
}

FlyByWireModelClass::~FlyByWireModelClass()
{
}
