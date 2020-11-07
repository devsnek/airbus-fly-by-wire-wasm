#include "FlyByWire.h"
#include "FlyByWire_private.h"

const uint8_T FlyByWire_IN_InAir = 1U;
const uint8_T FlyByWire_IN_NO_ACTIVE_CHILD = 0U;
const uint8_T FlyByWire_IN_OnGround = 2U;
const uint8_T FlyByWire_IN_frozen = 1U;
const uint8_T FlyByWire_IN_running = 2U;
const uint8_T FlyByWire_IN_Flare = 1U;
const uint8_T FlyByWire_IN_Flight_High = 2U;
const uint8_T FlyByWire_IN_Flight_Low = 3U;
const uint8_T FlyByWire_IN_Ground = 4U;
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
      0.0
    },

    {
      0.0
    }
  }
} ;

const fbw_input FlyByWire_rtZfbw_input = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0 }, { 0.0, 0.0, 0.0 } };

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
  real_T rateLimiterRate;
  real_T result[3];
  boolean_T condIsTrue;
  real_T rtb_Filter;
  real_T rtb_Integration_m;
  real_T rtb_Merge;
  real_T rtb_LimiteriH;
  real_T rtb_BusAssignment_l_pitch_data_computed_in_flight_gain;
  real_T rtb_GainTheta;
  real_T rtb_GainPhi;
  real_T rtb_Limiterxi1;
  real_T rtb_Gaineta;
  real_T rtb_Gainxi;
  int32_T rtb_onGround;
  real_T rtb_BusAssignment_a_sim_input_delta_zeta_pos;
  int32_T rtb_in_flight;
  int32_T rtb_in_flare;
  int32_T rtb_iH_deg_should_freeze;
  int32_T rtb_iH_deg_reset;
  real_T rtb_iH_deg_reset_deg;
  int32_T rtb_iH_deg_should_write;
  real_T rtb_iH_deg_rate_limit_up_deg_s;
  real_T rtb_iH_deg_rate_limit_lo_deg_s;
  real_T rtb_BusAssignment_f_pitch_law_protection_attitude_min_eta_dot_pos_s;
  real_T rtb_BusAssignment_f_pitch_law_protection_attitude_max_eta_dot_pos_s;
  real_T rtb_BusAssignment_f_roll_law_normal_xi_pos;
  real_T rtb_FilterCoefficient;
  real_T rtb_BusAssignment_mv_pitch_output_eta_pos;
  real_T rtb_Saturation[3];
  real_T rtb_UkYk1;
  int32_T rtb_inFlight;
  real_T rtb_deltafalllimit;
  real_T result_tmp;
  real_T tmp[9];
  boolean_T tmp_0;
  boolean_T tmp_1;
  boolean_T tmp_2;
  boolean_T tmp_3;
  int32_T exitg1;
  rtb_GainTheta = FlyByWire_P.GainTheta_Gain * FlyByWire_U.in.data.Theta_deg;
  rtb_GainPhi = FlyByWire_P.GainPhi_Gain * FlyByWire_U.in.data.Phi_deg;
  rtb_Limiterxi1 = FlyByWire_P.Gain_Gain * FlyByWire_U.in.data.qk_rad_s * FlyByWire_P.Gainqk_Gain;
  rtb_LimiteriH = FlyByWire_P.Gain_Gain_a * FlyByWire_U.in.data.pk_rad_s * FlyByWire_P.Gainpk_Gain;
  rtb_Integration_m = 0.017453292519943295 * rtb_GainTheta;
  rtb_Gaineta = 0.017453292519943295 * rtb_GainPhi;
  result_tmp = std::sin(rtb_Gaineta);
  rtb_Gaineta = std::cos(rtb_Gaineta);
  rtb_Gainxi = std::tan(rtb_Integration_m);
  tmp[0] = 0.0;
  tmp[3] = rtb_Gaineta;
  tmp[6] = -result_tmp;
  tmp[1] = 0.0;
  rtb_Integration_m = 1.0 / std::cos(rtb_Integration_m);
  tmp[4] = rtb_Integration_m * result_tmp;
  tmp[7] = rtb_Integration_m * rtb_Gaineta;
  tmp[2] = 1.0;
  tmp[5] = result_tmp * rtb_Gainxi;
  tmp[8] = rtb_Gaineta * rtb_Gainxi;
  result_tmp = FlyByWire_P.Gain_Gain_e * FlyByWire_U.in.data.q_dot_rad_s2 * FlyByWire_P.Gainqk1_Gain;
  rtb_Integration_m = FlyByWire_P.Gain_Gain_aw * FlyByWire_U.in.data.r_dot_rad_s2 * FlyByWire_P.Gainrk1_Gain;
  rtb_Gaineta = FlyByWire_P.Gain_Gain_n * FlyByWire_U.in.data.p_dot_rad_s2 * FlyByWire_P.Gainpk1_Gain;
  for (rtb_inFlight = 0; rtb_inFlight < 3; rtb_inFlight++) {
    result[rtb_inFlight] = tmp[rtb_inFlight + 6] * rtb_Gaineta + (tmp[rtb_inFlight + 3] * rtb_Integration_m +
      tmp[rtb_inFlight] * result_tmp);
  }

  rtb_Merge = FlyByWire_P.Gainpk2_Gain * FlyByWire_U.in.data.iH_deg;
  rtb_deltafalllimit = FlyByWire_P.Gain1_Gain * FlyByWire_U.in.data.gear_animation_pos_1 - FlyByWire_P.Constant_Value_g;
  if (rtb_deltafalllimit > FlyByWire_P.Saturation1_UpperSat) {
    rtb_deltafalllimit = FlyByWire_P.Saturation1_UpperSat;
  } else {
    if (rtb_deltafalllimit < FlyByWire_P.Saturation1_LowerSat) {
      rtb_deltafalllimit = FlyByWire_P.Saturation1_LowerSat;
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

  rateLimiterRate = FlyByWire_U.in.input.delta_eta_pos - FlyByWire_DWork.PrevY;
  if (rateLimiterRate > FlyByWire_P.RateLimitereta_RisingLim) {
    rtb_Integration_m = FlyByWire_DWork.PrevY + FlyByWire_P.RateLimitereta_RisingLim;
  } else if (rateLimiterRate < FlyByWire_P.RateLimitereta_FallingLim) {
    rtb_Integration_m = FlyByWire_DWork.PrevY + FlyByWire_P.RateLimitereta_FallingLim;
  } else {
    rtb_Integration_m = FlyByWire_U.in.input.delta_eta_pos;
  }

  FlyByWire_DWork.PrevY = rtb_Integration_m;
  rtb_Gaineta = FlyByWire_P.Gaineta_Gain * rtb_Integration_m;
  rateLimiterRate = FlyByWire_U.in.input.delta_xi_pos - FlyByWire_DWork.PrevY_c;
  if (rateLimiterRate > FlyByWire_P.RateLimiterxi_RisingLim) {
    rtb_Integration_m = FlyByWire_DWork.PrevY_c + FlyByWire_P.RateLimiterxi_RisingLim;
  } else if (rateLimiterRate < FlyByWire_P.RateLimiterxi_FallingLim) {
    rtb_Integration_m = FlyByWire_DWork.PrevY_c + FlyByWire_P.RateLimiterxi_FallingLim;
  } else {
    rtb_Integration_m = FlyByWire_U.in.input.delta_xi_pos;
  }

  FlyByWire_DWork.PrevY_c = rtb_Integration_m;
  rtb_Gainxi = FlyByWire_P.Gainxi_Gain * rtb_Integration_m;
  rateLimiterRate = FlyByWire_U.in.input.delta_zeta_pos - FlyByWire_DWork.PrevY_f;
  if (rateLimiterRate > FlyByWire_P.RateLimiterxi1_RisingLim) {
    rtb_Integration_m = FlyByWire_DWork.PrevY_f + FlyByWire_P.RateLimiterxi1_RisingLim;
  } else if (rateLimiterRate < FlyByWire_P.RateLimiterxi1_FallingLim) {
    rtb_Integration_m = FlyByWire_DWork.PrevY_f + FlyByWire_P.RateLimiterxi1_FallingLim;
  } else {
    rtb_Integration_m = FlyByWire_U.in.input.delta_zeta_pos;
  }

  FlyByWire_DWork.PrevY_f = rtb_Integration_m;
  if (FlyByWire_DWork.is_active_c1_FlyByWire == 0U) {
    FlyByWire_DWork.is_active_c1_FlyByWire = 1U;
    FlyByWire_DWork.is_c1_FlyByWire = FlyByWire_IN_OnGround;
    rtb_onGround = 1;
  } else if (FlyByWire_DWork.is_c1_FlyByWire == FlyByWire_IN_InAir) {
    if ((rtb_deltafalllimit > 0.1) || (result_tmp > 0.1)) {
      FlyByWire_DWork.is_c1_FlyByWire = FlyByWire_IN_OnGround;
      rtb_onGround = 1;
    } else {
      rtb_onGround = 0;
    }
  } else {
    if ((rtb_deltafalllimit == 0.0) && (result_tmp == 0.0)) {
      FlyByWire_DWork.is_c1_FlyByWire = FlyByWire_IN_InAir;
      rtb_onGround = 0;
    } else {
      rtb_onGround = 1;
    }
  }

  FlyByWire_Y.out.sim.data.pk_deg_s = rtb_LimiteriH;
  FlyByWire_Y.out.sim.data.iH_deg = rtb_Merge;
  FlyByWire_Y.out.sim.data.gear_strut_compression_1 = rtb_deltafalllimit;
  rtb_BusAssignment_a_sim_input_delta_zeta_pos = rtb_Integration_m;
  FlyByWire_DWork.chartAbsoluteTimeCounter++;
  condIsTrue = ((rtb_onGround == 1) && (rtb_GainTheta < 2.5));
  if ((!condIsTrue) || (!FlyByWire_DWork.condWasTrueAtLastTimeStep_1)) {
    FlyByWire_DWork.durationLastReferenceTick_1 = FlyByWire_DWork.chartAbsoluteTimeCounter;
  }

  FlyByWire_DWork.condWasTrueAtLastTimeStep_1 = condIsTrue;
  if (FlyByWire_DWork.is_active_c3_FlyByWire == 0U) {
    FlyByWire_DWork.chartAbsoluteTimeCounter = 0;
    FlyByWire_DWork.is_active_c3_FlyByWire = 1U;
    FlyByWire_DWork.is_c3_FlyByWire = FlyByWire_IN_Ground;
    rtb_in_flight = 0;
    rtb_in_flare = 0;
  } else {
    switch (FlyByWire_DWork.is_c3_FlyByWire) {
     case FlyByWire_IN_Flare:
      condIsTrue = ((rtb_onGround == 1) && (rtb_GainTheta < 2.5));
      if ((!condIsTrue) || (!FlyByWire_DWork.condWasTrueAtLastTimeStep_1)) {
        FlyByWire_DWork.durationLastReferenceTick_1 = FlyByWire_DWork.chartAbsoluteTimeCounter;
      }

      FlyByWire_DWork.condWasTrueAtLastTimeStep_1 = condIsTrue;
      if (FlyByWire_DWork.chartAbsoluteTimeCounter - FlyByWire_DWork.durationLastReferenceTick_1 > 250) {
        FlyByWire_DWork.is_c3_FlyByWire = FlyByWire_IN_Ground;
        rtb_in_flight = 0;
        rtb_in_flare = 0;
      } else if ((FlyByWire_U.in.data.radio_height_ft > 50.0) && (rtb_GainTheta > 8.0)) {
        FlyByWire_DWork.is_c3_FlyByWire = FlyByWire_IN_Flight_Low;
        rtb_in_flight = 1;
        rtb_in_flare = 0;
      } else {
        rtb_in_flight = 1;
        rtb_in_flare = 1;
      }
      break;

     case FlyByWire_IN_Flight_High:
      if (FlyByWire_U.in.data.radio_height_ft <= 50.0) {
        FlyByWire_DWork.durationLastReferenceTick_1 = FlyByWire_DWork.chartAbsoluteTimeCounter;
        FlyByWire_DWork.is_c3_FlyByWire = FlyByWire_IN_Flare;
        rtb_in_flight = 1;
        rtb_in_flare = 1;
        FlyByWire_DWork.condWasTrueAtLastTimeStep_1 = ((rtb_onGround == 1) && (rtb_GainTheta < 2.5));
      } else {
        rtb_in_flight = 1;
        rtb_in_flare = 0;
      }
      break;

     case FlyByWire_IN_Flight_Low:
      if (FlyByWire_U.in.data.radio_height_ft > 50.0) {
        FlyByWire_DWork.is_c3_FlyByWire = FlyByWire_IN_Flight_High;
        rtb_in_flight = 1;
        rtb_in_flare = 0;
      } else {
        rtb_in_flight = 1;
        rtb_in_flare = 0;
      }
      break;

     default:
      if (((rtb_onGround == 0) && (rtb_GainTheta > 8.0)) || (FlyByWire_U.in.data.radio_height_ft > 400.0)) {
        FlyByWire_DWork.is_c3_FlyByWire = FlyByWire_IN_Flight_Low;
        rtb_in_flight = 1;
        rtb_in_flare = 0;
      } else {
        rtb_in_flight = 0;
        rtb_in_flare = 0;
      }
      break;
    }
  }

  if (rtb_in_flight > FlyByWire_P.Saturation_UpperSat_e) {
    rtb_Integration_m = FlyByWire_P.Saturation_UpperSat_e;
  } else if (rtb_in_flight < FlyByWire_P.Saturation_LowerSat_a) {
    rtb_Integration_m = FlyByWire_P.Saturation_LowerSat_a;
  } else {
    rtb_Integration_m = rtb_in_flight;
  }

  rateLimiterRate = rtb_Integration_m - FlyByWire_DWork.PrevY_h;
  if (rateLimiterRate > FlyByWire_P.RateLimit_RisingLim) {
    rtb_Integration_m = FlyByWire_DWork.PrevY_h + FlyByWire_P.RateLimit_RisingLim;
  } else {
    if (rateLimiterRate < FlyByWire_P.RateLimit_FallingLim) {
      rtb_Integration_m = FlyByWire_DWork.PrevY_h + FlyByWire_P.RateLimit_FallingLim;
    }
  }

  FlyByWire_DWork.PrevY_h = rtb_Integration_m;
  if (FlyByWire_DWork.is_active_c9_FlyByWire == 0U) {
    FlyByWire_DWork.is_active_c9_FlyByWire = 1U;
    FlyByWire_DWork.is_c9_FlyByWire = FlyByWire_IN_running;
    rtb_iH_deg_should_freeze = 0;
  } else if (FlyByWire_DWork.is_c9_FlyByWire == FlyByWire_IN_frozen) {
    if ((rtb_in_flare == 0) && (FlyByWire_U.in.data.nz_g < 1.25) && (FlyByWire_U.in.data.nz_g > 0.5) && (rtb_GainPhi <
         30.0) && (rtb_GainPhi > -30.0)) {
      FlyByWire_DWork.is_c9_FlyByWire = FlyByWire_IN_running;
      rtb_iH_deg_should_freeze = 0;
    } else {
      rtb_iH_deg_should_freeze = 1;
    }
  } else {
    if ((rtb_in_flare != 0) || (FlyByWire_U.in.data.nz_g >= 1.25) || (FlyByWire_U.in.data.nz_g <= 0.5) || (rtb_GainPhi >
         30.0) || (rtb_GainPhi < -30.0)) {
      FlyByWire_DWork.is_c9_FlyByWire = FlyByWire_IN_frozen;
      rtb_iH_deg_should_freeze = 1;
    } else {
      rtb_iH_deg_should_freeze = 0;
    }
  }

  if (FlyByWire_DWork.is_active_c8_FlyByWire == 0U) {
    FlyByWire_DWork.is_active_c8_FlyByWire = 1U;
    FlyByWire_DWork.is_c8_FlyByWire = FlyByWire_IN_manual;
    rtb_iH_deg_reset = 1;
    rtb_iH_deg_reset_deg = rtb_Merge;
    rtb_iH_deg_should_write = 0;
  } else {
    switch (FlyByWire_DWork.is_c8_FlyByWire) {
     case FlyByWire_IN_automatic:
      if (rtb_in_flight == 0) {
        FlyByWire_DWork.is_c8_FlyByWire = FlyByWire_IN_reset;
        rtb_iH_deg_reset = 1;
        rtb_iH_deg_reset_deg = 0.0;
        rtb_iH_deg_should_write = 1;
      } else {
        rtb_iH_deg_reset = 0;
        rtb_iH_deg_reset_deg = rtb_Merge;
        rtb_iH_deg_should_write = 1;
      }
      break;

     case FlyByWire_IN_manual:
      if (rtb_in_flight != 0) {
        FlyByWire_DWork.is_c8_FlyByWire = FlyByWire_IN_automatic;
        rtb_iH_deg_reset = 0;
        rtb_iH_deg_reset_deg = rtb_Merge;
        rtb_iH_deg_should_write = 1;
      } else {
        rtb_iH_deg_reset = 1;
        rtb_iH_deg_reset_deg = rtb_Merge;
        rtb_iH_deg_should_write = 0;
      }
      break;

     default:
      if ((rtb_in_flight == 0) && (rtb_Merge == 0.0)) {
        FlyByWire_DWork.is_c8_FlyByWire = FlyByWire_IN_manual;
        rtb_iH_deg_reset = 1;
        rtb_iH_deg_reset_deg = rtb_Merge;
        rtb_iH_deg_should_write = 0;
      } else {
        rtb_iH_deg_reset = 1;
        rtb_iH_deg_reset_deg = 0.0;
        rtb_iH_deg_should_write = 1;
      }
      break;
    }
  }

  if (FlyByWire_DWork.is_active_c7_FlyByWire == 0U) {
    FlyByWire_DWork.is_active_c7_FlyByWire = 1U;
    FlyByWire_DWork.is_c7_FlyByWire = FlyByWire_IN_ground;
    rtb_iH_deg_rate_limit_up_deg_s = 0.7;
    rtb_iH_deg_rate_limit_lo_deg_s = -0.7;
  } else {
    switch (FlyByWire_DWork.is_c7_FlyByWire) {
     case FlyByWire_IN_flight_clean:
      if (FlyByWire_U.in.data.flaps_handle_index != 0.0) {
        FlyByWire_DWork.is_c7_FlyByWire = FlyByWire_IN_flight_flaps;
        rtb_iH_deg_rate_limit_up_deg_s = 0.7;
        rtb_iH_deg_rate_limit_lo_deg_s = -0.7;
      } else if ((rtb_in_flight == 0) && (!(FlyByWire_U.in.data.flaps_handle_index != 0.0))) {
        FlyByWire_DWork.is_c7_FlyByWire = FlyByWire_IN_ground;
        rtb_iH_deg_rate_limit_up_deg_s = 0.7;
        rtb_iH_deg_rate_limit_lo_deg_s = -0.7;
      } else {
        rtb_iH_deg_rate_limit_up_deg_s = 0.3;
        rtb_iH_deg_rate_limit_lo_deg_s = -0.3;
      }
      break;

     case FlyByWire_IN_flight_flaps:
      if (!(FlyByWire_U.in.data.flaps_handle_index != 0.0)) {
        FlyByWire_DWork.is_c7_FlyByWire = FlyByWire_IN_flight_clean;
        rtb_iH_deg_rate_limit_up_deg_s = 0.3;
        rtb_iH_deg_rate_limit_lo_deg_s = -0.3;
      } else if (rtb_in_flight == 0) {
        FlyByWire_DWork.is_c7_FlyByWire = FlyByWire_IN_ground;
        rtb_iH_deg_rate_limit_up_deg_s = 0.7;
        rtb_iH_deg_rate_limit_lo_deg_s = -0.7;
      } else {
        rtb_iH_deg_rate_limit_up_deg_s = 0.7;
        rtb_iH_deg_rate_limit_lo_deg_s = -0.7;
      }
      break;

     default:
      if ((rtb_in_flight != 0) && (!(FlyByWire_U.in.data.flaps_handle_index != 0.0))) {
        FlyByWire_DWork.is_c7_FlyByWire = FlyByWire_IN_flight_clean;
        rtb_iH_deg_rate_limit_up_deg_s = 0.3;
        rtb_iH_deg_rate_limit_lo_deg_s = -0.3;
      } else if ((rtb_in_flight != 0) && (FlyByWire_U.in.data.flaps_handle_index != 0.0)) {
        FlyByWire_DWork.is_c7_FlyByWire = FlyByWire_IN_flight_flaps;
        rtb_iH_deg_rate_limit_up_deg_s = 0.7;
        rtb_iH_deg_rate_limit_lo_deg_s = -0.7;
      } else {
        rtb_iH_deg_rate_limit_up_deg_s = 0.7;
        rtb_iH_deg_rate_limit_lo_deg_s = -0.7;
      }
      break;
    }
  }

  rtb_BusAssignment_l_pitch_data_computed_in_flight_gain = rtb_Integration_m;
  rtb_Integration_m = FlyByWire_P.K7857_Gain * result[0] + FlyByWire_P.K7958_Gain * rtb_Limiterxi1;
  rtb_deltafalllimit = look1_binlxpw(FlyByWire_U.in.data.Vk_kt, FlyByWire_P.F193LUT_bp01Data,
    FlyByWire_P.F193LUT_tableData, 7U);
  rtb_BusAssignment_f_pitch_law_protection_attitude_min_eta_dot_pos_s = ((FlyByWire_P.u1_Value + rtb_GainTheta) *
    FlyByWire_P.K7659_Gain + rtb_Integration_m) * rtb_deltafalllimit;
  rtb_BusAssignment_f_pitch_law_protection_attitude_max_eta_dot_pos_s = ((rtb_GainTheta - FlyByWire_P.u7_Value) *
    FlyByWire_P.K7765_Gain + rtb_Integration_m) * rtb_deltafalllimit;
  rtb_Integration_m = FlyByWire_P.Gain1_Gain_j * rtb_Limiterxi1 * FlyByWire_P.Constant2_Value + FlyByWire_U.in.data.nz_g;
  rateLimiterRate = rtb_Gaineta - FlyByWire_DWork.PrevY_j;
  if (rateLimiterRate > FlyByWire_P.RateLimiter_RisingLim) {
    rtb_deltafalllimit = FlyByWire_DWork.PrevY_j + FlyByWire_P.RateLimiter_RisingLim;
  } else if (rateLimiterRate < FlyByWire_P.RateLimiter_FallingLim) {
    rtb_deltafalllimit = FlyByWire_DWork.PrevY_j + FlyByWire_P.RateLimiter_FallingLim;
  } else {
    rtb_deltafalllimit = rtb_Gaineta;
  }

  FlyByWire_DWork.PrevY_j = rtb_deltafalllimit;
  rtb_deltafalllimit = look1_binlxpw(rtb_deltafalllimit, FlyByWire_P.loaddemand_bp01Data,
    FlyByWire_P.loaddemand_tableData, 2U);
  rtb_deltafalllimit += std::cos(FlyByWire_P.Gain1_Gain_p * rtb_GainTheta) / std::cos(FlyByWire_P.Gain1_Gain_b *
    rtb_GainPhi);
  if (rtb_deltafalllimit > FlyByWire_P.Saturation_UpperSat_ec) {
    rtb_deltafalllimit = FlyByWire_P.Saturation_UpperSat_ec;
  } else {
    if (rtb_deltafalllimit < FlyByWire_P.Saturation_LowerSat_n) {
      rtb_deltafalllimit = FlyByWire_P.Saturation_LowerSat_n;
    }
  }

  rtb_Filter = rtb_Integration_m - rtb_deltafalllimit;
  rtb_FilterCoefficient = (FlyByWire_P.PID_D * rtb_Filter - FlyByWire_DWork.Filter_DSTATE) * FlyByWire_P.PID_N;
  rtb_Merge = FlyByWire_P.PID_P * rtb_Filter + rtb_FilterCoefficient;
  if (rtb_Merge > FlyByWire_P.PID_UpperSaturationLimit) {
    rtb_Merge = FlyByWire_P.PID_UpperSaturationLimit;
  } else {
    if (rtb_Merge < FlyByWire_P.PID_LowerSaturationLimit) {
      rtb_Merge = FlyByWire_P.PID_LowerSaturationLimit;
    }
  }

  rateLimiterRate = rtb_Merge - FlyByWire_DWork.PrevY_i[0];
  if (rateLimiterRate > FlyByWire_P.Limiter_RisingLim) {
    FlyByWire_DWork.PrevY_i[0] += FlyByWire_P.Limiter_RisingLim;
  } else if (rateLimiterRate < FlyByWire_P.Limiter_FallingLim) {
    FlyByWire_DWork.PrevY_i[0] += FlyByWire_P.Limiter_FallingLim;
  } else {
    FlyByWire_DWork.PrevY_i[0] = rtb_Merge;
  }

  if (FlyByWire_DWork.PrevY_i[0] > FlyByWire_P.Saturation_UpperSat_h) {
    rtb_Saturation[0] = FlyByWire_P.Saturation_UpperSat_h;
  } else if (FlyByWire_DWork.PrevY_i[0] < FlyByWire_P.Saturation_LowerSat_l) {
    rtb_Saturation[0] = FlyByWire_P.Saturation_LowerSat_l;
  } else {
    rtb_Saturation[0] = FlyByWire_DWork.PrevY_i[0];
  }

  rateLimiterRate = rtb_BusAssignment_f_pitch_law_protection_attitude_min_eta_dot_pos_s - FlyByWire_DWork.PrevY_i[1];
  if (rateLimiterRate > FlyByWire_P.Limiter_RisingLim) {
    FlyByWire_DWork.PrevY_i[1] += FlyByWire_P.Limiter_RisingLim;
  } else if (rateLimiterRate < FlyByWire_P.Limiter_FallingLim) {
    FlyByWire_DWork.PrevY_i[1] += FlyByWire_P.Limiter_FallingLim;
  } else {
    FlyByWire_DWork.PrevY_i[1] = rtb_BusAssignment_f_pitch_law_protection_attitude_min_eta_dot_pos_s;
  }

  if (FlyByWire_DWork.PrevY_i[1] > FlyByWire_P.Saturation_UpperSat_h) {
    rtb_Saturation[1] = FlyByWire_P.Saturation_UpperSat_h;
  } else if (FlyByWire_DWork.PrevY_i[1] < FlyByWire_P.Saturation_LowerSat_l) {
    rtb_Saturation[1] = FlyByWire_P.Saturation_LowerSat_l;
  } else {
    rtb_Saturation[1] = FlyByWire_DWork.PrevY_i[1];
  }

  rateLimiterRate = rtb_BusAssignment_f_pitch_law_protection_attitude_max_eta_dot_pos_s - FlyByWire_DWork.PrevY_i[2];
  if (rateLimiterRate > FlyByWire_P.Limiter_RisingLim) {
    FlyByWire_DWork.PrevY_i[2] += FlyByWire_P.Limiter_RisingLim;
  } else if (rateLimiterRate < FlyByWire_P.Limiter_FallingLim) {
    FlyByWire_DWork.PrevY_i[2] += FlyByWire_P.Limiter_FallingLim;
  } else {
    FlyByWire_DWork.PrevY_i[2] = rtb_BusAssignment_f_pitch_law_protection_attitude_max_eta_dot_pos_s;
  }

  if (FlyByWire_DWork.PrevY_i[2] > FlyByWire_P.Saturation_UpperSat_h) {
    rtb_Filter = FlyByWire_P.Saturation_UpperSat_h;
  } else if (FlyByWire_DWork.PrevY_i[2] < FlyByWire_P.Saturation_LowerSat_l) {
    rtb_Filter = FlyByWire_P.Saturation_LowerSat_l;
  } else {
    rtb_Filter = FlyByWire_DWork.PrevY_i[2];
  }

  if (FlyByWire_DWork.PrevY_i[2] > FlyByWire_P.Saturation_UpperSat_h) {
    rtb_Saturation[2] = FlyByWire_P.Saturation_UpperSat_h;
  } else if (FlyByWire_DWork.PrevY_i[2] < FlyByWire_P.Saturation_LowerSat_l) {
    rtb_Saturation[2] = FlyByWire_P.Saturation_LowerSat_l;
  } else {
    rtb_Saturation[2] = FlyByWire_DWork.PrevY_i[2];
  }

  rtb_inFlight = 0;
  do {
    exitg1 = 0;
    if (rtb_inFlight < 3) {
      if (rtIsNaN(rtb_Saturation[rtb_inFlight])) {
        rtb_Filter = (rtNaN);
        exitg1 = 1;
      } else {
        rtb_inFlight++;
      }
    } else {
      if (rtIsNaN(rtb_Saturation[1])) {
        condIsTrue = !rtIsNaN(rtb_Saturation[0]);
      } else {
        condIsTrue = ((!rtIsNaN(rtb_Saturation[0])) && (rtb_Saturation[0] < rtb_Saturation[1]));
      }

      if (rtIsNaN(rtb_Filter)) {
        tmp_3 = !rtIsNaN(rtb_Saturation[1]);
        tmp_1 = !rtIsNaN(rtb_Saturation[0]);
        tmp_0 = tmp_1;
        tmp_2 = tmp_3;
      } else {
        tmp_2 = !rtIsNaN(rtb_Saturation[1]);
        tmp_3 = (tmp_2 && (rtb_Saturation[1] < rtb_Filter));
        tmp_0 = !rtIsNaN(rtb_Saturation[0]);
        tmp_1 = (tmp_0 && (rtb_Saturation[0] < rtb_Filter));
        tmp_0 = (tmp_0 && (rtb_Saturation[0] < rtb_Filter));
        tmp_2 = (tmp_2 && (rtb_Saturation[1] < rtb_Filter));
      }

      if (condIsTrue) {
        if (tmp_3) {
          rtb_inFlight = 1;
        } else if (tmp_1) {
          rtb_inFlight = 2;
        } else {
          rtb_inFlight = 0;
        }
      } else if (tmp_0) {
        rtb_inFlight = 0;
      } else if (tmp_2) {
        rtb_inFlight = 2;
      } else {
        rtb_inFlight = 1;
      }

      rtb_Filter = rtb_Saturation[rtb_inFlight];
      exitg1 = 1;
    }
  } while (exitg1 == 0);

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

  if ((rtb_in_flight > 0) && (FlyByWire_DWork.Integration_PrevResetState <= 0)) {
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

  FlyByWire_Y.out.pitch.law_normal.Cstar_c_g = rtb_deltafalllimit;
  FlyByWire_Y.out.pitch.law_normal.eta_dot_pos_s = rtb_Merge;
  if (rtb_BusAssignment_l_pitch_data_computed_in_flight_gain > FlyByWire_P.Saturation_UpperSat_j) {
    rtb_deltafalllimit = FlyByWire_P.Saturation_UpperSat_j;
  } else if (rtb_BusAssignment_l_pitch_data_computed_in_flight_gain < FlyByWire_P.Saturation_LowerSat_i) {
    rtb_deltafalllimit = FlyByWire_P.Saturation_LowerSat_i;
  } else {
    rtb_deltafalllimit = rtb_BusAssignment_l_pitch_data_computed_in_flight_gain;
  }

  rtb_Merge = FlyByWire_DWork.Integration_DSTATE * rtb_deltafalllimit;
  rtb_deltafalllimit -= FlyByWire_P.Constant_Value_j;
  rtb_deltafalllimit *= rtb_Gaineta;
  rtb_BusAssignment_mv_pitch_output_eta_pos = rtb_Merge + rtb_deltafalllimit;
  if (FlyByWire_DWork.Integration_IC_LOADING_a != 0) {
    FlyByWire_DWork.Integration_DSTATE_e = rtb_iH_deg_reset_deg;
    if (FlyByWire_DWork.Integration_DSTATE_e >= FlyByWire_P.Integration_UpperSat_f) {
      FlyByWire_DWork.Integration_DSTATE_e = FlyByWire_P.Integration_UpperSat_f;
    } else {
      if (FlyByWire_DWork.Integration_DSTATE_e <= FlyByWire_P.Integration_LowerSat_f) {
        FlyByWire_DWork.Integration_DSTATE_e = FlyByWire_P.Integration_LowerSat_f;
      }
    }
  }

  if ((rtb_iH_deg_reset != 0) || (FlyByWire_DWork.Integration_PrevResetState_p != 0)) {
    FlyByWire_DWork.Integration_DSTATE_e = rtb_iH_deg_reset_deg;
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

  rtb_UkYk1 = FlyByWire_DWork.Integration_DSTATE_e - FlyByWire_DWork.DelayInput2_DSTATE;
  rtb_deltafalllimit = rtb_iH_deg_rate_limit_up_deg_s * FlyByWire_P.sampletime_WtEt;
  if (!(rtb_UkYk1 > rtb_deltafalllimit)) {
    rtb_deltafalllimit = rtb_iH_deg_rate_limit_lo_deg_s * FlyByWire_P.sampletime_WtEt;
    if (!(rtb_UkYk1 < rtb_deltafalllimit)) {
      rtb_deltafalllimit = rtb_UkYk1;
    }
  }

  FlyByWire_DWork.DelayInput2_DSTATE += rtb_deltafalllimit;
  if (FlyByWire_DWork.is_active_c5_FlyByWire == 0U) {
    FlyByWire_DWork.is_active_c5_FlyByWire = 1U;
    FlyByWire_DWork.is_c5_FlyByWire = FlyByWire_IN_GroundMode;
    rtb_inFlight = 0;
  } else if (FlyByWire_DWork.is_c5_FlyByWire == FlyByWire_IN_FlightMode) {
    if (rtb_onGround == 1) {
      FlyByWire_DWork.is_c5_FlyByWire = FlyByWire_IN_GroundMode;
      rtb_inFlight = 0;
    } else {
      rtb_inFlight = 1;
    }
  } else {
    if (((rtb_onGround == 0) && (rtb_GainTheta > 8.0)) || (FlyByWire_U.in.data.radio_height_ft > 400.0)) {
      FlyByWire_DWork.is_c5_FlyByWire = FlyByWire_IN_FlightMode;
      rtb_inFlight = 1;
    } else {
      rtb_inFlight = 0;
    }
  }

  if (rtb_inFlight > FlyByWire_P.Saturation_UpperSat_p) {
    rtb_deltafalllimit = FlyByWire_P.Saturation_UpperSat_p;
  } else if (rtb_inFlight < FlyByWire_P.Saturation_LowerSat_h) {
    rtb_deltafalllimit = FlyByWire_P.Saturation_LowerSat_h;
  } else {
    rtb_deltafalllimit = rtb_inFlight;
  }

  rateLimiterRate = rtb_deltafalllimit - FlyByWire_DWork.PrevY_jv;
  if (rateLimiterRate > FlyByWire_P.RateLimit_RisingLim_b) {
    rtb_deltafalllimit = FlyByWire_DWork.PrevY_jv + FlyByWire_P.RateLimit_RisingLim_b;
  } else {
    if (rateLimiterRate < FlyByWire_P.RateLimit_FallingLim_m) {
      rtb_deltafalllimit = FlyByWire_DWork.PrevY_jv + FlyByWire_P.RateLimit_FallingLim_m;
    }
  }

  FlyByWire_DWork.PrevY_jv = rtb_deltafalllimit;
  rtb_Merge = FlyByWire_P.Gain1_Gain_m * rtb_Gainxi + look1_binlxpw(rtb_GainPhi,
    FlyByWire_P.BankAngleProtection_bp01Data, FlyByWire_P.BankAngleProtection_tableData, 6U);
  if (rtb_Merge > FlyByWire_P.RateLimiter_UpperSat) {
    rtb_Merge = FlyByWire_P.RateLimiter_UpperSat;
  } else {
    if (rtb_Merge < FlyByWire_P.RateLimiter_LowerSat) {
      rtb_Merge = FlyByWire_P.RateLimiter_LowerSat;
    }
  }

  rtb_UkYk1 = rtb_Merge * static_cast<real_T>(rtb_inFlight);
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

  if ((rtb_inFlight > 0) && (FlyByWire_DWork.DiscreteTimeIntegrator_PrevResetState <= 0)) {
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

  rtb_BusAssignment_f_roll_law_normal_xi_pos = (FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE - rtb_GainPhi) *
    FlyByWire_P.Gain2_Gain_i + FlyByWire_P.Gain1_Gain_mg * rtb_LimiteriH * FlyByWire_P.pKp_Gain;
  if (rtb_deltafalllimit > FlyByWire_P.Saturation_UpperSat_n) {
    rtb_Merge = FlyByWire_P.Saturation_UpperSat_n;
  } else if (rtb_deltafalllimit < FlyByWire_P.Saturation_LowerSat_n2) {
    rtb_Merge = FlyByWire_P.Saturation_LowerSat_n2;
  } else {
    rtb_Merge = rtb_deltafalllimit;
  }

  rtb_LimiteriH = rtb_BusAssignment_f_roll_law_normal_xi_pos * rtb_Merge;
  rtb_Merge -= FlyByWire_P.Constant_Value_h;
  rtb_Merge *= rtb_Gainxi;
  rtb_LimiteriH += rtb_Merge;
  FlyByWire_Y.out.sim.data.qk_deg_s = rtb_Limiterxi1;
  rateLimiterRate = rtb_BusAssignment_mv_pitch_output_eta_pos - FlyByWire_DWork.PrevY_d;
  if (rateLimiterRate > FlyByWire_P.RateLimitereta_RisingLim_j) {
    rtb_Merge = FlyByWire_DWork.PrevY_d + FlyByWire_P.RateLimitereta_RisingLim_j;
  } else if (rateLimiterRate < FlyByWire_P.RateLimitereta_FallingLim_m) {
    rtb_Merge = FlyByWire_DWork.PrevY_d + FlyByWire_P.RateLimitereta_FallingLim_m;
  } else {
    rtb_Merge = rtb_BusAssignment_mv_pitch_output_eta_pos;
  }

  FlyByWire_DWork.PrevY_d = rtb_Merge;
  rtb_Merge *= FlyByWire_P.Gaineta_Gain_d;
  if (rtb_Merge > FlyByWire_P.Limitereta_UpperSat) {
    FlyByWire_Y.out.sim.raw.output.eta_pos = FlyByWire_P.Limitereta_UpperSat;
  } else if (rtb_Merge < FlyByWire_P.Limitereta_LowerSat) {
    FlyByWire_Y.out.sim.raw.output.eta_pos = FlyByWire_P.Limitereta_LowerSat;
  } else {
    FlyByWire_Y.out.sim.raw.output.eta_pos = rtb_Merge;
  }

  rateLimiterRate = rtb_LimiteriH - FlyByWire_DWork.PrevY_jg;
  if (rateLimiterRate > FlyByWire_P.RateLimiterxi_RisingLim_n) {
    rtb_Limiterxi1 = FlyByWire_DWork.PrevY_jg + FlyByWire_P.RateLimiterxi_RisingLim_n;
  } else if (rateLimiterRate < FlyByWire_P.RateLimiterxi_FallingLim_f) {
    rtb_Limiterxi1 = FlyByWire_DWork.PrevY_jg + FlyByWire_P.RateLimiterxi_FallingLim_f;
  } else {
    rtb_Limiterxi1 = rtb_LimiteriH;
  }

  FlyByWire_DWork.PrevY_jg = rtb_Limiterxi1;
  rtb_Limiterxi1 *= FlyByWire_P.Gainxi_Gain_n;
  if (rtb_Limiterxi1 > FlyByWire_P.Limiterxi_UpperSat) {
    FlyByWire_Y.out.sim.raw.output.xi_pos = FlyByWire_P.Limiterxi_UpperSat;
  } else if (rtb_Limiterxi1 < FlyByWire_P.Limiterxi_LowerSat) {
    FlyByWire_Y.out.sim.raw.output.xi_pos = FlyByWire_P.Limiterxi_LowerSat;
  } else {
    FlyByWire_Y.out.sim.raw.output.xi_pos = rtb_Limiterxi1;
  }

  rateLimiterRate = rtb_BusAssignment_a_sim_input_delta_zeta_pos - FlyByWire_DWork.PrevY_g;
  if (rateLimiterRate > FlyByWire_P.RateLimiterxi1_RisingLim_k) {
    rtb_Limiterxi1 = FlyByWire_DWork.PrevY_g + FlyByWire_P.RateLimiterxi1_RisingLim_k;
  } else if (rateLimiterRate < FlyByWire_P.RateLimiterxi1_FallingLim_o) {
    rtb_Limiterxi1 = FlyByWire_DWork.PrevY_g + FlyByWire_P.RateLimiterxi1_FallingLim_o;
  } else {
    rtb_Limiterxi1 = rtb_BusAssignment_a_sim_input_delta_zeta_pos;
  }

  FlyByWire_DWork.PrevY_g = rtb_Limiterxi1;
  rtb_Limiterxi1 *= FlyByWire_P.Gainxi1_Gain;
  if (rtb_Limiterxi1 > FlyByWire_P.Limiterxi1_UpperSat) {
    FlyByWire_Y.out.sim.raw.output.zeta_pos = FlyByWire_P.Limiterxi1_UpperSat;
  } else if (rtb_Limiterxi1 < FlyByWire_P.Limiterxi1_LowerSat) {
    FlyByWire_Y.out.sim.raw.output.zeta_pos = FlyByWire_P.Limiterxi1_LowerSat;
  } else {
    FlyByWire_Y.out.sim.raw.output.zeta_pos = rtb_Limiterxi1;
  }

  FlyByWire_Y.out.sim.raw.data = FlyByWire_U.in.data;
  FlyByWire_Y.out.sim.raw.input = FlyByWire_U.in.input;
  rtb_Merge = FlyByWire_P.GainiH_Gain * FlyByWire_DWork.DelayInput2_DSTATE;
  if (rtb_Merge > FlyByWire_P.LimiteriH_UpperSat) {
    FlyByWire_Y.out.sim.raw.output.iH_deg = FlyByWire_P.LimiteriH_UpperSat;
  } else if (rtb_Merge < FlyByWire_P.LimiteriH_LowerSat) {
    FlyByWire_Y.out.sim.raw.output.iH_deg = FlyByWire_P.LimiteriH_LowerSat;
  } else {
    FlyByWire_Y.out.sim.raw.output.iH_deg = rtb_Merge;
  }

  FlyByWire_Y.out.sim.raw.output.iH_deg_should_write = rtb_iH_deg_should_write;
  FlyByWire_Y.out.sim.data.nz_g = FlyByWire_U.in.data.nz_g;
  FlyByWire_Y.out.sim.data.Theta_deg = rtb_GainTheta;
  FlyByWire_Y.out.sim.data.Phi_deg = rtb_GainPhi;
  FlyByWire_Y.out.sim.data.rk_deg_s = FlyByWire_P.Gain_Gain_l * FlyByWire_U.in.data.rk_rad_s * FlyByWire_P.Gainrk_Gain;
  FlyByWire_Y.out.sim.data.qk_dot_deg_s2 = result[0];
  FlyByWire_Y.out.sim.data.rk_dot_deg_s2 = result[1];
  FlyByWire_Y.out.sim.data.pk_dot_deg_s2 = result[2];
  FlyByWire_Y.out.sim.data.Vk_kt = FlyByWire_U.in.data.Vk_kt;
  FlyByWire_Y.out.sim.data.radio_height_ft = FlyByWire_U.in.data.radio_height_ft;
  FlyByWire_Y.out.sim.data.CG_percent_MAC = FlyByWire_U.in.data.CG_percent_MAC;
  rtb_Merge = FlyByWire_P.Gain_Gain_i * FlyByWire_U.in.data.gear_animation_pos_0 - FlyByWire_P.Constant_Value_g;
  if (rtb_Merge > FlyByWire_P.Saturation_UpperSat) {
    FlyByWire_Y.out.sim.data.gear_strut_compression_0 = FlyByWire_P.Saturation_UpperSat;
  } else if (rtb_Merge < FlyByWire_P.Saturation_LowerSat) {
    FlyByWire_Y.out.sim.data.gear_strut_compression_0 = FlyByWire_P.Saturation_LowerSat;
  } else {
    FlyByWire_Y.out.sim.data.gear_strut_compression_0 = rtb_Merge;
  }

  FlyByWire_Y.out.sim.data.gear_strut_compression_2 = result_tmp;
  FlyByWire_Y.out.sim.data.flaps_handle_index = FlyByWire_U.in.data.flaps_handle_index;
  FlyByWire_Y.out.sim.data_computed.on_ground = rtb_onGround;
  FlyByWire_Y.out.sim.input.delta_eta_pos = rtb_Gaineta;
  FlyByWire_Y.out.sim.input.delta_xi_pos = rtb_Gainxi;
  FlyByWire_Y.out.sim.input.delta_zeta_pos = rtb_BusAssignment_a_sim_input_delta_zeta_pos;
  FlyByWire_Y.out.pitch.data_computed.in_flight = rtb_in_flight;
  FlyByWire_Y.out.pitch.data_computed.in_flare = rtb_in_flare;
  FlyByWire_Y.out.pitch.data_computed.in_flight_gain = rtb_BusAssignment_l_pitch_data_computed_in_flight_gain;
  FlyByWire_Y.out.pitch.data_computed.iH_deg_should_freeze = rtb_iH_deg_should_freeze;
  FlyByWire_Y.out.pitch.data_computed.iH_deg_reset = rtb_iH_deg_reset;
  FlyByWire_Y.out.pitch.data_computed.iH_deg_reset_deg = rtb_iH_deg_reset_deg;
  FlyByWire_Y.out.pitch.data_computed.iH_deg_should_write = rtb_iH_deg_should_write;
  FlyByWire_Y.out.pitch.data_computed.iH_deg_rate_limit_up_deg_s = rtb_iH_deg_rate_limit_up_deg_s;
  FlyByWire_Y.out.pitch.data_computed.iH_deg_rate_limit_lo_deg_s = rtb_iH_deg_rate_limit_lo_deg_s;
  FlyByWire_Y.out.pitch.law_normal.Cstar_g = rtb_Integration_m;
  FlyByWire_Y.out.pitch.law_protection.attitude_min.eta_dot_pos_s =
    rtb_BusAssignment_f_pitch_law_protection_attitude_min_eta_dot_pos_s;
  FlyByWire_Y.out.pitch.law_protection.attitude_max.eta_dot_pos_s =
    rtb_BusAssignment_f_pitch_law_protection_attitude_max_eta_dot_pos_s;
  FlyByWire_Y.out.pitch.vote.eta_dot_pos_s = rtb_Filter;
  FlyByWire_Y.out.pitch.integrated.eta_pos = FlyByWire_DWork.Integration_DSTATE;
  FlyByWire_Y.out.pitch.output.eta_pos = rtb_BusAssignment_mv_pitch_output_eta_pos;
  FlyByWire_Y.out.pitch.output.iH_deg = FlyByWire_DWork.DelayInput2_DSTATE;
  FlyByWire_Y.out.roll.data_computed.in_flight = rtb_inFlight;
  FlyByWire_Y.out.roll.data_computed.in_flight_gain = rtb_deltafalllimit;
  FlyByWire_Y.out.roll.law_normal.pk_c_deg_s = rtb_UkYk1;
  FlyByWire_Y.out.roll.law_normal.Phi_c_deg = FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE;
  FlyByWire_Y.out.roll.law_normal.xi_pos = rtb_BusAssignment_f_roll_law_normal_xi_pos;
  FlyByWire_Y.out.roll.output.xi_pos = rtb_LimiteriH;
  if (rtb_iH_deg_should_freeze == 1) {
    rtb_Merge = FlyByWire_P.Constant_Value;
  } else {
    rtb_Merge = FlyByWire_P.Convertto_Gain * FlyByWire_DWork.Integration_DSTATE;
  }

  FlyByWire_DWork.Filter_DSTATE += FlyByWire_P.Filter_gainval * rtb_FilterCoefficient;
  FlyByWire_DWork.Integration_IC_LOADING = 0U;
  FlyByWire_DWork.Integration_DSTATE += FlyByWire_P.Integration_gainval * rtb_Filter;
  if (FlyByWire_DWork.Integration_DSTATE >= FlyByWire_P.Integration_UpperSat) {
    FlyByWire_DWork.Integration_DSTATE = FlyByWire_P.Integration_UpperSat;
  } else {
    if (FlyByWire_DWork.Integration_DSTATE <= FlyByWire_P.Integration_LowerSat) {
      FlyByWire_DWork.Integration_DSTATE = FlyByWire_P.Integration_LowerSat;
    }
  }

  if (rtb_in_flight > 0) {
    FlyByWire_DWork.Integration_PrevResetState = 1;
  } else {
    FlyByWire_DWork.Integration_PrevResetState = 0;
  }

  FlyByWire_DWork.Integration_IC_LOADING_a = 0U;
  FlyByWire_DWork.Integration_DSTATE_e += FlyByWire_P.Gain_Gain_ip * rtb_Merge * FlyByWire_P.Integration_gainval_c;
  if (FlyByWire_DWork.Integration_DSTATE_e >= FlyByWire_P.Integration_UpperSat_f) {
    FlyByWire_DWork.Integration_DSTATE_e = FlyByWire_P.Integration_UpperSat_f;
  } else {
    if (FlyByWire_DWork.Integration_DSTATE_e <= FlyByWire_P.Integration_LowerSat_f) {
      FlyByWire_DWork.Integration_DSTATE_e = FlyByWire_P.Integration_LowerSat_f;
    }
  }

  if (rtb_iH_deg_reset > 0) {
    FlyByWire_DWork.Integration_PrevResetState_p = 1;
  } else {
    FlyByWire_DWork.Integration_PrevResetState_p = 0;
  }

  FlyByWire_DWork.DiscreteTimeIntegrator_IC_LOADING = 0U;
  FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE += FlyByWire_P.DiscreteTimeIntegrator_gainval * rtb_UkYk1;
  if (FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE >= FlyByWire_P.DiscreteTimeIntegrator_UpperSat) {
    FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE = FlyByWire_P.DiscreteTimeIntegrator_UpperSat;
  } else {
    if (FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE <= FlyByWire_P.DiscreteTimeIntegrator_LowerSat) {
      FlyByWire_DWork.DiscreteTimeIntegrator_DSTATE = FlyByWire_P.DiscreteTimeIntegrator_LowerSat;
    }
  }

  if (rtb_inFlight > 0) {
    FlyByWire_DWork.DiscreteTimeIntegrator_PrevResetState = 1;
  } else {
    FlyByWire_DWork.DiscreteTimeIntegrator_PrevResetState = 0;
  }
}

void FlyByWireModelClass::initialize()
{
  rt_InitInfAndNaN(sizeof(real_T));
  (void) std::memset(static_cast<void *>(&FlyByWire_DWork), 0,
                     sizeof(D_Work_FlyByWire_T));
  FlyByWire_U.in = FlyByWire_rtZfbw_input;
  FlyByWire_Y.out = FlyByWire_rtZfbw_output;
  FlyByWire_DWork.PrevY = FlyByWire_P.RateLimitereta_IC;
  FlyByWire_DWork.PrevY_c = FlyByWire_P.RateLimiterxi_IC;
  FlyByWire_DWork.PrevY_f = FlyByWire_P.RateLimiterxi1_IC;
  FlyByWire_DWork.PrevY_h = FlyByWire_P.RateLimit_IC;
  FlyByWire_DWork.PrevY_j = FlyByWire_P.RateLimiter_IC;
  FlyByWire_DWork.Filter_DSTATE = FlyByWire_P.PID_InitialConditionForFilter;
  FlyByWire_DWork.PrevY_i[0] = FlyByWire_P.Limiter_IC;
  FlyByWire_DWork.PrevY_i[1] = FlyByWire_P.Limiter_IC;
  FlyByWire_DWork.PrevY_i[2] = FlyByWire_P.Limiter_IC;
  FlyByWire_DWork.Integration_IC_LOADING = 1U;
  FlyByWire_DWork.Integration_PrevResetState = 2;
  FlyByWire_DWork.Integration_IC_LOADING_a = 1U;
  FlyByWire_DWork.Integration_PrevResetState_p = 0;
  FlyByWire_DWork.DelayInput2_DSTATE = FlyByWire_P.DelayInput2_InitialCondition;
  FlyByWire_DWork.PrevY_jv = FlyByWire_P.RateLimit_IC_d;
  FlyByWire_DWork.DiscreteTimeIntegrator_IC_LOADING = 1U;
  FlyByWire_DWork.DiscreteTimeIntegrator_PrevResetState = 2;
  FlyByWire_DWork.PrevY_d = FlyByWire_P.RateLimitereta_IC_c;
  FlyByWire_DWork.PrevY_jg = FlyByWire_P.RateLimiterxi_IC_c;
  FlyByWire_DWork.PrevY_g = FlyByWire_P.RateLimiterxi1_IC_i;
  FlyByWire_DWork.is_active_c1_FlyByWire = 0U;
  FlyByWire_DWork.is_c1_FlyByWire = FlyByWire_IN_NO_ACTIVE_CHILD;
  FlyByWire_DWork.is_active_c3_FlyByWire = 0U;
  FlyByWire_DWork.is_c3_FlyByWire = FlyByWire_IN_NO_ACTIVE_CHILD;
  FlyByWire_DWork.chartAbsoluteTimeCounter = 0;
  FlyByWire_DWork.is_active_c9_FlyByWire = 0U;
  FlyByWire_DWork.is_c9_FlyByWire = FlyByWire_IN_NO_ACTIVE_CHILD;
  FlyByWire_DWork.is_active_c8_FlyByWire = 0U;
  FlyByWire_DWork.is_c8_FlyByWire = FlyByWire_IN_NO_ACTIVE_CHILD;
  FlyByWire_DWork.is_active_c7_FlyByWire = 0U;
  FlyByWire_DWork.is_c7_FlyByWire = FlyByWire_IN_NO_ACTIVE_CHILD;
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
