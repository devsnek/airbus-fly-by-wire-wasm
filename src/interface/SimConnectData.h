#pragma once

#include <MSFS/Legacy/gauges.h>
#include <SimConnect.h>

struct SimData
{
  double nz_g;
  double Theta_deg;
  double Phi_deg;
  SIMCONNECT_DATA_XYZ worldRotationVelocity;
  SIMCONNECT_DATA_XYZ bodyRotationAcceleration;
  double iH_deg;
  double Vk_kt;
  double radio_height_ft;
  double CG_percent_MAC;
  double geat_animation_pos_0;
  double geat_animation_pos_1;
  double geat_animation_pos_2;
  double flaps_handle_index;
};

struct SimInput
{
  double inputs[3];
};

struct SimOutput
{
  double eta;
  double iH;
  double xi;
  double zeta;
};

struct SimOutputNoTrim
{
  double eta;
  double xi;
  double zeta;
};
