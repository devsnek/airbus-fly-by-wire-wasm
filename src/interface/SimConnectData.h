#pragma once

#include <MSFS/Legacy/gauges.h>
#include <SimConnect.h>

struct SimData
{
  double nz_g;
  double Theta_deg;
  double Phi_deg;
  double Vk_kt;
  double radio_height_ft;
  double CG_percent_MAC;
  SIMCONNECT_DATA_XYZ worldRotationVelocity;
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
};
