#pragma once

#include <MSFS/Legacy/gauges.h>
#include <SimConnect.h>

#include "SimConnectInterface.h"
#include "fbw.h"

class FlyByWire
{
public:
  bool connect();

  void disconnect();

  bool update(
      double sampleTime);

private:
  bool isConnected = false;
  HANDLE hSimConnect = 0;

  SimConnectInterface simConnectInterface;
  fbwModelClass model;

  bool getModelInputDataFromSim();

  bool writeModelOuputDataToSim();
};
