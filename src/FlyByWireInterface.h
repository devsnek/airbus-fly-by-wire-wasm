#pragma once

#include <MSFS/Legacy/gauges.h>
#include <SimConnect.h>

#include "SimConnectInterface.h"
#include "FlyByWire.h"

class FlyByWireInterface
{
public:
  bool connect();

  void disconnect();

  bool update(
      double sampleTime
  );

private:
  SimConnectInterface simConnectInterface;
  FlyByWireModelClass model;

  bool getModelInputDataFromSim(
    double sampleTime
  );

  bool writeModelOuputDataToSim();
};
