#include <iostream>

#include "SimConnectData.h"
#include "FlyByWireInterface.h"

using namespace std;

bool FlyByWireInterface::connect()
{
  // initialize model
  model.initialize();

  // connect to sim connect
  return simConnectInterface.connect();
}

void FlyByWireInterface::disconnect()
{
  // disconnect from sim connect
  simConnectInterface.disconnect();

  // terminate model
  model.terminate();
}

bool FlyByWireInterface::update(
  double sampleTime
) {
  bool result = true;

  // get data & inputs
  result &= getModelInputDataFromSim();

  // step model
  model.step();

  // write output
  result &= writeModelOuputDataToSim();

  // return result
  return result;
}

bool FlyByWireInterface::getModelInputDataFromSim()
{
  // request data
  if (!simConnectInterface.requestData())
  {
    cout << "WASM: Request data failed!" << endl;
    return false;
  }

  // read data
  if (!simConnectInterface.readData())
  {
    cout << "WASM: Read data failed!" << endl;
    return false;
  }

  // get data from interface
  SimData simData = simConnectInterface.getSimData();
  SimInput simInput = simConnectInterface.getSimInput();

  // fill data into model
  model.FlyByWire_U.in.data.nz_g = simData.nz_g;
  model.FlyByWire_U.in.data.Theta_deg = simData.Theta_deg;
  model.FlyByWire_U.in.data.Phi_deg = simData.Phi_deg;
  model.FlyByWire_U.in.data.qk_rad_s = simData.worldRotationVelocity.x;
  model.FlyByWire_U.in.data.rk_rad_s = simData.worldRotationVelocity.y;
  model.FlyByWire_U.in.data.pk_rad_s = simData.worldRotationVelocity.z;
  model.FlyByWire_U.in.data.q_dot_rad_s2 = simData.bodyRotationAcceleration.x;
  model.FlyByWire_U.in.data.r_dot_rad_s2 = simData.bodyRotationAcceleration.y;
  model.FlyByWire_U.in.data.p_dot_rad_s2 = simData.bodyRotationAcceleration.z;
  model.FlyByWire_U.in.data.iH_deg = simData.iH_deg;
  model.FlyByWire_U.in.data.Vk_kt = simData.Vk_kt;
  model.FlyByWire_U.in.data.radio_height_ft = simData.radio_height_ft;
  model.FlyByWire_U.in.data.CG_percent_MAC = simData.CG_percent_MAC;
  model.FlyByWire_U.in.data.gear_animation_pos_0 = simData.geat_animation_pos_0;
  model.FlyByWire_U.in.data.gear_animation_pos_1 = simData.geat_animation_pos_1;
  model.FlyByWire_U.in.data.gear_animation_pos_2 = simData.geat_animation_pos_2;
  model.FlyByWire_U.in.data.flaps_handle_index = simData.flaps_handle_index;

  // fill inputs into model
  model.FlyByWire_U.in.input.delta_eta_pos = simInput.inputs[0];
  model.FlyByWire_U.in.input.delta_xi_pos = simInput.inputs[1];
  model.FlyByWire_U.in.input.delta_zeta_pos = simInput.inputs[2];

  // success
  return true;
}

bool FlyByWireInterface::writeModelOuputDataToSim()
{
  if (model.FlyByWire_Y.out.sim.raw.output.iH_deg_should_write)
  {
    // object to write with trim
    SimOutput output = {
      model.FlyByWire_Y.out.sim.raw.output.eta_pos,
      model.FlyByWire_Y.out.sim.raw.output.iH_deg,
      model.FlyByWire_Y.out.sim.raw.output.xi_pos,
      model.FlyByWire_Y.out.sim.raw.output.zeta_pos
    };

    // send data via sim connect
    if (!simConnectInterface.sendData(output))
    {
      cout << "WASM: Write data failed!" << endl;
      return false;
    }
  }
  else {
    // object to write without trim
    SimOutputNoTrim output = {
      model.FlyByWire_Y.out.sim.raw.output.eta_pos,
      model.FlyByWire_Y.out.sim.raw.output.xi_pos,
      model.FlyByWire_Y.out.sim.raw.output.zeta_pos
    };

    // send data via sim connect
    if (!simConnectInterface.sendData(output))
    {
      cout << "WASM: Write data failed!" << endl;
      return false;
    }
  }

  return true;
}
