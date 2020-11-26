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
  result &= getModelInputDataFromSim(sampleTime);

  // step model
  model.step();

  // write output
  result &= writeModelOuputDataToSim();

  // return result
  return result;
}

bool FlyByWireInterface::getModelInputDataFromSim(
  double sampleTime
)
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

  // fill time into model
  model.FlyByWire_U.in.time.dt = sampleTime;

  // fill data into model
  model.FlyByWire_U.in.data.nz_g = simData.nz_g;
  model.FlyByWire_U.in.data.Theta_deg = simData.Theta_deg;
  model.FlyByWire_U.in.data.Phi_deg = simData.Phi_deg;
  model.FlyByWire_U.in.data.q_rad_s = simData.bodyRotationVelocity.x;
  model.FlyByWire_U.in.data.r_rad_s = simData.bodyRotationVelocity.y;
  model.FlyByWire_U.in.data.p_rad_s = simData.bodyRotationVelocity.z;
  model.FlyByWire_U.in.data.q_dot_rad_s2 = simData.bodyRotationAcceleration.x;
  model.FlyByWire_U.in.data.r_dot_rad_s2 = simData.bodyRotationAcceleration.y;
  model.FlyByWire_U.in.data.p_dot_rad_s2 = simData.bodyRotationAcceleration.z;
  model.FlyByWire_U.in.data.eta_pos = simData.eta_pos;
  model.FlyByWire_U.in.data.eta_trim_deg = simData.eta_trim_deg;
  model.FlyByWire_U.in.data.xi_pos = simData.xi_pos;
  model.FlyByWire_U.in.data.zeta_pos = simData.zeta_pos;
  model.FlyByWire_U.in.data.zeta_trim_pos = simData.zeta_trim_pos;
  model.FlyByWire_U.in.data.alpha_deg = simData.alpha_deg;
  model.FlyByWire_U.in.data.beta_deg = simData.beta_deg;
  model.FlyByWire_U.in.data.beta_dot_deg_s = simData.beta_dot_deg_s;
  model.FlyByWire_U.in.data.V_ias_kn = simData.V_ias_kn;
  model.FlyByWire_U.in.data.V_tas_kn = simData.V_tas_kn;
  model.FlyByWire_U.in.data.V_mach = simData.V_mach;
  model.FlyByWire_U.in.data.H_ft = simData.H_ft;
  model.FlyByWire_U.in.data.H_ind_ft = simData.H_ind_ft;
  model.FlyByWire_U.in.data.H_radio_ft = simData.H_radio_ft;
  model.FlyByWire_U.in.data.CG_percent_MAC = simData.CG_percent_MAC;
  model.FlyByWire_U.in.data.gear_animation_pos_0 = simData.geat_animation_pos_0;
  model.FlyByWire_U.in.data.gear_animation_pos_1 = simData.geat_animation_pos_1;
  model.FlyByWire_U.in.data.gear_animation_pos_2 = simData.geat_animation_pos_2;
  model.FlyByWire_U.in.data.flaps_handle_index = simData.flaps_handle_index;
  model.FlyByWire_U.in.data.autopilot_master_on = simData.autopilot_master_on;
  model.FlyByWire_U.in.data.slew_on = simData.slew_on;
  model.FlyByWire_U.in.data.pause_on = 0;

  // fill inputs into model
  model.FlyByWire_U.in.input.delta_eta_pos = simInput.inputs[0];
  model.FlyByWire_U.in.input.delta_xi_pos = simInput.inputs[1];
  model.FlyByWire_U.in.input.delta_zeta_pos = simInput.inputs[2];

  // success
  return true;
}

bool FlyByWireInterface::writeModelOuputDataToSim()
{
  // when tracking mode is on do not write anything
  if (model.FlyByWire_Y.out.sim.data_computed.tracking_mode_on)
  {
    return true;
  }

  // object to write with trim
  SimOutput output = {
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

  if (model.FlyByWire_Y.out.sim.raw.output.eta_trim_deg_should_write)
  {
    // object to write without trim
    SimOutputEtaTrim output = {
      model.FlyByWire_Y.out.sim.raw.output.eta_trim_deg
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
