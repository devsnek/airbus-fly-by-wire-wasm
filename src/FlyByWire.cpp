#include <iostream>

#include "SimConnectData.h"
#include "FlyByWire.h"

using namespace std;

bool FlyByWire::connect()
{
  // initialize model
  model.initialize();

  // connect to sim connect
  return simConnectInterface.connect();
}

void FlyByWire::disconnect()
{
  // disconnect from sim connect
  simConnectInterface.disconnect();

  // terminate model
  model.terminate();
}

bool FlyByWire::update(
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

bool FlyByWire::getModelInputDataFromSim()
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
  model.fbw_U.in_sim_simrawdata_nz_g = simData.nz_g;
  model.fbw_U.in_sim_simrawdata_Theta_deg = simData.Theta_deg;
  model.fbw_U.in_sim_simrawdata_Phi_deg = simData.Phi_deg;
  model.fbw_U.in_sim_simrawdata_Vk_kt = simData.Vk_kt;
  model.fbw_U.in_sim_simrawdata_radio_height_ft = simData.radio_height_ft;
  model.fbw_U.in_sim_simrawdata_CG_percent_MAC = simData.CG_percent_MAC;
  model.fbw_U.in_sim_simrawdata_qk_rad_s = simData.worldRotationVelocity.x;
  model.fbw_U.in_sim_simrawdata_rk_rad_s = simData.worldRotationVelocity.y;
  model.fbw_U.in_sim_simrawdata_pk_rad_s = simData.worldRotationVelocity.z;

  // fill inputs into model
  model.fbw_U.in_sim_simrawinput_delta_eta_pos = simInput.inputs[0];
  model.fbw_U.in_sim_simrawinput_delta_xi_pos = simInput.inputs[1];

  // success
  return true;
}

bool FlyByWire::writeModelOuputDataToSim()
{
  // object to write
  SimOutput output = {
    model.fbw_Y.out_sim_simrawoutput_eta_pos,
    model.fbw_Y.out_sim_simrawoutput_iH_deg,
    model.fbw_Y.out_sim_simrawoutput_xi_pos
  };

  // send data via sim connect
  if (!simConnectInterface.sendData(output))
  {
    cout << "WASM: Write data failed!" << endl;
    return false;
  }

  return true;
}
