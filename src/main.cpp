// airbus_fly_by_wire_wasm.cpp

#include <MSFS/MSFS.h>

#include "main.h"
#include "FlyByWire.h"

FlyByWire flyByWire;

extern "C" {
  MSFS_CALLBACK bool FlyByWire_gauge_callback(
    FsContext ctx,
    int service_id,
    void* pData
  ) {
    // print event type
    switch (service_id) {
      case PANEL_SERVICE_PRE_INSTALL: {
        // connect to sim connect
        return flyByWire.connect();
      };

      case PANEL_SERVICE_PRE_DRAW: {
        // read data & inputs, step model, write output
        return flyByWire.update(static_cast<sGaugeDrawData*>(pData)->dt);
      };

      case PANEL_SERVICE_PRE_KILL: {
        // disconnect sim connect
        flyByWire.disconnect();
      } break;
    }

    // success
    return true;
  }
}
