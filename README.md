# airbus-fly-by-wire-wasm

This repository contains code to create a WASM module with a custom Fly-by-Wire system that can be used inside Microsoft Flight Simulator 2020. This custom Fly-by-Wire system is based on a [MATLAB/Simulink model](https://github.com/aguther/airbus-fly-by-wire-matlab).

## Disclaimer

This is work in progress, there are still a lot of issues. Examples:

- pitch attitude protections can oscillate -> should be improved now
- nose-down pitch attitude protection sometimes kicks-in too early -> should be improved now
- pitch normal law (C* law) sometimes oscillates on low speed -> should be improved now
- transformation from ground to flight mode might take longer than intended (nose might drop after releasing the stick)
- yaw damper / rudder control missing -> first version is now included
- auto-trim feature locks trim wheel completely -> fixed
- strange interaction with default auto thrust system -> thrust lever sometimes does not move, fix is to manually disable ATHR
- flare mode might be stronger than expected, needs to be investigated -> was reduced in authority, should be better now

:warning: **The WASM interface does not provide control about timing yet, therefore the nominal sample rate of the fly-by-wire model it was made for (0.02 s) is not met. The actual sample rate depends heavily on the render performance of flight simulator. This leads to the fact that signals, tunings, timings, filtering, transfer functions etc. might not work as intendet or designed.**

## Adding the WASM to the FlyByWireSim A32NX

To test drive the custom fly-by-wire system with the A32NX you need to do the following:

- get the latest stable release that is compatible with your sim version from [https://flybywiresim.com/a32nx](https://flybywiresim.com/a32nx)
- install it as described, result is usually that you have the folder `%LOCALAPPDATA%\Packages\Microsoft.FlightSimulator_8wekyb3d8bbwe\LocalCache\Packages\Community\A32NX`
- copy the file `external\airbus-custom-fbw\SimObjects\AirPlanes\Asobo_A320_NEO\panel\airbus-fly-by-wire.wasm` to the A32NX in your community folder `%LOCALAPPDATA%\Packages\Microsoft.FlightSimulator_8wekyb3d8bbwe\LocalCache\Packages\Community\A32NX\SimObjects\AirPlanes\Asobo_A320_NEO\panel`
- adapt the file `%LOCALAPPDATA%\Packages\Microsoft.FlightSimulator_8wekyb3d8bbwe\LocalCache\Packages\Community\A32NX\SimObjects\AirPlanes\Asobo_A320_NEO\panel\panel.cfg` and add the following, where `<NN>` is last `Vcockpit` entry + `1` (as of today the right replacement for `<NN>` is `17`):
```
[Vcockpit<NN>]
size_mm=0,0
pixel_size=0,0
texture=$PFD
background_color=0,0,0

htmlgauge00=WasmInstrument/WasmInstrument.html?wasm_module=airbus-fly-by-wire.wasm&wasm_gauge=FlyByWire, 0,0,0,0
```
Example:
```
<other entries>

[VCockpit16]
size_mm=1280,1280
pixel_size=1280,1280
texture=$MFD2

htmlgauge00=Airliners/A320_Neo/MFD/A320_Neo_MFD.html?Index=2, 0,0,1280,1280

[Vcockpit17]
size_mm=0,0
pixel_size=0,0
texture=$PFD
background_color=0,0,0

htmlgauge00=WasmInstrument/WasmInstrument.html?wasm_module=airbus-fly-by-wire.wasm&wasm_gauge=FlyByWire, 0,0,0,0

[VPainting01]
size_mm				= 2048,512
texture				= $RegistrationNumber
location 			= exterior

painting00=Registration/Registration.html?font_color=black,	0, 0, 2048, 512
```

- adapt the `%LOCALAPPDATA%\Packages\Microsoft.FlightSimulator_8wekyb3d8bbwe\LocalCache\Packages\Community\A32NX\layout.json` and add the following snippet:
```
{
  "path": "SimObjects/AirPlanes/Asobo_A320_NEO/panel/airbus-fly-by-wire.wasm",
  "size": 499351,
  "date": 132494951695343550
},
```
Example:
```
{
  "content": [

  <other entries>

    {
      "path": "SimObjects/AirPlanes/Asobo_A320_NEO/model/A320_NEO_INTERIOR_LOD00.gltf",
      "size": 4450768,
      "date": 132494951695343550
    },
    {
      "path": "SimObjects/AirPlanes/Asobo_A320_NEO/panel/airbus-fly-by-wire.wasm",
      "size": 499351,
      "date": 132494951695343550
    },
    {
      "path": "SimObjects/AirPlanes/Asobo_A320_NEO/panel/panel.cfg",
      "size": 2583,
      "date": 132494951695343550
    },
    
    <other entries>
    
  ]
}
``` 
