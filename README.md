# airbus-fly-by-wire-wasm

This repository contains code to create a WASM module with a custom Fly-by-Wire system that can be used inside Microsoft Flight Simulator 2020. This custom Fly-by-Wire system is based on a [MATLAB/Simulink model](https://github.com/aguther/airbus-fly-by-wire-matlab).

## Disclaimer

This is work in progress, there are still issues.

#### Considered solved:
- :heavy_check_mark: pitch attitude protections can oscillate
- :heavy_check_mark: nose-down pitch attitude protection sometimes kicks-in too early
- :heavy_check_mark: transformation from ground to flight mode might take longer than intended (nose might drop after releasing the stick)
- :heavy_check_mark: auto-trim feature locks trim wheel completely

#### In principle solved, but fine tuning necessary for different flight conditions (a lot of tests need to be done to check for behaviour):
- :large_orange_diamond: pitch normal law (C* law) sometimes oscillates on low speed
- :large_orange_diamond: pitch normal law (C* law) creates a too small pitch rate on low speeds
- :large_orange_diamond: yaw damper / rudder control missing
- :large_orange_diamond: flare mode might be stronger than expected, needs to be investigated
- :large_orange_diamond: after landing sometimes a slight pitch up moment is introduced, needs to be investigated

#### Not solved / missing:
- :x: strange interaction with default auto thrust system -> thrust lever sometimes does not move, fix is to manually disable ATHR
- :x: High speed protection
- :x: High angle of attack (AoA) protection
- :x: alternative law
- :x: direct law (in flight)

#### Compatibility
- :x: incompatible with YourControls add-on

## Adding the WASM to the FlyByWireSim A32NX

To test drive the custom fly-by-wire system with the A32NX you need to do the following:

- get the latest stable release that is compatible with your sim version from [https://flybywiresim.com/a32nx](https://flybywiresim.com/a32nx)
- install it as described, result is usually that you have the folder `%LOCALAPPDATA%\Packages\Microsoft.FlightSimulator_8wekyb3d8bbwe\LocalCache\Packages\Community\A32NX`
- copy the file `external\airbus-custom-fbw\SimObjects\AirPlanes\Asobo_A320_NEO\panel\airbus-fly-by-wire.wasm` to the A32NX in your community folder `%LOCALAPPDATA%\Packages\Microsoft.FlightSimulator_8wekyb3d8bbwe\LocalCache\Packages\Community\A32NX\SimObjects\AirPlanes\Asobo_A320_NEO\panel`
- adapt the file `%LOCALAPPDATA%\Packages\Microsoft.FlightSimulator_8wekyb3d8bbwe\LocalCache\Packages\Community\A32NX\SimObjects\AirPlanes\Asobo_A320_NEO\flight_model.cfg` and set the following:
```
[AIRPLANE_GEOMETRY]
fly_by_wire = 0

[STALL PROTECTION]
stall_protection = 0
```
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
