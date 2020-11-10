# airbus-fly-by-wire-wasm

This repository contains code to create a WASM module with a custom Fly-by-Wire system that can be used inside Microsoft Flight Simulator 2020. This custom Fly-by-Wire system is based on a [MATLAB/Simulink model](https://github.com/aguther/airbus-fly-by-wire-matlab).

## Disclaimer

This is work in progress, there are still a lot of issues. Examples:

- pitch attitude protections can oscillate
- nose-down pitch attitude protection sometimes kicks-in too early
- pitch normal law (C* law) sometimes oscillates on low speed -> should be improved now
- transformation from ground to flight mode might take longer than intended (nose might drop after releasing the stick)
- yaw damper / rudder control missing
- auto-trim feature locks trim wheel completely -> fixed
- strange interaction with default auto thrust system -> thrust lever sometimes does not move
- flare mode might be stronger than expected, needs to be investigated

:warning: **The WASM interface does not provide control about timing yet, therefore the nominal sample rate of the fly-by-wire model it was made for (0.02 s) is not met. The actual sample rate depends heavily on the render performance of flight simulator. This leads to the fact that signals, tunings, timings, filtering, transfer functions etc. might not work as intendet or designed.**
