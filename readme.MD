# Description

The Simionic G1000 hardware bezels are designed to run in one of 2 configurations

* With an iPad inserted into each G1000 bezel. The bezel sends button presses and rotary encoder inputs to the iPad via Bluetooth LE. The iPad has it's own G1000 software which reacts immediately to the input, and then the results are also sent to Microsoft Flight Simulator. If you are in VR and can only see the "in-sim" G1000 panels, there is enough of a delay for it to be a very unpleasant user experience.
* With a Simionic supplied LCD screen panel and USB serial convertor. The in-sim G1000 panels are popped out and dragged into these screens which act second and third monitors. This requires extra expense which is not required for use in VR

I wanted a third option, which is more suited to a VR setup:

* Replace the screen with some non-functioning blanking panel (probably perspex or similar). The G1000 units use bluetooth to send button and encoder inputs directly to an app running on the MSFS PC. This app reads those values and maps them to relevant SimConnect events (likely HVars) and updates the values in the simulator. 

# Build and Run

## Create the Build Environments

If CmakeLists.txt or cache variables change, re-run the following from the "x64 Native Tools Command Prompt for Visual Studio 2022":

```cmake --preset debug ``` or ```cmake --preset release```

## Build the Project

```cmake --build --preset debug``` or ```cmake --build --preset release```

Note: if the app fails to find the bluetooth adapter (and you definitely have one), use dumpbin to confirm that you have built a 64 bit version.

```dumpbin /headers build\debug\msfs_ble_g1000.exe | findstr /C:"machine"```


## Debugging in VS2022

 Tools > Options > CMake check “Use CMake when both CMakeLists.txt and solution are present”