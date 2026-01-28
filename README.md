# ClearCore-library 

This repository contains the ClearCore Motion and I/O Library, providing a foundation to build ClearCore applications. Also included are Microchip Studio example programs that demonstrate various features of the ClearCore, and an Microchip Studio template project that can be used to start building your own application.

## Enhanced Features

This library includes enhanced motion control capabilities:

### Coordinated Motion
- **Coordinated Arc Moves**: Generate smooth circular arc paths with two motors moving synchronously
- **Coordinated Linear Moves**: Straight-line coordinated motion with constant velocity along the path
- **Continuous Motion Chaining**: Seamlessly chain multiple arcs or linear moves without stopping
- **Constant Path Velocity**: Maintains constant velocity along the programmed path (arc circumference or linear distance)

### Unit Conversion Support
- **Physical Units**: Program moves in inches, millimeters, centimeters, meters, revolutions, or degrees instead of raw step counts
- **Feed Rate Units**: Set velocities in inches/minute, mm/minute, mm/second, RPM, or steps/second
- **Mechanical Configuration**: Configure motors with steps per revolution, lead screw pitch, and gear ratios
- **Automatic Conversion**: Library automatically converts between physical units and step counts based on your mechanical setup

### Key Benefits
- **Intuitive Programming**: Work with familiar units (inches, mm) instead of calculating step counts manually
- **Reduced Errors**: Eliminate manual conversion calculations that can introduce mistakes
- **G-Code Compatible**: Natural fit for CNC applications and G-code interpreters
- **Backward Compatible**: All existing step-based APIs continue to work unchanged

See `NEW_FILES.md` for a complete list of new files added for these features.

#### Microchip Studio requirements

The included Microchip Studio projects require Microchip Studio version 7.0.1645 or later (latest version is recommended).

From the Microchip Studio Tools menu, open the Device Pack Manager. Ensure the following packs are installed:
* SAME53_DFP version 1.1.118
* CMSIS version 4.5.0

#### Installers and Resources

https://www.teknic.com/downloads/

### libClearCore

libClearCore provides a C++ object oriented API to interface with the ClearCore hardware. Each connector of the ClearCore has an associated object to use in your application. A Doxygen reference manual for the libClearCore API is available at https://teknic-inc.github.io/ClearCore-library/.

There is an Microchip Studio project file (*.cppproj) included to load and compile this library in Microchip Studio.

### LwIP

The ClearCore Ethernet implementation is based off of the LwIP stack. Ethernet applications should be developed using the ethernet API provided by libClearCore. The LwIP source code is included for completeness.

There is an Microchip Studio project file (*.cppproj) included to load and compile this library in Microchip Studio.

### Microchip_Examples

This folder contains example applications for a variety of ClearCore features. To run a provided example, first choose which subdirectory describes the feature that you want to run. Within each subdirectory is an Microchip solution file (*.atsln) that contains various examples related to that feature, as well as the required interface libraries. After the solution is loaded in Microchip Studio, browse for the project with the example that you wish to run within the solution explorer panel. Right click on the project and select "Set as Startup Project". 

The example programs are configured with a custom firmware loading script that will search for a connected ClearCore USB port and load the example programs on the ClearCore hardware. Simply click "Start Without Debugging (Ctrl+Alt+F5)" and the example program will compile, load the firmware, and start executing.

#### Coordinated Motion Examples

- **CoordinatedArcMoves**: Demonstrates coordinated arc moves with continuous chaining
- **CoordinatedMovesWithUnits**: Comprehensive example showing:
  - Single motor moves in physical units (inches, mm)
  - Coordinated linear moves with unit conversion
  - Coordinated arc moves with unit conversion
  - Feed rate programming in physical units
  - Continuous move chaining

For detailed documentation on coordinated motion features, see `docs/CoordinatedArcMotion/README.md`.

### Project Template
The Project Template directory is included as a starting point for writing your own application. Simply open the Microchip Studio solution file (*.atsln), and put your application code in main.cpp.

### Tools

We have included Windows tools for loading the firmware onto the ClearCore using the USB connector. 

**bossac** A command line flashing application

**flash_clearcore.cmd** A script that searches for a connected ClearCore USB port and uses bossac to load the firmware

**uf2-builder** Converts the compiled firmware binary file into a UF2 file that allows drag and drop flashing onto the bootloader's mass storage drive.

## Documentation

- **API Reference**: https://teknic-inc.github.io/ClearCore-library/
- **Coordinated Motion Documentation**: See `docs/CoordinatedArcMotion/README.md` for detailed theory of operation, usage guide, and API reference
- **New Files**: See `NEW_FILES.md` for a complete listing of files added for coordinated motion and unit support features

## License

The ClearCore library is provided under the MIT License. New coordinated motion and unit conversion features are also licensed under the MIT License with copyright attribution to Adam G. Sweeney <agsweeney@gmail.com>.
