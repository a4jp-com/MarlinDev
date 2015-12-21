#define CUSTOMIZATION_H

#define UUID "17138bb3-b0e5-483f-9bfd-b9ffe9667e9b"

#define  STRING_CONFIG_H_AUTHOR "(Roxanne Neufeld)" // Who made the changes.

// The following define selects which electronics board you have.
// Please choose the name from boards.h that matches your setup
#ifndef MOTHERBOARD
  #define MOTHERBOARD BOARD_ULTIMAKER
#endif

// Optional custom name for your RepStrap or other custom machine
// Displayed in the LCD "Ready" message
//#define CUSTOM_MACHINE_NAME "G2s"

// Define this to set a unique identifier for this printer, (Used by some programs to differentiate between machines)
// You can use an online service to generate a random UUID. (eg http://www.uuidgenerator.net/version4)
//#define MACHINE_UUID "00000000-0000-0000-0000-000000000000"

// This defines the number of extruders
#define EXTRUDERS 2

// Offset of the extruders (uncomment if using more than one and relying on firmware to position when changing).
// The offset has to be X=0, Y=0 for the extruder 0 hotend (default extruder).
// For the other hotends it is their distance from the extruder 0 hotend.
#define EXTRUDER_OFFSET_X {0.0, 18.9}  // (in mm) for each extruder, offset of the hotend on the X axis
#define EXTRUDER_OFFSET_Y {0.0, 11.0}  // (in mm) for each extruder, offset of the hotend on the Y axis

//===========================================================================
//============================= Thermal Settings ============================
//===========================================================================

// 0 is not used
// 1 is 100k thermistor - best choice for EPCOS 100k (4.7k pullup)
#define TEMP_SENSOR_0 1
#define TEMP_SENSOR_1 1
#define TEMP_SENSOR_2 0
#define TEMP_SENSOR_3 0
#define TEMP_SENSOR_BED 1

// When temperature exceeds max temp, your heater will be switched off.
// This feature exists to protect your hotend from overheating accidentally, but *NOT* from thermistor short/failure!
// You should use MINTEMP for thermistor short/failure protection.
#define HEATER_0_MAXTEMP 240
#define HEATER_1_MAXTEMP 240
#define HEATER_2_MAXTEMP 240
#define HEATER_3_MAXTEMP 240

//===========================================================================
//============================== Delta Settings =============================
//===========================================================================
// Enable DELTA kinematics and most of the default configuration for Deltas
#define DELTA
#define Z_PROBE_ALLEN_KEY
#define MANUAL_ALLEN_KEY_DEPLOYMENT
#define IGNORE_STOWED_Z_PROBE

// Make delta curves from many straight lines (linear interpolation).
// This is a trade-off between visible corners (not enough segments)
// and processor overload (too many expensive sqrt calls).
#define DELTA_SEGMENTS_PER_SECOND 160

// Center-to-center distance of the holes in the diagonal push rods.
#define DELTA_DIAGONAL_ROD 	198.5			// Was >>>---> 196.0 // mm

// Horizontal offset from middle of printer to smooth rod center.
#define DELTA_SMOOTH_ROD_OFFSET 160.0 // mm

// Horizontal offset of the universal joints on the end effector.
#define DELTA_EFFECTOR_OFFSET 34.0 // mm

// Horizontal offset of the universal joints on the carriages.
#define DELTA_CARRIAGE_OFFSET 25.0 // mm

// Horizontal distance bridged by diagonal push rods when effector is centered.
//#define DELTA_RADIUS (DELTA_SMOOTH_ROD_OFFSET-DELTA_EFFECTOR_OFFSET-DELTA_CARRIAGE_OFFSET+1.5)  // was 1.6
//#define DELTA_RADIUS (DELTA_SMOOTH_ROD_OFFSET-DELTA_EFFECTOR_OFFSET-DELTA_CARRIAGE_OFFSET+1.00)  //Total value currently needs to be 102.00
#define DELTA_RADIUS (DELTA_SMOOTH_ROD_OFFSET-DELTA_EFFECTOR_OFFSET-DELTA_CARRIAGE_OFFSET+0.00)  //Total value at 101.00

// Print surface diameter/2 minus unreachable space (avoid collisions with vertical towers).
#define DELTA_PRINTABLE_RADIUS 90

// Effective X/Y positions of the three vertical towers.
#define SIN_60 0.8660254037844386
#define COS_60 0.5
#define DELTA_TOWER1_X -SIN_60*DELTA_RADIUS // front left tower
#define DELTA_TOWER1_Y -COS_60*DELTA_RADIUS
#define DELTA_TOWER2_X SIN_60*DELTA_RADIUS // front right tower
#define DELTA_TOWER2_Y -COS_60*DELTA_RADIUS
#define DELTA_TOWER3_X 0.0 // back middle tower
#define DELTA_TOWER3_Y DELTA_RADIUS

// Diagonal rod squared
#define DELTA_DIAGONAL_ROD_2 (DELTA_DIAGONAL_ROD*DELTA_DIAGONAL_ROD)

// Invert the stepper direction. Change (or reverse the motor connector) if an axis goes the wrong way.
#define INVERT_X_DIR true
#define INVERT_Y_DIR true
#define INVERT_Z_DIR true

// @section extruder

// For direct drive extruder v9 set to true, for geared extruder set to false.
#define INVERT_E0_DIR true

// Travel limits after homing (units are in mm)
#define X_MIN_POS (-DELTA_PRINTABLE_RADIUS)
#define Y_MIN_POS (-DELTA_PRINTABLE_RADIUS)

#define X_MAX_LENGTH (X_MAX_POS - X_MIN_POS)
#define Y_MAX_LENGTH (Y_MAX_POS - Y_MIN_POS)
#define Z_MAX_LENGTH (Z_MAX_POS - Z_MIN_POS)

//===========================================================================
//============================ Bed Auto Leveling ============================
//===========================================================================

#define AUTO_BED_LEVELING_FEATURE // Delete the comment to enable (remove // at the start of the line)
#define DEBUG_LEVELING_FEATURE

#define Z_MIN_PROBE_REPEATABILITY_TEST

// set the rectangle in which to probe
#define DELTA_PROBEABLE_RADIUS (DELTA_PRINTABLE_RADIUS-38)
#define LEFT_PROBE_BED_POSITION (-DELTA_PROBEABLE_RADIUS+40)
#define RIGHT_PROBE_BED_POSITION (DELTA_PROBEABLE_RADIUS+10)
#define FRONT_PROBE_BED_POSITION (-DELTA_PROBEABLE_RADIUS+25)
#define BACK_PROBE_BED_POSITION (DELTA_PROBEABLE_RADIUS-25)
#define MIN_PROBE_EDGE 5 // The Z probe square sides can be no smaller than this.
#define AUTO_BED_LEVELING_GRID_POINTS 5

#define ABL_PROBE_PT_1_X (54+19)
#define ABL_PROBE_PT_1_Y (61-16)
#define ABL_PROBE_PT_2_X (-75+19)
#define ABL_PROBE_PT_2_Y (50-16)
#define ABL_PROBE_PT_3_X (-22+19)
#define ABL_PROBE_PT_3_Y (-77-16)

// Offsets to the Z probe relative to the nozzle tip.
// X and Y offsets must be integers.
#define X_PROBE_OFFSET_FROM_EXTRUDER  36
#define Y_PROBE_OFFSET_FROM_EXTRUDER -11
#define Z_PROBE_OFFSET_FROM_EXTRUDER  -2.8

#define Z_RAISE_BEFORE_HOMING 20       // (in mm) Raise Z axis before homing (G28) for Z probe clearance.
                                       // Be sure you have this distance over your Z_MAX_POS in case.

#define XY_TRAVEL_SPEED 3000         // X and Y axis travel speed between probes, in mm/min.

#define Z_RAISE_BEFORE_PROBING 15   // How much the Z axis will be raised before traveling to the first probing point.
#define Z_RAISE_BETWEEN_PROBINGS 20 // How much the Z axis will be raised when traveling from between next probing points.
#define Z_RAISE_AFTER_PROBING 20    // How much the Z axis will be raised after the last probing point.

#define Z_MIN_PROBE_ENDSTOP

#define BED_CENTER_AT_0_0  // If defined, the center of the bed is at (X=0, Y=0)

// Manual homing switch locations:
// For deltabots this means top and center of the Cartesian print volume.
//#define MANUAL_Z_HOME_POS 203.85	// #define MANUAL_Z_HOME_POS 204.3
#define MANUAL_Z_HOME_POS 100.85	// Give us some head room

// delta homing speeds must be the same on xyz
#define HOMING_FEEDRATE_XYZ (200*10)

// delta speeds must be the same on xyz
#define DEFAULT_AXIS_STEPS_PER_UNIT   {80, 80, 80, 93}
#define DEFAULT_MAX_FEEDRATE          {1500, 1500, 1500, 45}    // (mm/sec)

#define DEFAULT_MAX_ACCELERATION      {5000,5000,5000,5000}

#define DEFAULT_ACCELERATION          1000    // X, Y, Z and E acceleration in mm/s^2 for printing moves
#define DEFAULT_RETRACT_ACCELERATION  2000    // E acceleration in mm/s^2 for retracts
#define DEFAULT_TRAVEL_ACCELERATION   1000    // X, Y, Z acceleration in mm/s^2 for travel (non printing) moves

// EEPROM
// The microcontroller can store settings in the EEPROM, e.g. max velocity...
// M500 - stores parameters in EEPROM
// M501 - reads parameters from EEPROM (if you need reset them after you changed them temporarily).
// M502 - reverts to the default "factory settings".  You still need to store them in EEPROM afterwards if you want to.
#define EEPROM_SETTINGS

// The RepRapDiscount Smart Controller (white PCB)
// http://reprap.org/wiki/RepRapDiscount_Smart_Controller
#define REPRAP_DISCOUNT_SMART_CONTROLLER

/**
 * Thermal Protection parameters
 */
#if ENABLED(THERMAL_PROTECTION_HOTENDS)
  #define THERMAL_PROTECTION_PERIOD 40        // Seconds
  #define THERMAL_PROTECTION_HYSTERESIS 4     // Degrees Celsius

  #define WATCH_TEMP_PERIOD 20                // Seconds
  #define WATCH_TEMP_INCREASE 2               // Degrees Celsius
#endif

// Default stepper release if idle. Set to 0 to deactivate.
#define DEFAULT_STEPPER_DEACTIVE_TIME (60*5)		// changed by Roxy from 60

// Add support for experimental filament exchange support M600; requires display
#if ENABLED(ULTIPANEL)
  #define FILAMENTCHANGEENABLE
  #if ENABLED(FILAMENTCHANGEENABLE)
    #define AUTO_FILAMENT_CHANGE                //This extrude filament until you press the button on LCD
    #define AUTO_FILAMENT_CHANGE_LENGTH 0.04    //Extrusion length on automatic extrusion loop
    #define AUTO_FILAMENT_CHANGE_FEEDRATE 300   //Extrusion feedrate (mm/min) on automatic extrusion loop
  #endif
#endif

#include "configurations/transitional_default_configurations/delta/generic/Configuration.h"

