// Copyright (C) Pololu Corporation.  See LICENSE.txt for details.

/// @file ir_sensors.h
/// This header provides functions for reading the robot's infrared
/// down-facing line/reflectance sensors.
///
/// These functions use a state machine of the RP2040's PIO1 module and
/// 12 instructions in its program memory.

#pragma once

#include <stdint.h>
#include <stdbool.h>

/// The latest raw readings from the down-facing line sensors.
///
/// The first element corresponds to the left sensor.
/// The readings are between 0 and 1024, with higher readings corresponding to
/// less light being reflected.
/// The line_sensors_read() function updates this array.
extern uint16_t line_sensors[5];

/// Minimum sensor values detected during line sensor calibration.
/// Raw readings at this level or below are mapped to 0.
extern uint16_t line_sensors_cal_min[5];

/// Maximum sensor values detected during line sensor calibration.
/// Raw readings at this level or above are mapped to 1000.
extern uint16_t line_sensors_cal_max[5];

/// The latest calibrated readings from the down-facing line sensors.
///
/// The first element corresponds to the left sensor.
/// These readings are between 0 and 1000, with higher readings corresponding to
/// less light being reflected.
/// The line_sensors_read_calibrated() function updates this array.
extern uint16_t line_sensors_calibrated[5];

/// Resets the line sensor calibration to its initial state.
/// In this state, all the calibrated readings are 0.
void line_sensors_reset_calibration(void);

/// Reads the line sensors 10 times to update the calibration.
///
/// The general procedure for line sensor calibration is to call this function
/// repeatedly while exposing the line sensors to the brightest and darkest
/// surfaces they are expected to sense.
///
/// This function updates the arrays line_sensors_cal_min and
/// line_sensors_cal_max.
void line_sensors_calibrate(void);

/// Starts a reading of the down-facing line sensors on the robot.
///
/// This function turns on the down-facing infrared emitters.
///
/// You can get the results of the reading by calling line_sensors_read() at a
/// later time.
void line_sensors_start_read(void);

/// Reads the down-facing line sensors on the robot.
///
/// This function turns on the down-facing infrared emitters while reading.
///
/// The readings are stored in the line_sensors array.
void line_sensors_read(void);

/// Reads the down-facing line sensors on the robot and calculates calibrated
/// readings for them.
///
/// This function calls line_sensors_read() and also updates the
/// line_sensors_calibrated array.
///
/// The calibated readings will only be valid if the line sensors have been
/// calibrated: see line_sensors_calibrate().
void line_sensors_read_calibrated(void);
