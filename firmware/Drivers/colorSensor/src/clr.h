/*
 * clr_calibration.h
 *
 *  Created on: Sep 18, 2021
 *      Author: ksstms
 *
 * RGB values are calculated like this (example for red):
 *
 *    Rmin = SCALE / R_P_MAX
 *    Rmax = SCALE / R_P_MIN
 *    R'   = SCALE / Rmeas
 *    R = (R' - Rmin) * 255 / (Rmax - Rmin)
 *
 * Calibration procedure:
 *  1) Use colorSensorGetPeriods() to check periods. Collect these
 *     values for black, white and (just to be sure) all other desired colors.
 *  2) Find the max/min of each component and set them below. Ideally,
 *     those will correspond to the black and white measurements.
 *  3) If the results are bad, try tweaking the scale value.
 */

#ifndef COLORSENSOR_CALIBRATION_H_
#define COLORSENSOR_CALIBRATION_H_

#define SCALE 10000

#define R_P_MAX 1100
#define R_P_MIN 70

#define G_P_MAX 1550
#define G_P_MIN 70

#define B_P_MAX 1800
#define B_P_MIN 65

#define R_MIN (SCALE / R_P_MAX)
#define R_MAX (SCALE / R_P_MIN)
#define G_MIN (SCALE / G_P_MAX)
#define G_MAX (SCALE / G_P_MIN)
#define B_MIN (SCALE / B_P_MAX)
#define B_MAX (SCALE / B_P_MIN)

#endif /* COLORSENSOR_CALIBRATION_H_ */
