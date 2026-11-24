#ifndef ROUTE1_CONFIG_H
#define ROUTE1_CONFIG_H
/* Map origin: upper left, x right, y down, mm. Nominal geometry only.
 * Start heading DOWN, rear arm UP, left-side camera RIGHT on the map.
 * Includes 80mm raw-zone projection and 150mm work strips; no body envelope
 * or camera/arm offset calibration is implied. Change then rebuild. */
#define ROUTE1_START_X 2250U
#define ROUTE1_START_Y 150U
#define ROUTE1_RIGHT_X 2125U
#define ROUTE1_LEFT_X 350U
#define ROUTE1_TOP_Y 315U
#define ROUTE1_BOTTOM_Y 2050U
#define ROUTE1_CENTER_X 1200U
#define ROUTE1_SCAN_Y 1200U
#define ROUTE1_TURN_OFFSET_MM 35U /* 300mm square: preserve ~23mm turn margin */
#define ROUTE1_ROUGH_OFFSET_MM 25U /* D: ~13mm before the white zone during a turn */
#define ROUTE1_ROUGH_Y (ROUTE1_BOTTOM_Y-ROUTE1_ROUGH_OFFSET_MM)
#define ROUTE1_TEMP_X (ROUTE1_LEFT_X+ROUTE1_TURN_OFFSET_MM)
#define ROUTE1_QR_TIMEOUT_MS 60000U
#if ROUTE1_START_X < ROUTE1_RIGHT_X+10 || ROUTE1_RIGHT_X < ROUTE1_CENTER_X+10 || ROUTE1_CENTER_X < ROUTE1_LEFT_X+10 || ROUTE1_TOP_Y < ROUTE1_START_Y+10 || ROUTE1_SCAN_Y < ROUTE1_TOP_Y+10 || ROUTE1_BOTTOM_Y < ROUTE1_SCAN_Y+10 || ROUTE1_ROUGH_Y < ROUTE1_TOP_Y+10
#error Route1 motion segments must be at least 10mm
#endif
#if ROUTE1_TURN_OFFSET_MM < 10 || ROUTE1_TURN_OFFSET_MM > 50 || ROUTE1_ROUGH_OFFSET_MM < 10 || ROUTE1_ROUGH_OFFSET_MM > 50 || ROUTE1_ROUGH_Y <= ROUTE1_SCAN_Y || ROUTE1_TEMP_X >= ROUTE1_CENTER_X
#error Invalid route1 turn offset
#endif
#if ROUTE1_START_X <= ROUTE1_RIGHT_X || ROUTE1_RIGHT_X <= ROUTE1_CENTER_X || ROUTE1_CENTER_X <= ROUTE1_LEFT_X || ROUTE1_LEFT_X < 10 || ROUTE1_START_X > 2400
#error Invalid route1 x coordinates
#endif
#if ROUTE1_START_Y < 10 || ROUTE1_START_Y >= ROUTE1_TOP_Y || ROUTE1_TOP_Y >= ROUTE1_SCAN_Y || ROUTE1_SCAN_Y >= ROUTE1_BOTTOM_Y || ROUTE1_BOTTOM_Y > 2400
#error Invalid route1 y coordinates
#endif
#if ROUTE1_QR_TIMEOUT_MS < 1000 || ROUTE1_QR_TIMEOUT_MS > 180000
#error Invalid route1 QR timeout
#endif
#endif
