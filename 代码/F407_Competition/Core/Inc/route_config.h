#ifndef ROUTE_CONFIG_H
#define ROUTE_CONFIG_H
#include "route1_config.h"
/* Named defaults: chassis mm/deg/RPM are integers; arm travel is 0.001 mm/deg.
 * 0 RPM inherits the existing chassis/arm speed setting. Runtime changes RAM only.
 * Geometry below seeds defaults, NOT automatic runtime coordinate replanning. */
#define RP_CENTER       (ROUTE1_START_X-ROUTE1_RIGHT_X)
#define RP_SCAN_GO      (ROUTE1_SCAN_Y-ROUTE1_START_Y)
#define RP_SCAN_BACK    (ROUTE1_SCAN_Y-ROUTE1_TOP_Y)
#define RP_RAW_SHIFT    (ROUTE1_RIGHT_X-ROUTE1_CENTER_X)
#define RP_ROUGH_GO     (ROUTE1_ROUGH_Y-ROUTE1_TOP_Y)
#define RP_ROUGH_TURN   180U
#define RP_ROUGH_EXIT   (ROUTE1_BOTTOM_Y-ROUTE1_ROUGH_Y)
#define RP_TEMP_SHIFT   (ROUTE1_CENTER_X-ROUTE1_LEFT_X)
#define RP_TEMP_GO      (ROUTE1_BOTTOM_Y-ROUTE1_SCAN_Y)
#define RP_TEMP_OFFSET  ROUTE1_TURN_OFFSET_MM
#define RP_TEMP_TURN    90U
#define RP_HOME_TURN    90U
#define RP_HOME_OFFSET  ROUTE1_TURN_OFFSET_MM
#define RP_HOME_GO      (ROUTE1_SCAN_Y-ROUTE1_TOP_Y)
#define RP_HOME_CROSS   (ROUTE1_RIGHT_X-ROUTE1_LEFT_X)
#define RP_HOME_ENTRY   (ROUTE1_TOP_Y-ROUTE1_START_Y)
#define RP_HOME_SHIFT   (ROUTE1_START_X-ROUTE1_RIGHT_X)
/* Route 2 body-centre poses, map origin top-left, x right/y down (mm).
 * Edit station AND turn/lane poses together; recheck the swept footprint.
 * 300 mm square body: nominal work-zone margin 20 mm, turn margin 22.9 mm.
 * Fixed approaches happen before a visual session; exits undo them in the leg. */
#define ROUTE2_RAW_Y          250U
#define ROUTE2_ROUGH_Y        2080U
#define ROUTE2_ROUGH_TURN_Y   2015U
#define ROUTE2_TEMP_X         320U
#define ROUTE2_TEMP_TURN_X    385U
#define ROUTE2_LOWER_Y        2070U
#define RP_DRIVE_RPM    0U
#define RP_TURN_RPM     0U
#define RP_ARM_TURN     180000U
/* Teach the M6 reference at this working top. Heights below share the
 * existing ground-pick datum. Route moves are depths below the reference. */
#define ROUTE_M6_WORK_TOP_UM 160000U
#define RP_ARM_RAW         (ROUTE_M6_WORK_TOP_UM - 85000U)
#define RP_ARM_PICK        (ROUTE_M6_WORK_TOP_UM - 2000U)
#define RP_ARM_ROUGH_PLACE (ROUTE_M6_WORK_TOP_UM - 2000U)
#define RP_ARM_TEMP        (ROUTE_M6_WORK_TOP_UM - 2000U)
#define RP_ARM_DROP        0U                             /* h1: release at working top */
#define RP_ARM_TRAY_PICK   (ROUTE_M6_WORK_TOP_UM - 135000U) /* h2 */
#define RP_ARM_RPM      0U
#endif
