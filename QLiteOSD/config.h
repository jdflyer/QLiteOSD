/*
 *  QLiteOSD is an simple OSD for DJI FPV System:
 *  This is an Arduino project that handles basic OSD functions
 *  from BMP280 to a Simple Voltage Sensor to feed it 
 *  to DJI FPV System.
 *
 * ------------------------------------------------
 *
 * Copyright (C) 2024 David Payne
 * 
 * This software is based on and uses software published by Paul Kurucz (pkuruz):opentelem_to_bst_bridge
 * as well as software d3ngit : djihdfpv_mavlink_to_msp_V2 and crashsalot : VOT_to_DJIFPV
 * 
 * License info: See the LICENSE file at the repo top level
 *
 * THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 */

#define ESP8266  // Uncommment this line if using the ESP8266 (Wemos D1 Mini)
#define USE_GPS  //comment out to disable.  Reads and displays GPS data - requires Nano 328 due to file size
#if defined(ESP8266) && defined(USE_GPS) // do not remove or comment this line out
#define LOG_GPS //uncomment to log position and altitude data to the internal filesystem (requires a pushbutton to be added to switch to wifi mode)
#endif
//#define DEBUG    // uncomment this line to debug in Serial Monitor 

//********************************************************************
//  These settings are the defaults and are only loaded the first run.
//  When the values are updated in the web interface they are retained
//  until a Reset from the web interface.

char craftname[15] = "QLiteOSD"; // Do not make larger than 14 characters
boolean IMPERIAL_UNITS = true;  // Set to false to see units in Metric
boolean USE_PWM_ARM = false;  // pin D5 ESP8266 -- If commented out arming will occure with altitude change

//OSD Elements suppported by DJI
uint16_t osd_altitude_pos = 2240;
uint16_t osd_avg_cell_voltage_pos = 2145;
uint16_t osd_main_batt_voltage_pos = 2113;
uint16_t osd_crosshairs_pos = 2254;
uint16_t osd_craft_name_pos = 2048;

// GPS related -- change each to 234 to hide
uint16_t osd_gps_sats_pos = 2074;
uint16_t osd_home_dir_pos = 2095;
uint16_t osd_home_dist_pos = 2337;
uint16_t osd_gps_speed_pos = 2305;
uint16_t osd_gps_lat_pos = 2432;
uint16_t osd_gps_lon_pos = 2464;

 