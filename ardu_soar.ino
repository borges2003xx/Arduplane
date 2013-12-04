 /// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/*
 *  ArduSoar support functions
 *
 *  Peter Braswell 
 * 
 *  This firmware is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 */
#include <MatrixMath.h>
#include <ExtendedKalmanFilter.h>

#define N 4
#define MIN_THERMAL_TIME_MS  300000
#define MIN_CRUISE_TIME_MS  30000
#define RUN_FILTER 1

//gcs_send_text_P(SEVERITY_LOW, PSTR("In soar code, initialising variables"));
float x[N] = { 10, 10, 0, 0 }; //State vector
float p[N][N] = {{2, 0, 0, 0},{0,10,0,0},{0,0,20,0},{0,0,0,20}}; //Covaraince matric
float cov_r = pow(0.5,2); //Measurement covariance
float r[1][1] = {{cov_r}};
float cov_q = pow(0.1,2); //State covariance
float q[N][N] = {{cov_q, 0, 0, 0},{0, cov_q, 0, 0},{0,0, cov_q,0},{0,0,0,cov_q}};
float x2 = abs(-2.0);
ExtendedKalmanFilter ekf (x,p,q,r);  

 // Keep track of the previous flight mode so we can transition back
 // When we come out of thermal mode.
 static FlightMode previous_control_mode;
 
 // Keep track of the waypoint so we can restore after coming out of thermal mode.
 static struct Location prev_next_wp;
 
 //Store aircraft location at last update
 static struct Location prev_update_location;
 
 // store time thermal was entered for hysteresis
 unsigned long thermal_start_time_ms;
 
  // store time cruise was entered for hysteresis
 unsigned long cruise_start_time_ms;
 
 // store time of last update
 unsigned long prev_update_time;
 //gcs_send_text_P(SEVERITY_LOW, PSTR("Soar initialisation complete"));
 
 
 // Check to see if see if we should be thermalling
 static FlightMode thermal(FlightMode current_control_mode) {

   FlightMode calculated_control_mode = current_control_mode;
   
   if( g.soar_active == 1 ) {
     
     //if ((read_climb_rate() > g.thermal_vspeed ) ) { //&& (( millis()- cruise_start_time_ms ) > MIN_CRUISE_TIME_MS )) {
     if ((read_climb_rate() > g.thermal_vspeed ) && (( millis()- cruise_start_time_ms ) > MIN_CRUISE_TIME_MS )) {
       //gcs_send_text_P(SEVERITY_LOW, PSTR("Thermal detected, entering loiter"));
       hal.console->printf_P(PSTR("Thermal detected, entering loiter\n"));
       previous_control_mode = current_control_mode;
       
       prev_next_wp = next_WP;
       
       next_WP = current_loc; //for now set next wp to current location - it would be better if this was a set distance ahead of a/c 
       //as this represents the thermal location, and we know we are flying into the thermal.
       prev_update_location = current_loc; // needed to see how far the move the thermal relative to the a/c
       
       calculated_control_mode =  LOITER;
       
       thermal_start_time_ms = millis();
        
       // New state vector filter will be reset to. Thermal location is placed 10m in front of a/c 
       //float xr[] = {10.0,10.0,0.0,0.0};
       float xr[] = {5.0,15.0,10.0*cos(ahrs.yaw),10.0*sin(ahrs.yaw)};    
       // Also reset covariance matrix p so filter is not affected by previous data       
       ekf.reset(xr,p);
       
       prev_update_location = current_loc;                                // save for next time
       prev_update_time = millis();
     }
   }
   
   return calculated_control_mode;
   
 }
 
 // Check to see if we've topped out of a thermal and 
 // Should transition to cruise (or rather the previous control mode).
 static FlightMode cruise(FlightMode current_control_mode) {
   
   FlightMode calculated_control_mode = current_control_mode;
 
   if ( g.soar_active == 1 ) {
     if ( (read_climb_rate() < 0)  && ((millis() - thermal_start_time_ms) > MIN_THERMAL_TIME_MS) ) {
       //gcs_send_text_P(SEVERITY_LOW, PSTR("Thermal weak, reentering previous mode"));
       hal.console->printf_P(PSTR("Thermal weak, reentering previous mode\n"));
       calculated_control_mode =  previous_control_mode;
       next_WP = prev_next_wp;
     }
     else {
       if (RUN_FILTER)  { // allow testing wind drift following only
         // still in thermal - need to update the wp location and update the filter according to new measurement
         
         // now update filter
         float dx = get_offset_north(&current_loc, &prev_update_location);  // get distances from previous update
         float dy = get_offset_east(&current_loc, &prev_update_location);
         
         Vector3f wind = ahrs.wind_estimate();
         // N.B. can use wind.x and wind.y to correct for thermal wind drift - need to ascertain sense.
         // Need to reconsider this: if thermal is defined relative to a/c and a/c is also drifting with wind is it necessary?
         //dx += wind.x * (millis()-prev_update_time)/1000.0;
         //dy += wind.y * (millis()-prev_update_time)/1000.0;
         ekf.update(read_climb_rate(),dx, dy);                              // update the filter
         
         next_WP = current_loc; // as filter estimate is based on offset from current location
         location_offset(&next_WP, ekf.X[3], ekf.X[4]); //update the WP
         
         hal.console->printf_P(PSTR("%f %f\n"),ekf.X[3], ekf.X[4]);
         
         prev_update_location = current_loc;                                // save for next time
         prev_update_time = millis();
       }
       else {
         Vector3f wind = ahrs.wind_estimate(); //Always gives zero in FlightGear
         wind.y = 5.0; //
         // N.B. can use wind.x and wind.y to correct for thermal wind drift - need to ascertain sense.
         float dx = wind.x * ((float)(millis()-prev_update_time))/1000.0;
         float dy = wind.y * ((float)(millis()-prev_update_time))/1000.0;
         
         int32_t prev_lng = next_WP.lng;
         
         location_offset(&next_WP, dx, dy); //update the WP
         
         hal.console->printf_P(PSTR("%f %f %ld %ld\n"),dx,dy,next_WP.lat,next_WP.lng);
         prev_update_location = current_loc;                                // save for next time
         prev_update_time = millis();
       }
     }
   }
   return calculated_control_mode;
 }
