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
float p[N][N] = {{2.0, 0, 0, 0},{0,10.0,0,0},{0,0,20.0,0},{0,0,0,20.0}}; //Covaraince matric
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
       
       next_WP = current_loc; // filter offsets based on ac location
         
       prev_update_location = current_loc; // needed to see how far the move the thermal relative to the a/c
       
       calculated_control_mode =  LOITER;
       
       thermal_start_time_ms = millis();
        
       // New state vector filter will be reset. Thermal location is placed 10m in front of a/c 
       //float xr[] = {10.0,10.0,0.0,0.0};
       float xr[] = {2.0,100.0,50.0*cos(ahrs.yaw),50.0*sin(ahrs.yaw)};      
       // Also reset covariance matrix p so filter is not affected by previous data       
       ekf.reset(xr,p);
       
       if (1) {
         location_offset(&next_WP, ekf.X[3], ekf.X[4]); //place waypoint to reflect filter state
       }
       else
       {
         location_offset(&next_WP, 1000.0*cos(ahrs.yaw), 1000.0*sin(ahrs.yaw));
       }
       prev_update_location = current_loc;                                // save for next time
       prev_update_time = millis();
     }
   }
   
   return calculated_control_mode;
   
 }
 
 // Check to see if we've topped out of a thermal and 
 // Should transition to cruise (or rather the previous control mode).
 static FlightMode cruise(FlightMode current_control_mode) {
    
   FlightMode calculated_control_mode = current_control_mode;  // default  behaviour is to keep current mode
 
   if ( g.soar_active == 1 ) {
     
     float climb_rate = read_climb_rate();
     
     if ( (climb_rate < 0)  && ((millis() - thermal_start_time_ms) > MIN_THERMAL_TIME_MS) ) {  
       //gcs_send_text_P(SEVERITY_LOW, PSTR("Thermal weak, reentering previous mode"));
       hal.console->printf_P(PSTR("Thermal weak, reentering previous mode\n"));
       calculated_control_mode =  previous_control_mode;
       next_WP = prev_next_wp;    // continue to the waypoint being used before thermal mode
     }
     else {
       if (RUN_FILTER)  { // allow testing wind drift following only
         // still in thermal - need to update the wp location and update the filter according to new measurement
         
         // now update filter
         float dx = get_offset_north(&prev_update_location, &current_loc);  // get distances from previous update
         float dy =  get_offset_east(&prev_update_location, &current_loc);
         
         Vector3f wind = ahrs.wind_estimate();
         // N.B. can use wind.x and wind.y to correct for thermal wind drift - need to ascertain sense.
         // Need to reconsider this: if thermal is defined relative to a/c and a/c is also drifting with wind is it necessary?
         //dx += wind.x * (millis()-prev_update_time)/1000.0;
         //dy += wind.y * (millis()-prev_update_time)/1000.0;
         float oldX = ekf.X[2];
         float oldY = ekf.X[3];
         ekf.update(climb_rate,dx, dy);                              // update the filter
         
         next_WP = current_loc; // as filter estimate is based on offset from current location
         location_offset(&next_WP, ekf.X[2], ekf.X[3]); //update the WP
         if (1) { //((dx > 0.0001) || (dy > 0.0001)) {
           //hal.console->printf_P(PSTR("%f %f %f %f %f %f\n"),oldX, oldY, dx, dy, ekf.X[2], ekf.X[3]);
           hal.console->printf_P(PSTR("%f %f %f\n"),climb_rate, ekf.X[2]+dx-oldX, ekf.X[3]+dy-oldY);
         }

         prev_update_location = current_loc;                                // save for next time
         prev_update_time = millis();
       }
     }
   }
   return calculated_control_mode;
 }
