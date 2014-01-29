Arduplane 2.68 with Extended Kalman Filter for thermal centring
Sam Tabor, based on work by Peter Braswell

This code uses an Extended Kalman Filter to search for the centre of a thermal. The thermal model and state 
vector are the same as those detailed in this paper http://enu.kz/repository/2010/AIAA-2010-179.pdf. When in 
auto mode, the aircraft climb rate is checked and, if it exceeds a set value, the EKF is initialised. Inputs to 
the EKF are netto climb rate (i.e. measured climb rate plus the calculated aircraft sink at the current 
airspeed and bank) and aircraft movements since the last update. So far this filter has been tested in 
FlightGear's thermal demo successfully. See 
http://diydrones.ning.com/forum/topics/autonomous-soaring?id=705844%3ATopic%3A1105947&page=2#comments . 
The filter prints info to the console that can be used to rerun the filter in Matlab and experiment with 
different filter parameters.