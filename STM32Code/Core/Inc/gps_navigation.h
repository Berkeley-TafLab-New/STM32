#ifndef GPS_NAVIGATION_H
#define GPS_NAVIGATION_H

float calculate_bearing(float lat1, float lon1, float lat2, float lon2);
float calculate_heading_error(float target_bearing, float current_heading);

#endif