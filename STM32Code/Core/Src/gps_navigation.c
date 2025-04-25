#include "gps_navigation.h"
#include <math.h>

#define DEG_TO_RAD 0.0174533
#define RAD_TO_DEG 57.2958

// Calculate bearing from current (lat1, lon1) to target (lat2, lon2)
float calculate_bearing(float lat1, float lon1, float lat2, float lon2) {
    float dLon = (lon2 - lon1) * DEG_TO_RAD;
    lat1 *= DEG_TO_RAD;
    lat2 *= DEG_TO_RAD;

    float y = sinf(dLon) * cosf(lat2);
    float x = cosf(lat1) * sinf(lat2) - sinf(lat1) * cosf(lat2) * cosf(dLon);
    float bearing = atan2f(y, x) * RAD_TO_DEG;

    // Normalize to [0, 360)
    if (bearing < 0) bearing += 360.0f;

    return bearing;
}

// Compute the smallest angle difference (-180 to 180)
float calculate_heading_error(float target_bearing, float current_heading) {
    float error = target_bearing - current_heading;
    if (error > 180) error -= 360;
    if (error < -180) error += 360;
    return error;
}
