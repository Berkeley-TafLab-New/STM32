#include "rudder_control.h"
#include "math.h"

// Rudder control variables
static TIM_HandleTypeDef *rudder_htim = NULL;
static uint32_t rudder_channel = 0;

static float rudder_straight = 90.0;
static float rudder_range = 40.0;

static float rudder_current_angle = 90.0;
static float rudder_target_angle = 90.0;

// Initialize rudder control
void rudder_init(TIM_HandleTypeDef *htim, uint32_t channel) {
    rudder_htim = htim;
    rudder_channel = channel;
}

// Calculate bearing between two GPS coordinates
float rudder_calculate_bearing(float lat1, float lon1, float lat2, float lon2) {
    float dLon = (lon2 - lon1) * M_PI / 180.0f;

    float y = sinf(dLon) * cosf(lat2 * M_PI / 180.0f);

    float x = cosf(lat1 * M_PI / 180.0f) * cosf(lat2 * M_PI / 180.0f) - sinf(lat1 * M_PI / 180.0f) * sinf(lat2 * M_PI / 180.0f) * cosf(dLon);

    float bearing = atan2f(y, x) * 180.0f / M_PI;

    if (bearing < 0) 
        bearing += 360.0f;

    return bearing;
}

// Adjust rudder based on current vs target bearing
void rudder_adjust_from_gps(float curr_lat, float curr_lon, float target_lat, float target_lon, float current_heading) {

    float target_bearing = rudder_calculate_bearing(curr_lat, curr_lon, target_lat, target_lon);
    float diff = target_bearing - current_heading;

    // Normalize -180 to 180
    if (diff > 180) 
        diff -= 360;

    if (diff < -180) 
        diff += 360;

    // Calculate power (rudder angle)
    float power = sqrtf(fabsf(diff) / 180.0f);
    if (diff > 0) {
        rudder_target_angle = rudder_straight + power * rudder_range;

    } 
    else if (diff < 0) {
        rudder_target_angle = rudder_straight - power * rudder_range;
    } 
    else {
        rudder_target_angle = rudder_straight;
    }
}

// Smoothly move rudder toward target
void rudder_move_to(void) {

    if (fabsf(rudder_target_angle - rudder_current_angle) > 1) {

        if (rudder_current_angle < rudder_target_angle) {

            rudder_current_angle += 1;

        } 
        else {
            rudder_current_angle -= 1;
        }

        uint32_t pulse_width = 500 + (rudder_current_angle * (2000.0f / 180.0f));
        __HAL_TIM_SET_COMPARE(rudder_htim, rudder_channel, pulse_width);
    }
}
