/*
 * rudder_control.c
 *
 * Redefined rudder control to match servo_controls style
 * Author: Adapted for consistency by ChatGPT
 */

 #include "rudder_control.h"
 #include "servo_controls.h"
 #include "math.h"
 
 static TIM_HandleTypeDef *rudder_htim = NULL;
 static uint32_t rudder_channel = 0;
 
 static float rudder_target_angle = 90.0f;
 static const float rudder_straight = 90.0f;
 static const float rudder_range = 45.0f;
 
 void rudder_init(TIM_HandleTypeDef *htim, uint32_t channel) {
     rudder_htim = htim;
     rudder_channel = channel;
     rudder_target_angle = rudder_straight;
     set_servo_angle(rudder_htim, rudder_channel, rudder_target_angle);
 }
 
 void rudder_set_target_angle(float angle) {
     if (angle < (rudder_straight - rudder_range)) {
         angle = rudder_straight - rudder_range;
     } else if (angle > (rudder_straight + rudder_range)) {
         angle = rudder_straight + rudder_range;
     }
     rudder_target_angle = angle;
     set_servo_angle(rudder_htim, rudder_channel, rudder_target_angle);
 }
 
 void rudder_update(void) {
     set_servo_angle(rudder_htim, rudder_channel, rudder_target_angle);
 }


 float rudder_get_target_angle(void) {
     return rudder_target_angle;
 }
 
 float rudder_get_straight(void) {
     return rudder_straight;
 }
 
 float rudder_get_range(void) {
     return rudder_range;
 }
