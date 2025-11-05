/**
 * @file configuration.hpp
 * @brief a general file to configure the project
 * @author PSAF
 * @date 2022-06-01
 */
#ifndef PSAF_CONFIGURATION__CONFIGURATION_HPP_
#define PSAF_CONFIGURATION__CONFIGURATION_HPP_

#include <string>
#include <vector>

/**
 * To enable additional printouts
 */
#define DEBUG false

/**
 * To force some tests to pass. This can be helpful during development. Turn this
 * off for the finished code
 */
#define FORCE_TEST_PASS true

/**
* Define the uc_bridge topic names
* Note: The PSAF 1 cars usually only have 3 ultrasonic sensors.
*/
#define SET_SPEED_FORWARD_TOPIC "uc_bridge/set_motor_level_forward"
#define SET_SPEED_BACKWARD_TOPIC  "uc_bridge/set_motor_level_backward"
#define SET_STEERING_TOPIC  "uc_bridge/set_steering"

#define NBR_OF_US_SENSORS  3
#define US_TOPICS {"uc_bridge/us_front_center", "uc_bridge/us_mid_right", "uc_bridge/us_mid_left"}

#define GET_PUSHBUTTONS_TOPIC "/pb_data"

#endif  // PSAF_CONFIGURATION__CONFIGURATION_HPP_
