#ifndef PLANNER_CONSTANTS_HPP
#define PLANNER_CONSTANTS_HPP

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

const double H_VALUE_WEIGHT = 1.0;
const double G_VALUE_WEIGHT = 1.0;

const int ROBOT_FOOTPRINT_RADIUS = 6;
const int ROBOT_FOOTPRINT_RADIUS_WITH_PALLET = 7;
const int PALLET_FOOTPRINT_RADIUS = 3;
const int ROBOT_OFFSET = 1;

const double STRAIGHT_TIME_STEP = 0.16;
const double TURN_TIME_STEP = 1.4;

const bool ALLOW_IN_PLACE_TURN = false;

const int LEAD_IN_LENGTH_CM = 5;

const double TURN_COST = 14.0;
const double STRAIGHT_COST = 0.1;
const double IN_PLACE_TURN_COST = 0.5;
const double IN_PLACE_TURN_TIME_STEP = 0.5;
const double REVERSE_COST_PENALTY = 20.0;
const double IN_PLACE_TURN_COST_PENALTY = 8.0;

const double NO_TURN_THRESH = 15.0;
const double PALLET_WAIT_TIME = 2.5;
const double PALLET_RUNWAY_SLOWDOWN_FACTOR = 1.8;

const double GOAL_THETA_TOLERANCE = M_PI / 8.0;

const bool USE_HEURISTIC = true;

const bool DEBUG = false;

const int NUM_SAMPLED_STARTS = 10;
const double SEARCH_RADIUS_CM = 3.0;
const double MAX_SEARCH_TIME = 10.0;

#endif
