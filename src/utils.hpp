#include <math.h>
#include <iostream>
#include "coord_utils.hpp"
#include "spline.h"

using namespace std;

// In seconds
constexpr double timestep() { return 0.02; }

// In meters / second^2
constexpr double max_safe_acceleration() { return 9.81 - 2; }

// 1 m/s in mph
const double METERS_PER_SEC_IN_MPH = 2.23694;

// lane width in meters
constexpr double lane_width() { return 4; }

// meters
constexpr double too_close_distance() { return 15; }

// meters
constexpr double large_s_value() { return 100000; }

double ms2mph(double x) { return x * METERS_PER_SEC_IN_MPH; }
double mph2ms(double x) { return x / METERS_PER_SEC_IN_MPH; }

const double TRACK_LENGTH_M = 6945.554;
const double SPEED_LIMIT_BUFFER_MPH = 5;
const double SPEED_LIMIT_MPH = 50 - SPEED_LIMIT_BUFFER_MPH;
const double SPEED_LIMIT_MS = mph2ms(SPEED_LIMIT_MPH);

double getDisplacementInTime(double v, double a, double t) {
    return (v * t) + (a * t * t);
}

double getNextAcceleratedSpeed(double v1, double roadmate_speed, bool is_lane_roadmate_too_close, bool is_changing_lanes) {
    double t = timestep();
    // limit forward acceleration while changing lanes
    // to reduce risk of exceeding max allowed value
    double safe_acceleration_coefficient = is_changing_lanes ? 0.3 : 0.8;
    double a = safe_acceleration_coefficient * max_safe_acceleration();
    double speed_limit = is_lane_roadmate_too_close?
            min(roadmate_speed, SPEED_LIMIT_MS):
            SPEED_LIMIT_MS;
    double dir = speed_limit > v1 ? 1 : -1;

    double disp = getDisplacementInTime(v1, dir * a, t);
    double v2 = disp / t;
    if ((dir > 0 && v2 >= speed_limit) || (dir < 0 && v2 <= speed_limit)) {
        v2 = v1;
    }

    return v2;
}

vector<double> getCarXYPointFromGlobalXYPoint(double car_x, double car_y, double yaw, double global_x, double global_y) {
    const double yaw_r = deg2rad(yaw);
    // Translate global point by car's position
    const double xg_trans_c = global_x - car_x;
    const double yg_trans_c = global_y - car_y;
    // Perform rotation to finish mapping
    // from global coords to car coords
    const double x = xg_trans_c * cos(0 - yaw_r) - yg_trans_c * sin(0 - yaw_r);
    const double y = xg_trans_c * sin(0 - yaw_r) + yg_trans_c * cos(0 - yaw_r);
    return { x, y };
}

vector<double> getGlobalXYPointFromCarXYPoint(
        double car_x,
        double car_y,
        double yaw,
        double x,
        double y) {
    const double yaw_r = deg2rad(yaw);
    // Rotate from car coords into global coords
    const double xg_trans_c = x * cos(yaw_r - 0) - y * sin(yaw_r - 0);
    const double yg_trans_c = x * sin(yaw_r - 0) + y * cos(yaw_r - 0);

    const double global_x = xg_trans_c + car_x;
    const double global_y = yg_trans_c + car_y;
    return { global_x, global_y };
}

bool isCarInLane(int lane, int d) {
    double lane_start = lane * lane_width();
    double lane_end = (lane + 1) * lane_width();
    return lane_start <= d && d <= lane_end;
}
