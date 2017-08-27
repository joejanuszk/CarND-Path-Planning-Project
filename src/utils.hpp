#include <math.h>
#include <iostream>
#include "spline.h"

using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

double distance(double x1, double y1, double x2, double y2) {
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// In seconds
constexpr double timestep() { return 0.02; }

// In meters / second^2
constexpr double max_safe_acceleration() { return 9.81; }

// 1 m/s in mph
const double METERS_PER_SEC_IN_MPH = 2.23694;

double ms2mph(double x) { return x * METERS_PER_SEC_IN_MPH; }
double mph2ms(double x) { return x / METERS_PER_SEC_IN_MPH; }

const double SPEED_LIMIT_MPH = 50;
const double SPEED_LIMIT_MS = mph2ms(SPEED_LIMIT_MPH);

struct CarState {
    double x;
    double y;
    double v;
    double yaw;
};

double getDisplacementInTime(double v, double a, double t) {
    return (v * t) + (a * t * t);
}

void printCarState(const CarState &cs) {
    cout << "CarState:\n";
    cout << "\tx: " << cs.x << "\n";
    cout << "\ty: " << cs.y << "\n";
    cout << "\tv: " << cs.v << "\n";
    cout << "\tyaw: " << cs.yaw << "\n";
}

CarState getNextMaxSafeForwardCarState(const CarState &cs) {
    double t = timestep();
    double a = max_safe_acceleration();
    double yaw_r = deg2rad(cs.yaw);

    double x_frac = cos(yaw_r);
    double y_frac = sin(yaw_r);

    double dxy = getDisplacementInTime(cs.v, a, t);
    double v2 = dxy / t;
    if (v2 >= SPEED_LIMIT_MS) {
        v2 = cs.v;
        dxy = getDisplacementInTime(v2, 0, t);
    }

    double x2 = cs.x + dxy * x_frac;
    double y2 = cs.y + dxy * y_frac;

    return { x2, y2, v2, cs.yaw };
}

vector<double> getCarXYPointFromGlobalXYPoint(double car_x, double car_y, double yaw, double global_x, double global_y) {
    const double yaw_r = deg2rad(yaw);
    // Translate global point by car's position
    const double xg_trans_c = global_x - car_x;
    const double yg_trans_c = global_y - car_y;
    // Perform rotation to finish mapping
    // from global coords to car coords
    const double x = +xg_trans_c * cos(yaw_r) + yg_trans_c * sin(yaw_r);
    const double y = -xg_trans_c * sin(yaw_r) + yg_trans_c * cos(yaw_r);
    return { x, y };
}

int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y) {
    double closestLen = 100000; //large number
    int closestWaypoint = 0;

    for(int i = 0; i < maps_x.size(); i++) {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = distance(x, y, map_x, map_y);
        if (dist < closestLen) {
            closestLen = dist;
            closestWaypoint = i;
        }
    }

    return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y) {
    int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];

    double heading = atan2((map_y - y), (map_x - x));

    double angle = abs(theta - heading);

    if (angle > pi() / 4) {
        closestWaypoint++;
    }

    return closestWaypoint;
}

int getWaypointPrior(int waypoint, int total_waypoints) {
    return (waypoint - 1) % total_waypoints;
}

int getWaypointAfter(int waypoint, int total_waypoints) {
    return (waypoint + 1) % total_waypoints;
}

tk::spline getSplineFromNearbyWaypoints(
    double x,
    double y,
    double yaw,
    const vector<double> &maps_x,
    const vector<double> &maps_y) {
    int total_waypoints = maps_x.size();
    int curr_waypoint = NextWaypoint(x, y, yaw, maps_x, maps_y);
    int prev_waypoint = getWaypointPrior(curr_waypoint, total_waypoints);
    int next_waypoint = getWaypointAfter(curr_waypoint, total_waypoints);

    int spline_waypoints[] = { prev_waypoint, curr_waypoint, next_waypoint };
    const int total_spline_points = sizeof(spline_waypoints);

    vector<double> car_waypoints_x;
    vector<double> car_waypoints_y;
    for (int i = 0; i < total_spline_points; ++i) {
        vector<double> car_waypoints_xy = getCarXYPointFromGlobalXYPoint(
            x,
            y,
            yaw,
            maps_x[i],
            maps_y[i]);
        car_waypoints_x.push_back(car_waypoints_xy[0]);
        car_waypoints_y.push_back(car_waypoints_xy[1]);
    }

    tk::spline waypoints_spline;
    waypoints_spline.set_points(car_waypoints_x, car_waypoints_y);

    return waypoints_spline;
}