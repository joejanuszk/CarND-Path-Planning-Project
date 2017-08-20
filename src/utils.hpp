#include <math.h>
#include <iostream>

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
