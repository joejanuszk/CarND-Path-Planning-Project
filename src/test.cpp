#include <fstream>
#include <iostream>
#include "json.hpp"
#include "utils.hpp"

using namespace std;

int main() {
    CarStateXY cs_xy_loop = { 0, 0, 0, 0 };
    //printCarStateXY(cs_xy_loop);
    for (int i = 0; i < 50; ++i) {
        cs_xy_loop = getNextMaxSafeForwardCarStateXY(cs_xy_loop);
        //printCarStateXY(cs_xy_loop);
    }

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;

    ifstream in_map_(map_file_.c_str(), ifstream::in);

    string line;
    while (getline(in_map_, line)) {
        istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        map_waypoints_x.push_back(x);
        map_waypoints_y.push_back(y);
        map_waypoints_s.push_back(s);
        map_waypoints_dx.push_back(d_x);
        map_waypoints_dy.push_back(d_y);
    }
    /*

    //double car_x = 909.48;
    //double car_y = 1128.67;
    double car_x = 1445.01;
    double car_y = 6243.72;

    CarStateXY cs_xy = { car_x, car_y, 0, 0 };
    printCarStateXY(cs_xy);
    CarStateFrenet cs_fr = getCarStateFrenet(cs_xy, map_waypoints_x, map_waypoints_y);
    cs_xy = getCarStateXY(cs_fr, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    printCarStateXY(cs_xy);
    */
    double car_x = 909.48;
    double car_y = 1128.67;
    tk::spline waypoints_spline = getSplineFromNearbyWaypoints(car_x, car_y, 0, map_waypoints_x, map_waypoints_y);
    cout << waypoints_spline(10) << endl;

    return 0;
}
