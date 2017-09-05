#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "utils.hpp"

using namespace std;

// for convenience
using json = nlohmann::json;

// lane width in meters
constexpr double lane_width() { return 4; }

// meters
constexpr double too_close_distance() { return 20; }

// meters
constexpr double large_s_value() { return 100000; }

bool isCarInLane(int lane, int d) {
    double lane_start = lane * lane_width();
    double lane_end = (lane + 1) * lane_width();
    return lane_start <= d && d <= lane_end;
}

class Planner {
private:
    double m_car_d;
    int m_goal_lane;
    bool m_has_set_d;

public:
    Planner() : m_has_set_d(false) {}

    void setCurrentD(double car_d) {
        m_car_d = car_d;
        if (!m_has_set_d) {
            setGoalLane(getCurrentLane());
            m_has_set_d = true;
        }
    }

    void setGoalLane(int goal) {
        m_goal_lane = goal;
    }

    int getGoalLane() {
        return m_goal_lane;
    }

    double getGoalLaneCenterD() {
        return getLaneCenterD(m_goal_lane);
    }

    int getCurrentLane() {
        return getLaneForD(m_car_d);
    }

    // define "centered" as no more than some well-chosen offset from center
    bool isCenteredInLane() {
        double lane_center_d = getLaneCenterD(getCurrentLane());
        return abs(lane_center_d - m_car_d) < 0.7;
    }

    static double getLaneCenterD(int lane) {
        return (lane + 0.5) * lane_width();
    }

    static double getLaneForD(double d) {
        return static_cast<int>(floor(d / lane_width())) % static_cast<int>(lane_width());
    }
};

class Roadmate {
private:
    int m_id;
    double m_s_diff;
    double m_speed;

public:
    Roadmate(int id, double s_diff, double speed) : m_id(id), m_s_diff(s_diff), m_speed(speed) {}

    int getID() { return m_id; }
    double getSDiff() { return m_s_diff; }
    double getSpeed() { return m_speed; }
};

class RoadmateAnalyzer {
private:
    Roadmate *lane_roadmate;
    Roadmate *left_roadmate;
    Roadmate *right_roadmate;
    bool is_left_change_safe;
    bool is_right_change_safe;

public:
    RoadmateAnalyzer(
            const vector<vector<double>>& sensor_fusion,
            int curr_lane,
            double car_s,
            int path_size) :
            is_left_change_safe(true),
            is_right_change_safe(true),
            lane_roadmate(nullptr),
            left_roadmate(nullptr),
            right_roadmate(nullptr) {
        double lane_s_diff = large_s_value();
        double left_s_diff = large_s_value();
        double right_s_diff = large_s_value();

        for (int i = 0; i < sensor_fusion.size(); ++i) {
            vector<double> roadmate_data = sensor_fusion[i];
            int id = static_cast<int>(roadmate_data[0]);
            double vx = roadmate_data[3];
            double vy = roadmate_data[4];
            double s = roadmate_data[5];
            double d = roadmate_data[6];
            double speed = sqrt(vx * vx + vy * vy);
            double s_diff = s + path_size * timestep() * speed - car_s;
            bool is_ahead = s_diff > 0;

            bool is_rm_in_curr_lane = isCarInLane(curr_lane, d);
            if (is_rm_in_curr_lane && is_ahead && s_diff < lane_s_diff) {
                delete lane_roadmate;
                lane_roadmate = new Roadmate(id, s_diff, speed);
                lane_s_diff = s_diff;
            }

            bool is_rm_in_left_lane = isCarInLane(curr_lane - 1, d);
            if (is_rm_in_left_lane && is_ahead && s_diff < left_s_diff) {
                delete left_roadmate;
                left_roadmate = new Roadmate(id, s_diff, speed);
                left_s_diff = s_diff;
            }
            if (is_rm_in_left_lane && !is_ahead) {
                if (abs(s_diff) - too_close_distance() < 0) {
                    is_left_change_safe = false;
                }
            }

            bool is_rm_in_right_lane = isCarInLane(curr_lane + 1, d);
            if (is_rm_in_right_lane && is_ahead && s_diff < right_s_diff) {
                delete right_roadmate;
                right_roadmate = new Roadmate(id, s_diff, speed);
                right_s_diff = s_diff;
            }
            if (is_rm_in_right_lane && !is_ahead) {
                if (abs(s_diff) - too_close_distance() < 0) {
                    is_right_change_safe = false;
                }
            }
        }
    }

    ~RoadmateAnalyzer() {
        delete lane_roadmate;
        delete left_roadmate;
        delete right_roadmate;
    }

    bool hasLaneRoadmate() {
        return lane_roadmate != nullptr;
    }

    double getLaneRoadmateSpeed() {
        return hasLaneRoadmate() ? lane_roadmate->getSpeed() : 0;
    }

    bool isLaneRoadmateTooClose() {
        return hasLaneRoadmate() && lane_roadmate->getSDiff() < too_close_distance();
    }

    bool hasLeftRoadmate() {
        return left_roadmate != nullptr;
    }

    double getLeftRoadmateSpeed() {
        return hasLeftRoadmate() ? left_roadmate->getSpeed() : 0;
    }

    bool isLeftRoadmateTooClose() {
        return hasLeftRoadmate() && left_roadmate->getSDiff() < too_close_distance();
    }

    bool isLeftRoadmateReasonablyFarAhead() {
        if (hasLeftRoadmate()) {
            return left_roadmate->getSDiff() > 2 * too_close_distance();
        }
        return true;
    }

    bool hasRightRoadmate() {
        return right_roadmate != nullptr;
    }

    double getRightRoadmateSpeed() {
        return hasRightRoadmate() ? right_roadmate->getSpeed() : 0;
    }

    bool isRightRoadmateTooClose() {
        return hasRightRoadmate() && right_roadmate->getSDiff() < too_close_distance();
    }

    bool isRightRoadmateReasonablyFarAhead() {
        if (hasRightRoadmate()) {
            return right_roadmate->getSDiff() > 2 * too_close_distance();
        }
        return true;
    }

    bool isSafeToChangeLeft() {
        return is_left_change_safe;
    }

    bool isSafeToChangeRight() {
        return is_right_change_safe;
    }
};

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

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

  Planner planner;

  h.onMessage([
    &map_waypoints_x,
    &map_waypoints_y,
    &map_waypoints_s,
    &map_waypoints_dx,
    &map_waypoints_dy,
    &planner
  ](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

            double path_size = previous_path_x.size();
            double pos_x;
            double pos_y;
            double yaw;
            double v;

            for (int i = 0; i < path_size; ++i) {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }

            vector<double> spline_pts_x;
            vector<double> spline_pts_y;
            if (path_size < 2) {
                double prev_car_x = car_x - cos(deg2rad(car_yaw));
                double prev_car_y = car_y - sin(deg2rad(car_yaw));
                pos_x = car_x;
                pos_y = car_y;
                yaw = car_yaw;
                v = car_speed;

                spline_pts_x.push_back(prev_car_x);
                spline_pts_x.push_back(pos_x);
                spline_pts_y.push_back(prev_car_y);
                spline_pts_y.push_back(pos_y);
            }
            else {
                pos_x = previous_path_x[path_size - 1];
                pos_y = previous_path_y[path_size - 1];
                double pos_x2 = previous_path_x[path_size - 2];
                double pos_y2 = previous_path_y[path_size - 2];
                double prev_dist = distance(pos_x2, pos_y2, pos_x, pos_y);
                yaw = rad2deg(atan2(pos_y - pos_y2, pos_x - pos_x2));
                v = prev_dist / timestep();

                spline_pts_x.push_back(pos_x2);
                spline_pts_x.push_back(pos_x);
                spline_pts_y.push_back(pos_y2);
                spline_pts_y.push_back(pos_y);

                car_s = end_path_s;
                car_d = end_path_d;
            }
            planner.setCurrentD(car_d);

            RoadmateAnalyzer ra(sensor_fusion, planner.getCurrentLane(), car_s, path_size);
            double roadmate_speed = ra.getLaneRoadmateSpeed();

            if (ra.isLaneRoadmateTooClose() && planner.isCenteredInLane()) {
                int curr_lane = planner.getCurrentLane();
                if (curr_lane == 1) {
                    double lane_rm_speed = ra.getLaneRoadmateSpeed();
                    double left_rm_speed = ra.getLeftRoadmateSpeed();
                    double right_rm_speed = ra.getRightRoadmateSpeed();
                    bool isWorthChangingLeft = (left_rm_speed > lane_rm_speed && !ra.isLeftRoadmateTooClose()) || ra.isLeftRoadmateReasonablyFarAhead();
                    bool isWorthChangingRight = (right_rm_speed > lane_rm_speed && !ra.isRightRoadmateTooClose()) || ra.isRightRoadmateReasonablyFarAhead();
                    if (isWorthChangingLeft && ra.isSafeToChangeLeft()) {
                        planner.setGoalLane(0);
                    }
                    else if (isWorthChangingRight && ra.isSafeToChangeRight()) {
                        planner.setGoalLane(2);
                    }
                }
                if (curr_lane == 0) {
                    double lane_rm_speed = ra.getLaneRoadmateSpeed();
                    double right_rm_speed = ra.getRightRoadmateSpeed();
                    bool isWorthChangingRight = (right_rm_speed > lane_rm_speed && !ra.isRightRoadmateTooClose()) || ra.isRightRoadmateReasonablyFarAhead();
                    if (isWorthChangingRight && ra.isSafeToChangeRight()) {
                        planner.setGoalLane(1);
                    }
                }
                if (curr_lane == 2) {
                    double lane_rm_speed = ra.getLaneRoadmateSpeed();
                    double left_rm_speed = ra.getLeftRoadmateSpeed();
                    bool isWorthChangingLeft = (left_rm_speed > lane_rm_speed && !ra.isLeftRoadmateTooClose()) || ra.isLeftRoadmateReasonablyFarAhead();
                    if (isWorthChangingLeft && ra.isSafeToChangeLeft()) {
                        planner.setGoalLane(1);
                    }
                }
            }

            for (int i = 0; i < 3; ++i) {
                vector<double> next_wp = getXY(
                        car_s + (i + 1) * 30,
                        planner.getGoalLaneCenterD(),
                        map_waypoints_s,
                        map_waypoints_x,
                        map_waypoints_y);
                spline_pts_x.push_back(next_wp[0]);
                spline_pts_y.push_back(next_wp[1]);
            }
            for (int i = 0; i < spline_pts_x.size(); ++i) {
                vector<double> car_ref_frame_xy = getCarXYPointFromGlobalXYPoint(
                        pos_x,
                        pos_y,
                        yaw,
                        spline_pts_x[i],
                        spline_pts_y[i]);
                spline_pts_x[i] = car_ref_frame_xy[0];
                spline_pts_y[i] = car_ref_frame_xy[1];
            }
            tk::spline waypoint_spline;
            waypoint_spline.set_points(spline_pts_x, spline_pts_y);
            double target_v = getNextAcceleratedSpeed(v, roadmate_speed, ra.isLaneRoadmateTooClose());
            double target_x = 30;
            double target_y = waypoint_spline(target_x);
            double target_dist = sqrt(target_x * target_x + target_y * target_y);

            double x_add_on = 0;
            double N = target_dist / (timestep() * target_v);

            for (int i = 0; i < 50 - path_size; ++i) {
                double x_point = x_add_on + target_x / N;
                double y_point = waypoint_spline(x_point);
                x_add_on = x_point;

                vector<double> global_ref_frame_xy = getGlobalXYPointFromCarXYPoint(
                    pos_x,
                    pos_y,
                    yaw,
                    x_point,
                    y_point);
                next_x_vals.push_back(global_ref_frame_xy[0]);
                next_y_vals.push_back(global_ref_frame_xy[1]);

                target_v = getNextAcceleratedSpeed(target_v, roadmate_speed, ra.isLaneRoadmateTooClose());
                N = target_dist / (timestep() * target_v);
            }

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
