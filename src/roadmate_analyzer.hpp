#include <math.h>
#include <vector>
#include "utils.hpp"

#ifndef CARND_PATH_PLANNING_ROADMATE_ANALYZER
#define CARND_PATH_PLANNING_ROADMATE_ANALYZER

using namespace std;

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
            double speed = sqrt(vx * vx + vy * vy);
            double s = roadmate_data[5] + path_size * timestep() * speed;
            double d = roadmate_data[6];
            double s_diff = s - car_s;
            // handle track wraparound while allowing a safe amount of
            // negative value; i.e. do not treat as "pure" modular math
            if (s_diff < -0.5 * TRACK_LENGTH_M) {
                s_diff += TRACK_LENGTH_M;
            }
            if (s_diff > 0.5 * TRACK_LENGTH_M) {
                s_diff -= TRACK_LENGTH_M;
            }
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

    double getLeftSDiff() {
        if (hasLeftRoadmate()) {
            return left_roadmate->getSDiff();
        }
        return large_s_value();
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

    double getRightSDiff() {
        if (hasRightRoadmate()) {
            return right_roadmate->getSDiff();
        }
        return large_s_value();
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

#endif
