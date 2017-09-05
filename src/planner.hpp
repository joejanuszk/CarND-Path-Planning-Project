#include <math.h>

#ifndef CARND_PATH_PLANNING_PLANNER
#define CARND_PATH_PLANNING_PLANNER

using namespace std;

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

    bool isChangingLanes() {
        return getGoalLane() != getCurrentLane() && !isCenteredInLane();
    }

    static double getLaneCenterD(int lane) {
        return (lane + 0.5) * lane_width();
    }

    static double getLaneForD(double d) {
        return static_cast<int>(floor(d / lane_width())) % static_cast<int>(lane_width());
    }
};

#endif
