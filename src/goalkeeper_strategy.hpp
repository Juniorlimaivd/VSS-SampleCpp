#ifndef GOALLKEEPER_STRATEGY_H
#define GOALLKEEPER_STRATEGY_H

#include <Communications/StateReceiver.h>
#include <Communications/CommandSender.h>
#include <math.h>
#include "utils.hpp"

#define doublePair std::pair <double, double>

class GoalKeeperStrategy{
    private:
        vss::TeamType _teamType;

        double _lastAngleError;
    public:

        GoalKeeperStrategy(vss::TeamType type) {
            this->_teamType = type;
        }

        vss::WheelsCommand update(vss::State state, int index) {
            doublePair objective = defineObjective(state, index);
            return updateMotion(state, objective, index);
        }

        doublePair defineObjective(vss::State, int index) {
            return doublePair (10., 65.0);
        }

        vss::WheelsCommand updateMotion(vss::State state, doublePair objective, int index) 
        {
            vss::WheelsCommand result;
            bool reversed = false;
            vss::Robot robot = (this->_teamType == vss::TeamType::Blue) ? state.teamBlue[index] : state.teamYellow[index];

            robot.angle = Utils::to180range(robot.angle * M_PI / 180.0);

            robot.y = 130 - robot.y;

            double obj_dir = std::atan2(robot.y - objective.second, objective.first - robot.x);

            double angle_error = Utils::smallestAngleDiff(robot.angle, obj_dir);

            // if (fabs(angle_error) > M_PI/2. + M_PI/20.) {
            //     reversed = true;
            //     robot.angle = Utils::to180range(robot.angle + M_PI);

            //     angle_error = Utils::smallestAngleDiff(robot.angle, obj_dir);
            // }  

            double kp = 5, kd = 10;

            std::cout << "angle_error: " << angle_error << std::endl; 

            double motorSpeed = kp*angle_error + kd*(angle_error - this->_lastAngleError);
            this->_lastAngleError = angle_error;

            double baseSpeed = 10 * Utils::distance(doublePair(robot.x, robot.y), objective);

            baseSpeed = Utils::bound(baseSpeed, 0, 45);

            motorSpeed = Utils::bound(motorSpeed, -baseSpeed, baseSpeed);

            if (fabs(motorSpeed) <= 8) motorSpeed = 0; 

            if (baseSpeed <= 20.) baseSpeed = 0;

            reversed = true;

            if (reversed) {
                if (motorSpeed < 0) {
                    result.leftVel = -baseSpeed + motorSpeed;
                    result.rightVel = -baseSpeed - motorSpeed;
                } else {
                    result.leftVel = -baseSpeed + motorSpeed;
                    result.rightVel = -baseSpeed - motorSpeed;
                }
            } else {
                if (motorSpeed < 0)
                {
                    result.leftVel = baseSpeed + motorSpeed;
                    result.rightVel = baseSpeed;
                }
                else
                {
                    result.leftVel = baseSpeed;
                    result.rightVel = baseSpeed - motorSpeed;
                }
            }
            std::cout << result.leftVel << "," << result.rightVel << std::endl;
            return result;
            
        }

};

#endif