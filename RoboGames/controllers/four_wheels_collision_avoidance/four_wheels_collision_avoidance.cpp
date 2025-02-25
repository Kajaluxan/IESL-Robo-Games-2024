#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/GPS.hpp>
#include <webots/Camera.hpp>
#include <queue>
#include <stack>
#include <vector>
#include <cmath>

#define TIME_STEP 32
#define MAX_SPEED 6.28

using namespace webots;

struct Position {
    double x, y;
};

class MazeRobot {
private:
    Robot *robot;
    Motor *wheel1, *wheel2, *wheel3, *wheel4;
    DistanceSensor *ds_front, *ds_left, *ds_right, *ds_back;
    GPS *gps;
    Camera *camera;
    Position startPos;
    bool foundGreenWall;

public:
    MazeRobot() {
        robot = new Robot();
        wheel1 = robot->getMotor("wheel1");
        wheel2 = robot->getMotor("wheel2");
        wheel3 = robot->getMotor("wheel3");
        wheel4 = robot->getMotor("wheel4");
        
        wheel1->setPosition(INFINITY);
        wheel2->setPosition(INFINITY);
        wheel3->setPosition(INFINITY);
        wheel4->setPosition(INFINITY);
        
        ds_front = robot->getDistanceSensor("ds_front");
        ds_left = robot->getDistanceSensor("ds_left");
        ds_right = robot->getDistanceSensor("ds_right");
        ds_back = robot->getDistanceSensor("ds_back");
        
        ds_front->enable(TIME_STEP);
        ds_left->enable(TIME_STEP);
        ds_right->enable(TIME_STEP);
        ds_back->enable(TIME_STEP);
        
        gps = robot->getGPS("gps");
        gps->enable(TIME_STEP);
        
        camera = robot->getCamera("camera");
        camera->enable(TIME_STEP);
        
        startPos = {gps->getValues()[0], gps->getValues()[1]};
        foundGreenWall = false;
    }
    
    bool detectGreenWall() {
        const unsigned char *image = camera->getImage();
        int width = camera->getWidth();
        int height = camera->getHeight();
        
        for (int i = 0; i < width; ++i) {
            for (int j = 0; j < height; ++j) {
                int r = camera->imageGetRed(image, width, i, j);
                int g = camera->imageGetGreen(image, width, i, j);
                int b = camera->imageGetBlue(image, width, i, j);
                
                if (g > r * 1.5 && g > b * 1.5) {
                    return true;
                }
            }
        }
        return false;
    }
    
    void move(double leftSpeed, double rightSpeed) {
        wheel1->setVelocity(leftSpeed);
        wheel2->setVelocity(rightSpeed);
        wheel3->setVelocity(leftSpeed);
        wheel4->setVelocity(rightSpeed);
    }
    
    void navigateMaze() {
        std::stack<Position> path;
        path.push(startPos);
        
        while (robot->step(TIME_STEP) != -1) {
            double left_value = ds_left->getValue();
            double right_value = ds_right->getValue();
            double front_value = ds_front->getValue();
            double back_value = ds_back->getValue();
            
            if (detectGreenWall()) {
                foundGreenWall = true;
                break;
            }
            
            if (front_value > 800) {
                move(MAX_SPEED, MAX_SPEED);
            } else if (left_value > 800) {
                move(-MAX_SPEED / 2, MAX_SPEED);
            } else if (right_value > 800) {
                move(MAX_SPEED, -MAX_SPEED / 2);
            } else {
                move(-MAX_SPEED / 2, -MAX_SPEED / 2);
            }
            
            path.push({gps->getValues()[0], gps->getValues()[1]});
        }
        
        if (foundGreenWall) {
            returnToStart(path);
        }
    }
    
    void returnToStart(std::stack<Position> &path) {
        while (!path.empty() && robot->step(TIME_STEP) != -1) {
            Position pos = path.top();
            path.pop();
            
            if (std::hypot(pos.x - startPos.x, pos.y - startPos.y) < 0.05) {
                move(0, 0);
                break;
            }
            move(MAX_SPEED, MAX_SPEED);
        }
    }
    
    ~MazeRobot() {
        delete robot;
    }
};

int main() {
    MazeRobot robot;
    robot.navigateMaze();
    return 0;
}