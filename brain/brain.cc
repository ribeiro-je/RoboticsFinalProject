#include <iostream>
#include <math.h>
#include <thread>
#include <algorithm>
#include <vector>
#include <stdio.h>
#include <stdlib.h>

#include "robot.hh"
#include "cam.hh"
#include "grid.hh"
#include "viz.hh"

#include <opencv2/imgproc.hpp>

using std::cout;
using std::endl;

// Globals //

// Have objects been found //
bool foundBlueCylinder = false;
bool foundOrangeCone = false;
bool foundRedBackPack = false;
bool foundBox = false;

// path and thread mutex //
std::vector<coords> path;
std::mutex mtx;

using namespace cv;

///////////////////////////////////////////////
//////////////// FUNCTIONS ////////////////////
///////////////////////////////////////////////

// check if image is empty
// refrence from https://docs.opencv.org/3.4/da/d5c/tutorial_canny_detector.html
bool isImageEmpty(cv::Mat image)
{
    // Check for failure
    if (image.empty())
    {
        cout << "Could not open or find the image" << endl;
        return false;
    }
    return true;
}

// find distance from top of photo to wall
std::string objectColor(cv::Mat image)
{
    //Vec3b -> B G R
    Vec3b pixelColor = image.at<Vec3b>(cv::Point(70, 50));
    int red = pixelColor.val[0];
    int green = pixelColor.val[1];
    int blue = pixelColor.val[2];
    std::cout << "blue : " << blue << " green " << green << " red: " << red << endl;

    if (red > 120 && blue < 80)
    {
        return "red";
    }
    if (red < 100 && blue > 170)
    {
        return "blue";
    }
    if (red > 100 && blue > 100)
    {
        return "orange";
    }
    if (red > 100 && blue < 100)
    {
        return "brown";
    }
    if (red > 220 && blue > 220 && green > 220)
    {
        return "white";
    }

    return "noObject";
}

/*
To view the camera image in time, you could press CTRL-T in Gazebo
, choosing the Topic-"~/tankbot0/tankbot/camera_sensor/link/camera/image", 
then a Image Window will pop up, in order to view the Image in time.
*/
void callback(Robot *robot)
{
    bool hitIsObject = false;
    cv::Mat fullMat;
    fullMat = robot->frame;
    if (!isImageEmpty(fullMat))
    {
        return;
    }
    Pose pose(robot->pos_x, robot->pos_y, robot->pos_t);

    // if within range of object beep three times when found
    if (robot->range <= 1.25)
    {
        std::string object = objectColor(fullMat);
        if (object == "red" && !foundRedBackPack)
        {
            grid_apply_hit_color(robot->range, 0, pose, object);
            std::cout << "BACKPACK FOUND! LED RED" << endl;
            system("echo -ne '\a'"); // beep!

            hitIsObject = true;
            foundRedBackPack = true;

            // stop - object found!:)
            robot->set_vel(0, 0);
            return;
        }
        if ((object == "orange" || object == "white") && !foundOrangeCone)
        {
            grid_apply_hit_color(robot->range, 0, pose, "orange");
            std::cout << "CONE FOUND! LED ORANGE" << endl;
            system("echo -ne '\a'"); // beep!

            hitIsObject = true;
            foundOrangeCone = true;

            // stop - object found!:)
            robot->set_vel(0, 0);
            return;
        }
        if (object == "brown" && !foundBox)
        {
            grid_apply_hit_color(robot->range, 0, pose, object);
            std::cout << "BOX FOUND! LED BROWN" << endl;
            system("echo -ne '\a'"); // beep!

            hitIsObject = true;
            foundBox = true;

            // stop - object found!:)
            robot->set_vel(0, 0);
            return;
        }
        if (object == "blue" && !foundBlueCylinder)
        {
            grid_apply_hit_color(robot->range, 0, pose, object);
            std::cout << "BUE CYLINDER FOUND! LED BLUE" << endl;
            system("echo -ne '\a'"); // beep!

            hitIsObject = true;
            foundBlueCylinder = true;

            // stop - object found!:)
            robot->set_vel(0, 0);
            return;
        }
    }

    // if within 1.0 meters add information to ogrid and it is not the object
    if (robot->range < 1.0 && robot->range > 0 && !hitIsObject)
    {
        cout << "LASER HIT " << robot->range << endl;
        grid_apply_hit(robot->range, 0, pose);
    }

    // path for exploring
    mtx.lock();
    std::vector<coords> pathcopy = path;
    mtx.unlock();

    Mat view = grid_view(pose, pathcopy);
    viz_show(view);

    float ang = grid_goal_angle(pose, pathcopy);
    float trn = clamp(-1.0, 3 * ang, 1.0);

    float fwd = clamp(0.0, robot->range, 2.0);

    // if all objects found done!
    if (foundBlueCylinder && foundBox && foundOrangeCone && foundRedBackPack)
    {
        robot->set_vel(0, 0);
        robot->done();
        return;
    }

    // following path logic
    if (abs(ang) > 2.5)
    {
        cout << "here 1" << endl;
        robot->set_vel(+trn, -trn);
        return;
    }

    if (fwd > 1.0)
    {
        cout << "here 2" << endl;
        robot->set_vel(2.0f - trn, 2.0f + trn);
        return;
    }

    // if all fails
    robot->set_vel(2, 2);
}

///////////////////////////////////////////////
////////////////// THREADS ////////////////////
///////////////////////////////////////////////

void robot_thread(Robot *robot)
{
    robot->do_stuff();
}

void path_thread(Robot *robot)
{
    while (1)
    {
        std::vector<coords> pathcopy = explore(robot->pos_x, robot->pos_y);

        mtx.lock();
        path = pathcopy;
        mtx.unlock();
        sleep(.0001);
    }
}

///////////////////////////////////////////////
/////////////////// MAIN //////////////////////
///////////////////////////////////////////////

int main(int argc, char *argv[])
{
    cam_init();

    cout << "making robot" << endl;
    cout << "I am trained to find the cardboard box, red backpack, blue cylinder, and orange cone. I will beep once I found an object!" << endl;

    // give people time to read the intro
    sleep(2);

    //start up threads
    Robot robot(argc, argv, callback);
    std::thread rthr(robot_thread, &robot);
    std::thread path(path_thread, &robot);
    return viz_run(argc, argv);
}
