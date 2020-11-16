#include <iostream>
#include <math.h>
#include <thread>
#include <algorithm>
#include <vector>

#include "robot.hh"
#include "cam.hh"
#include "grid.hh"
#include "viz.hh"

#include <opencv2/imgproc.hpp>

using std::cout;
using std::endl;

// globals
const double GOAL_X = 20.0;
const double GOAL_Y = 0.0;
int CAMWitdh = 100;
int CAMHeight = 100;
int CAMWitdhSCALED = 100;
int CAMHeightSCALED = 100;
float pixToRealHit = 0.0;
float rightHit = 0;
float leftHIT = 0;
std::vector<coords> path;
std::mutex mtx;
using namespace cv;

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
int wallDistanceInPixels(int col, cv::Mat image)
{
    for (int row = 0; row < CAMHeightSCALED; row++)
    {
        //Vec3b -> B G R
        Vec3b pixelColor = image.at<Vec3b>(cv::Point(col, row));

        // find where sky becomes wall Sky color is 178 178 178
        if (abs(pixelColor.val[0] - 178) > 0 && abs(pixelColor.val[1] - 178) > 0 && abs(pixelColor.val[2] - 178) > 0)
        {
            return row;
        }
    }
    return -99;
}

/*
To view the camera image in time, you could press CTRL-T in Gazebo
, choosing the Topic-"~/tankbot0/tankbot/camera_sensor/link/camera/image", 
then a Image Window will pop up, in order to view the Image in time.
*/

void callback(Robot *robot)
{
    cv::Mat fullMat, smallMat;
    fullMat = robot->frame;
    if (!isImageEmpty(fullMat))
    {
        return;
    }
    //cam_show(robot->frame);

    Pose pose(robot->pos_x, robot->pos_y, robot->pos_t);
    if (robot->range < 1.9 && robot->range > 0)
    {
        cout << "LASER HIT " << robot->range << endl;
        // for directly in front of cam
        int pix = wallDistanceInPixels(CAMWitdhSCALED / 2, fullMat);
        if (pix > 0)
        {
            // pixel to real world scale factor
            pixToRealHit = pix / robot->range;

            grid_apply_hit(pix / pixToRealHit, 0, pose);
            cout << "F " << pix / pixToRealHit << " ANG " << 0 << endl;
            cout << "F pixel " << pix << " pixToRealHit " << pixToRealHit << endl;
        }
        if (pix == 0)
        {
            cout << "can only see wall" << endl;
            grid_apply_hit(.5, 0, pose);
        }
        else
        {
            if (abs(robot->pos_t) < .2)
            {
                // to the left 20
                int pix2 = wallDistanceInPixels((CAMWitdhSCALED / 2) - 20, fullMat);
                if (pix2 > 0)
                {
                    float hypot = sqrt((20 * 20) + (pix2 * pix2));
                    float angle = asin(pix2 / hypot);
                    angle = (M_PI / 2) - angle;
                    float hypoFactor = hypot / pixToRealHit;
                    leftHIT = hypot / pixToRealHit;

                    if (hypoFactor < 2.0)
                    {
                        grid_apply_hit(hypoFactor, angle, pose);
                        cout << "L " << hypot / pixToRealHit << " ANG " << angle << endl;
                    }
                }

                //sample right 40
                int pix3 = wallDistanceInPixels(CAMWitdhSCALED / 2 + 40, fullMat);
                if (pix3 > 0)
                {

                    float hypot = sqrt((40 * 40) + (pix3 * pix3));
                    float angle = asin(pix3 / hypot);
                    angle = (M_PI / 2) - angle;
                    float hypoFactor = hypot / pixToRealHit;
                    rightHit = hypot / pixToRealHit;
                    angle = -angle;

                    if (hypoFactor < 2.0)
                    {
                        grid_apply_hit(hypoFactor, angle, pose);
                        cout << "R " << hypot / pixToRealHit << " ANG " << angle << endl;
                    }
                }

                //sample right 20
                int pix4 = wallDistanceInPixels(CAMWitdhSCALED / 2 + 20, fullMat);
                if (pix4 > 0)
                {

                    float hypot = sqrt((20 * 20) + (pix4 * pix4));
                    float angle = asin(pix4 / hypot);
                    angle = (M_PI / 2) - angle;
                    float hypoFactor = hypot / pixToRealHit;
                    rightHit = hypot / pixToRealHit;
                    angle = -angle;

                    if (hypoFactor < 2.4)
                    {
                        grid_apply_hit(hypoFactor, angle, pose);
                        cout << "R " << hypot / pixToRealHit << " ANG " << angle << endl;
                    }
                }
            }
        }
    }

    mtx.lock();
    std::vector<coords> pathcopy = path;
    mtx.unlock();

    Mat view = grid_view(pose, pathcopy);
    viz_show(view);

    float ang = grid_goal_angle(pose, pathcopy);
    float trn = clamp(-1.0, 3 * ang, 1.0);

    float fwd = clamp(0.0, robot->range, 2.0);
    float lft = clamp(0.0, leftHIT, 2.0);
    float rgt = clamp(0.0, rightHit, 2.0);

    double dx = GOAL_X - robot->pos_x;
    double dy = GOAL_Y - robot->pos_y;
    if (abs(dx) < 0.75 && abs(dy) < 0.75)
    {
        cout << "we win!" << endl;
        robot->set_vel(0, 0);
        robot->done();
        return;
    }

    if (abs(ang) > 0.5)
    {
        robot->set_vel(-trn, +trn);
        return;
    }

    if (fwd > 1.0)
    {
        robot->set_vel(2.0f - trn, 2.0f + trn);
        return;
    }

    robot->set_vel(-2.0, 2.0);
}

void robot_thread(Robot *robot)
{
    robot->do_stuff();
}

void path_thread(Robot *robot)
{
    while (1)
    {
        std::vector<coords> pathcopy = grid_find_path(robot->pos_x, robot->pos_y, 20.0f, 0.0f);

        mtx.lock();
        path = pathcopy;
        mtx.unlock();
        sleep(.0001);
    }
}

int main(int argc, char *argv[])
{
    cam_init();

    cout << "making robot" << endl;
    Robot robot(argc, argv, callback);
    std::thread rthr(robot_thread, &robot);
    std::thread path(path_thread, &robot);
    return viz_run(argc, argv);
}

// Callback filling out occupancy map w just laser info
// Putting this here in case we need it
// void callback(Robot *robot)
// {
//     if (robot->ranges.size() < 5)
//     {
//         return;
//     }

//     Pose pose(robot->pos_x, robot->pos_y, robot->pos_t);

//     for (auto hit : robot->ranges)
//     {
//         if (hit.range < 100)
//         {
//             grid_apply_hit(hit, pose);
//         }
//     }

//     // Goal is at x = 20, y = 0
//     grid_find_path(pose.x, pose.y, 20.0f, 0.0f);

//     Mat view = grid_view(pose);
//     viz_show(view);

//     float ang = grid_goal_angle(pose);
//     float trn = clamp(-1.0, 3 * ang, 1.0);

//     float fwd = clamp(0.0, robot->ranges[3].range, 2.0);
//     float lft = clamp(0.0, robot->ranges[2].range, 2.0);
//     float rgt = clamp(0.0, robot->ranges[4].range, 2.0);

//     if (abs(ang) > 0.5)
//     {
//         robot->set_vel(-trn, +trn);
//         return;
//     }

//     if (fwd > 1.0)
//     {
//         robot->set_vel(2.0f - trn, 2.0f + trn);
//         return;
//     }

//     // Wall follow.
//     float spd = clamp(0, fwd - 1.0, 1);
//     if (lft < rgt)
//     {
//         trn = 1;
//         if (lft < 0.75)
//         {
//             spd = 0;
//         }
//     }
//     else
//     {
//         if (rgt < 0.75)
//         {
//             spd = 0;
//         }
//         trn = -1;
//     }

//     //cout << "spd,trn = " << spd << "," << trn << endl;
//     robot->set_vel(spd - trn, spd + trn);
// }