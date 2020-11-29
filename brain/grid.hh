#ifndef GRID_HH
#define GRID_HH

#include "robot.hh"
#include "viz.hh"
#include "pose.hh"
typedef std::pair<int, int> coords;

static const float CELL_SIZE = 0.5;
static const float VIEW_SIZE = 41;

// convert color to corresponding num value to be used in grid_view
int stringToInt(std::string color);

// add wall hit to grid
void grid_apply_hit(float dist, float angle, Pose pose);

// add found object to object map
void grid_apply_hit_color(float dist, float angle, Pose pose, std::string color);

// return Mat to be pased to vix show
Mat grid_view(Pose pose, std::vector<coords> path);

// find path to given goal using grid using A*
std::vector<coords> grid_find_path(float x0, float y0, float x1, float y1);

// find angle to goal
float grid_goal_angle(Pose pose, std::vector<coords> path);

// explore map using Backtracing algorithm
std::vector<coords> explore(float x0, float y0);

// check if coordinate is safe
bool isSafe(coords point);

#endif
