#include "Robot.h"
#include "Berth.h"
#include "utils.h"

int Berth::max_transport_time = 1;
int Berth::min_transport_time = INT_MAX;
int Berth::nearest_berth_id = -1;
int Berth::farest_berth_id = -1;
int Berth::average_transport_time = 1;

// later change to available point list, not a single point
// and at same, fill relative grid to ocean
void Berth::initXY(char grid[][N])
{
    // // count ocean grid num near the four point
    // int left_top = countOceanGrid(x, y, grid);
    // int left_bottom = countOceanGrid(x + 3, y, grid);
    // int right_top = countOceanGrid(x, y + 3, grid);
    // int right_bottom = countOceanGrid(x + 3, y + 3, grid);

    // if (left_top >= 2 && left_bottom >= 2) {
    //     // left is ocean
    //     robot_at_x = x + 1;
    //     robot_at_y = y + 3;
    //     // fillOcean(grid, x, y, x + 3, y + 1);
    // }
    // else if (left_top >= 2 && right_top >= 2) {
    //     // up is ocean
    //     robot_at_x = x + 3;
    //     robot_at_y = y + 1;
    //     // fillOcean(grid, x, y, x + 1, y + 3);
    // }
    // else if (left_bottom >= 2 && right_bottom >= 0) {
    //     // bottom is ocean
    //     robot_at_x = x;
    //     robot_at_y = y + 1;
    //     // fillOcean(grid, x + 2, y, x + 3, y + 3);
    // }
    // else if(right_top >= 2 && right_bottom >= 2){
    //     //right is ocean
    //     robot_at_x = x + 1;
    //     robot_at_y = y;
    //     // fillOcean(grid, x, y + 2, x + 3, y + 3);
    // }
    // else if (left_top >= 2) {
    //     robot_at_x = x + 2;
    //     robot_at_y = y + 3;
    //     // fillOcean(grid, x, y, x + 3, y + 2);
    // }
    // else if (left_bottom >= 2) {
    //     robot_at_x = x + 1;
    //     robot_at_y = y + 3;
    //     // fillOcean(grid, x, y, x + 3, y + 2);
    // }
    // else if (right_top >= 2) {
    //     robot_at_x = x + 2;
    //     robot_at_y = y;
    //     // fillOcean(grid, x, y + 1, x + 3, y + 3);
    // }
    // else {
    //     robot_at_x = x + 1;
    //     robot_at_y = y;
    //     // fillOcean(grid, x, y + 1, x + 3, y + 3);
    // }

    robot_at_x = x + 1;
    robot_at_y = y + 1;
}