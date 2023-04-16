// Libraries
#include <math.h>

// Constants
#define coffset 61.86 // Plads holder værdi
#define j1_l 154 // Plads holder værdi
#define j2_l 140 // Plads holder værdi
#define j3_l 140 // Plads holder værdi
#define j5_l 150 // Plads holder værdi

// Function to solve inverse kinematics
void solveInverseKinematics(int x, int y, int z){
    float c_to_p = sqrt(pow(x - 0, 2) + pow(y - 0, 2));
    float vc1 = acos(x / c_to_p) * (180.0 / M_PI);
    float vc2 = acos(coffset / c_to_p) * (180.0 / M_PI);
    float v1 = 180.0 - (vc1 + vc2);
    float offsetx = (cos(v1 * (M_PI / 180.0)) * coffset);
    float offsety = (sin(v1 * (M_PI / 180.0)) * coffset);
    float offset_to_j4 = sqrt(pow(x - offsetx, 2) + pow(y - offsety, 2) + pow((z + j5_l) - 0, 2));
    float j2_to_j4 = sqrt(pow(x - offsetx, 2) + pow(y - offsety, 2) + pow((z + j5_l) - j1_l, 2));
    float v2_1 = acos((pow(j1_l, 2) + pow(j2_to_j4, 2) - pow(offset_to_j4, 2)) / (2 * j1_l * j2_to_j4)) * (180.0 / M_PI);
    float v2_2 = acos((pow(j2_l, 2) + pow(j2_to_j4, 2) - pow(j3_l, 2)) / (2 * j2_l * j2_to_j4)) * (180.0 / M_PI);
    float v2 = v2_1 + v2_2;
    float v3 = acos((pow(j3_l, 2) + pow(j2_l, 2) - pow(j2_to_j4, 2)) / (2 * j3_l * j2_l)) * (180.0 / M_PI);
    float v4_1 = acos((z + j5_l) / offset_to_j4) * (180.0 / M_PI);
    float v4_2 = acos((pow(j2_to_j4, 2) + pow(offset_to_j4, 2) - pow(j1_l, 2)) / (2 * j2_to_j4 * offset_to_j4)) * (180.0 / M_PI);
    float v4_3 = 180.0 - v2_2 - v3;
    float v4 = v4_1 + v4_2 + v4_3;
    float v5 = acos((x - offsetx) / c_to_p) * (180.0 / M_PI);
}

void robot_move(double x, double y, double z);
{
    solveInverseKinematics(x,y,z)
}

