#include <stdio.h>
#include <math.h>
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "AS5600.h"
#include "TCA9548A"

double j1_l,j2_l,,j4_l,j5_l;


void solveInverseKinematics(double x, double y, double z)
{
    float c_to_p;
    c_to_p = sqrt((x - 0)**2 + (y - 0)**2); // regner fra centrum til punktet
}
void robot_move(double x, double y, double z);
{
    TCA9548A_selectBUS(1);
    double j1_l = AS5600_read();
    TCA9548A_selectBUS(2);
    double j2_l = AS5600_read();
    TCA9548A_selectBUS(3);
    double j3_l = AS5600_read();
    TCA9548A_selectBUS(4);
    double j4_l = AS5600_read();
    TCA9548A_selectBUS(5);
    double j5_l = AS5600_read();
    solveInverseKinematics(x,y,z)
}