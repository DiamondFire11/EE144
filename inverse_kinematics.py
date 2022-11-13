import numpy as np
from math import pi, cos, sin, atan, atan2, sqrt, acos

def inverse_kinematics(position):
    # input: the position of end effector [x, y, z]
    # output: joint angles [joint1, joint2, joint3]
    # add your code here to complete the computation
    
    link1z = 0.065
    link2z = 0.039
    link3x = 0.050
    link3z = 0.150
    link4x = 0.150
    x = position[0]
    y = position[1]
    z = position[2]

    joint1 = atan2(y,x)

    new_x = sqrt(x*x + y*y)
    z_mod = z-(link1z+link2z) #portion of Z parameter used for calculation

    a = atan2(link3x,link3z)

    c = atan2(z_mod, new_x)

    j2j4 = sqrt((new_x*new_x)+(z_mod*z_mod))

    j2j3 = sqrt((link3x*link3x)+(link3z*link3z))

    b_part1 = (j2j3*j2j3) + (j2j4*j2j4) - (link4x*link4x)

    b_part2 = 2*j2j3*j2j4

    b = acos(b_part1/b_part2)

    d_part1 = (j2j3*j2j3)+(link4x*link4x) - (j2j4*j2j4)

    d_part2 = 2*j2j3*link4x #j2j4 or l4?

    d_theta = acos(d_part1/d_part2)


    joint2 = (pi/2)-(a+b+c)


    d = (pi/2)-a

    joint3 = d + d_theta - pi


    return [joint1, joint2, joint3]
