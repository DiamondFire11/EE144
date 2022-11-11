import numpy as np
from math import pi, cos, sin
import modern_robotics as mr


def createTransformationMatrix(rotationMatrix, translationMatrix):
    intermediateMatrix = np.concatenate((rotationMatrix, translationMatrix.getT()), axis=1)
    finalMatrix = np.vstack((intermediateMatrix, [0.0, 0.0, 0.0, 1.0]))
    return finalMatrix


def createRotationMatrix(axis, theta):
    if axis == "X":
        return np.matrix([[1, 0.0, 0.0], [0.0, cos(theta), -sin(theta)], [0.0, sin(theta), cos(theta)]])
    if axis == "Y":
        return np.matrix([[cos(theta), 0.0, sin(theta)], [0.0, 1, 0.0], [-sin(theta), 0.0, cos(theta)]])
    if axis == "Z":
        return np.matrix([[cos(theta), -sin(theta), 0.0], [sin(theta), cos(theta), 0.0], [0.0, 0.0, 1]])


def createTranslationMatrix(xLength=0.0, yLength=0.0, zLength=0.0):
    return np.matrix([xLength, yLength, zLength])


def forward_kinematics(joints):
    # input: joint angles [joint1, joint2, joint3]
    # output: the position of end effector [x, y, z]
    # add your code here to complete the computation

    # Joint constraints
    link1z = 0.065
    link2z = 0.039
    link3x = 0.050
    link3z = 0.150
    link4x = 0.150
    joint1 = joints[0]
    joint2 = joints[1]
    joint3 = joints[2]

    t01 = createTransformationMatrix(createRotationMatrix('Z', joint1), createTranslationMatrix(0.0, 0.0, link1z))                                                                  # t01 describes pose of joint 1 relative to joint 0
    t12 = createTransformationMatrix(createRotationMatrix('Y', joint2), createTranslationMatrix(0.0, 0.0, link2z))                                                                  # t12 describes pose of joint 2 relative to joint 1
    t23 = createTransformationMatrix([[1, 0.0, 0.0], [0.0, -1, 0.0], [0.0, 0.0, -1]] * createRotationMatrix('Y', joint3), createTranslationMatrix(link3x, 0.0, link3z))             # t23 describes pose of joint 3 relative to joint 2
    t34 = createTransformationMatrix([[1, 0, 0], [0, 1, 0], [0, 0, 1]], createTranslationMatrix(link4x, 0.0, 0.0))                                                                  # t34 describes pose of joint 4 relative to joint 3

    t04 = t01 * t12 * t23 * t34  # t04 is the final transformation matrix describing (X, Y, Z) pose of robotic arm
    return [t04.item(0, 3), t04.item(1, 3), t04.item(2, 3)]
