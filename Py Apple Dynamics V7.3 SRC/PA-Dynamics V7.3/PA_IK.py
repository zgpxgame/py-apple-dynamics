# Copyright Deng（灯哥） (ream_d@yeah.net)  Py-apple dog project
# Github:https://github.com/ToanTech/py-apple-quadruped-robot
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:http://www.apache.org/licenses/LICENSE-2.0

from math import acos, atan, pi, sqrt

#######################
# 逆运动学
#######################

# 弧度转角度
Rad2Deg = 57.29578


def leg_ik(l1, l2, x, y):
    x = -x

    shank = pi - acos((x * x + y * y - l1 * l1 - l2 * l2) / (-2 * l1 * l2))
    fai = acos((l1 * l1 + x * x + y * y - l2 * l2) / (2 * l1 * sqrt(x * x + y * y)))
    if x > 0:
        ham = abs(atan(y / x)) - fai
    elif x < 0:
        ham = pi - abs(atan(y / x)) - fai
    else:
        ham = pi - 1.5707 - fai

    shank = shank * Rad2Deg
    ham = ham * Rad2Deg

    return shank, ham


# 输入参数
# l1 大腿长度
# l2 小腿长度
# x1, y1 腿1坐标
# x2, y2 腿2坐标
# x3, y3 腿3坐标
# x4, y4 腿4坐标
#
# 输出
#   大腿1 2 3 4角度，小腿1 2 3 4角度
def ik(l1, l2, x1, x2, x3, x4, y1, y2, y3, y4):
    shank1, ham1 = leg_ik(l1, l2, x1, y1)
    shank2, ham2 = leg_ik(l1, l2, x2, y2)
    shank3, ham3 = leg_ik(l1, l2, x3, y3)
    shank4, ham4 = leg_ik(l1, l2, x4, y4)
    return ham1, ham2, ham3, ham4, shank1, shank2, shank3, shank4
