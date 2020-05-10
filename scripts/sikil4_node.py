#!/usr/bin/env python

import rospy
import json
import math
from std_msgs.msg import Int32MultiArray
from sikil4.srv import *

min_pwm = 1800
max_pwm = 8700
default_pwm = (max_pwm - min_pwm) / 2
servo_pwm = [default_pwm, default_pwm, default_pwm, default_pwm, default_pwm, default_pwm, default_pwm, default_pwm, default_pwm, default_pwm, default_pwm, default_pwm, default_pwm, default_pwm, default_pwm, default_pwm]

body_queue = [[0, 0, 0]]
last_body_pos = [0, 0, 0]
leg_queue = [[[0, 0, 0]], [[0, 0, 0]], [[0, 0, 0]], [[0, 0, 0]]]
last_leg_pos = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
leg_mirror = [1, 1, -1, -1]
movement = [0, 0, 0]

L = 8 # keep last position
walk_seq = [[[L, L, L, 3], [0, 0, 1, 1], [1, 1, 1, 1], [1, 1, 0, 5], [0, 0, 0, 2]], [[0, 0, 1, 1], [1, 1, 1, 1], [1, 1, 0, 2], [0, 0, 0, 6], [-1, -1, 0, 2]], [[L, L, L, 4], [-1, -1, 0, 5], [0, 0, 1, 1], [1, 1, 1, 1], [1, 1, 0, 1]], [[L, L, L, 4], [-1, -1, 0, 2], [0, 0, 1, 1], [1, 1, 1, 1], [1, 1, 0, 2], [0, 0, 0, 2]]]
walk_prefix = [[], [], [[0, 0, 1], [0, 0, 0]], [[0, 0, 1], [0, 0, 0]]]

def enqueue(queue, x, y, z, step):
    last_pos = queue[0]
    dx = x - last_pos[0]
    dy = y - last_pos[1]
    dz = z - last_pos[2]
    #rospy.loginfo("%f %f %f", dx, dy, dz)
    if step == 0:
        d = math.sqrt((dx * dx) + (dy * dy) + (dz * dz))
        step = round(d / 0.01, 0)
        if step == 0:
            step = 1

    i = 1
    while i <= step:
        pos = [0, 0, 0]
        pos[0] = last_pos[0] + ((i / step) * dx)
        pos[1] = last_pos[1] + ((i / step) * dy)
        pos[2] = last_pos[2] + ((i / step) * dz)
        queue.insert(0, pos)
        i += 1
        #rospy.loginfo("%f %f %f", pos[0], pos[1], pos[2])

def dequeue(queue):
    pos = queue[len(queue) - 1]
    if len(queue) > 1:
        queue.pop()
    return pos

def reset_queue(queue, pos):
    queue = [pos]

servo_pos = [90, 180, 180, 90, 180, 180, 90, 180, 180, 90, 180, 180, 0, 0, 0, 0]
servo_rev = [1, 1, 1, 1, 1, 1, -1, -1, -1, -1, -1, -1, 1, 1, 1, 1]

def move_servo():
    for i in range(16):
        pos = servo_pos[i]
        if servo_rev[i] < 0:
            pos = 180 - pos
        if pos < 0:
            pos = 0
        if pos > 180:
            pos = 180
        servo_pwm[i] = min_pwm + (pos * (max_pwm - min_pwm) / 180)        

    cmd = Int32MultiArray()
    cmd.data = servo_pwm
    pub.publish(cmd)

def rad_to_deg(rad):
    return rad * 57.296

def deg_to_rad(deg):
    return deg / 57.296

rad90 = deg_to_rad(90)
rad180 = deg_to_rad(180)

def set_servo(ch, pos):
    servo_pos[ch] = rad_to_deg(pos)

def calc_servo():
    global body_queue, last_body_pos, leg_queue, last_leg_pos
    for i in range(4):
        body_pos = dequeue(body_queue)
        last_body_pos = [body_pos[0], body_pos[1], body_pos[2]]
        leg_pos = dequeue(leg_queue[i])
        last_leg_pos[i] = [leg_pos[0], leg_pos[1], leg_pos[2]]

        x = -body_pos[0] + leg_pos[0]
        y = -body_pos[1] + leg_pos[1]
        z = body_pos[2] - leg_pos[2]

        n = math.sqrt((y * y) + (4 * z * z)) / 2
        A = math.asin(n) * 2
        if n != 0:
            B = math.acos(n) - math.asin(y / (2 * n))
            S = math.asin(x / (2 * n))
        else:
            B = math.acos(n)
            S = 0
        set_servo((i * 3), rad90 - S)
        set_servo((i * 3) + 1, rad90 + B)
        set_servo((i * 3) + 2, rad180 - A)

def reset_leg_queue():
    global leg_queue
    for i in range(4):
        reset_queue(leg_queue[i], [0, 0, 0])
    
def cut_leg_queue():
    global leg_queue, last_leg_pos
    for i in range(4):
        reset_queue(leg_queue[i], last_leg_pos)
    
def leg_requeue():
    global movement, leg_queue, last_leg_pos, walk_seq    
    step = round(max(movement[0], movement[1], movement[2]) / 0.05, 0)
    if (movement[0] != 0 or movement[1] != 0) and movement[2] != 0:
        for i in range(4):
            x = last_leg_pos[i][0]
            y = last_leg_pos[i][1]
            z = last_leg_pos[i][2]
            for j in range(len(walk_seq[i])):
                seq = walk_seq[i][j]
                for k in range(seq[3]):
                    if seq[0] != L:
                        x = seq[0] * movement[0] * leg_mirror[i]
                    if seq[1] != L:
                        y = seq[1] * movement[1]
                    if seq[2] != L:
                        z = seq[2] * movement[2]
                    enqueue(leg_queue[i], x, y, z, step)
                    #rospy.loginfo("%d %f %f %f", i, x, y, z)

def handle_task(task):
    global body_queue, leg_queue, movement
    if task.cmd == 'stand':
        enqueue(body_queue, task.x, task.y, task.z, 0)
        movement = [movement[0], movement[1], task.z / 2]
        return True
    if task.cmd == 'sit':
        movement = [0, 0, 0]
        reset_leg_queue()
        enqueue(body_queue, 0, 0, 0, 0)
        return True
    if task.cmd == 'walk':
        enqueue(body_queue, 0, 0, task.z, 0)
        movement = [task.x, task.y, task.z / 2]
        cut_leg_queue()
        if (movement[0] != 0 or movement[1] != 0) and movement[2] != 0:
            for i in range(4):
                for j in range(len(walk_prefix[i])):
                    seq = walk_prefix[i][j]
                    x = seq[0] * movement[0] * leg_mirror[i]
                    y = seq[1] * movement[1]
                    z = seq[2] * movement[2]
                    enqueue(leg_queue[i], x, y, z, 0)
        leg_requeue()
        return True
    if task.cmd == 'stop':
        movement = [0, 0, movement[2]]
        cut_leg_queue()
        for i in range(4):
            enqueue(leg_queue[i], leg_queue[i][0][0], leg_queue[i][0][1], 0, 0)
        return True
    return False

def sikil4_node():
    global pub, leg_queue
    rospy.init_node('sikil4')
    pub = rospy.Publisher('/command', Int32MultiArray, queue_size = 0)
    task = rospy.Service('sikil4/task', Task, handle_task)

    rate = rospy.Rate(25)
    while not rospy.is_shutdown():
        if len(leg_queue[0]) <= 1:
            leg_requeue()
        calc_servo();
        move_servo();
        rate.sleep()

    return 0

if __name__ == '__main__':
    try:
        sikil4_node()
    except rospy.ROSInterruptException:
        pass
