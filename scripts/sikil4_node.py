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

L = 8 #relative to last position
walk_seq = [[[L, L, L, 2], [0, 0, 1, 1], [1, 1, 1, 1], [1, 1, 0, 1], [0, 0, 0, 1], [L, L, L, 4], [0, -1, 0, 1]], [[0, 0, 1, 1], [1, 1, 1, 1], [1, 1, 0, 1], [L, L, L, 2], [0, 0, 0, 1], [L, L, L, 4], [0, -1, 0, 1]], [[L, L, L, 4], [0, -1, 0, 1], [0, 0, 1, 1], [1, 1, 1, 1], [1, 1, 0, 1], [L, L, L, 2], [0, 0, 0, 1]], [[L, L, L, 4], [0, -1, 0, 1], [L, L, L, 2], [0, 0, 1, 1], [1, 1, 1, 1], [1, 1, 0, 1], [0, 0, 0, 1]]]

def enqueue(queue, x, y, z, step):
    #rospy.loginfo("%f %f %f %f", x, y, z, step)
    last_pos = queue[0]
    dx = x - last_pos[0]
    dy = y - last_pos[1]
    dz = z - last_pos[2]
    if step == 0:
        d = math.sqrt((dx * dx) + (dy * dy) + (dz * dz))
        step = round(d / 0.01, 0)

    i = 1
    while i < step:
        pos = [0, 0, 0]
        pos[0] = last_pos[0] + ((i / step) * dx)
        pos[1] = last_pos[1] + ((i / step) * dy)
        pos[2] = last_pos[2] + ((i / step) * dz)
        #rospy.loginfo("%f %f %f", pos[0], pos[1], pos[2])
        queue.insert(0, pos)
        i += 1
    queue.insert(0, [x, y, z])
    #rospy.loginfo("%f %f %f", x, y, z)

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

        x = (-body_pos[0] * leg_mirror[i]) + leg_pos[0]
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
    global leg_queue, last_leg_pos
    for i in range(4):
        reset_queue(leg_queue[i], last_leg_pos)
    
def leg_requeue():
    global movement, last_body_pos, leg_queue, last_leg_pos, walk_seq
    e = 99
    for i in range(4):
        if e > len(leg_queue[i]):
            e = len(leg_queue[i])
    if e > 1:
        return
  
    step = round(last_body_pos[2] / 0.05, 0)
    reset_leg_queue()
    for i in range(4):
        if (movement[0] != 0 or movement[1] != 0) and movement[2] != 0:
            for j in range(len(walk_seq[i])):
                seq = walk_seq[i][j]
                for k in range(seq[3]):
                    if seq[0] == L:
                        x = last_leg_pos[i][0]
                    else:
                        x = seq[0] * last_body_pos[2] * movement[0]
                    if seq[1] == L:
                        y = last_leg_pos[i][1]
                    else:
                        y = seq[1] * last_body_pos[2] * movement[1]
                    if seq[2] == L:
                        z = last_leg_pos[i][2]
                    else:
                        z = seq[2] * last_body_pos[2] * movement[2]
                    enqueue(leg_queue[i], x, y, z, step)

def handle_task(task):
    global body_queue, leg_queue, movement
    if task.cmd == 'stand':
        enqueue(body_queue, task.x, task.y, task.z, 0)
        return True
    if task.cmd == 'sit':
        enqueue(body_queue, 0, 0, 0, 0)
        return True
    if task.cmd == 'walk':
        movement = [task.x, task.y, task.z]
        reset_leg_queue()
        return True
    if task.cmd == 'stop':
        movement = [0, 0, 0]
        reset_leg_queue()
        for i in range(4):
            enqueue(leg_queue[i], 0, 0, 0.5, 0)
            enqueue(leg_queue[i], 0, 0, 0, 0)
        return True
    return False

def sikil4_node():
    global pub
    rospy.init_node('sikil4')
    pub = rospy.Publisher('/command', Int32MultiArray, queue_size = 0)
    task = rospy.Service('sikil4/task', Task, handle_task)

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
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
