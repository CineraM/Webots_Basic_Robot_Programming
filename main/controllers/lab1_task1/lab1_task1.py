"""
Matias Cinera - U 6931_8506
CAP-6626
Instructor: Dr.Alfredo Weitzenfeld 
Ta:         Chance Hamilton
Assigment:  lab1_task1   controller.
"""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
# import numpy as it may be used in future labs
import numpy as np

# create the Robot instance.
robot = Robot()

# get the time step of the current world in msec.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

# getting the position sensors
leftposition_sensor = robot.getDevice('left wheel sensor')
rightposition_sensor = robot.getDevice('right wheel sensor')
leftposition_sensor.enable(timestep)
rightposition_sensor.enable(timestep)

# Robot specs || Global variables 
distBtwWhe = 2.28
distBtwWheR = distBtwWhe/2
w_dia = 1.6
w_r = w_dia/2

# move either left, or right motor with speed v
def singleMotorMotionV(v, isLeft):
    if(isLeft):
        leftMotor.setVelocity(v)
    else:
        rightMotor.setVelocity(v)

# Transform inches to rad, based on the max speed of the motors
# max speed is 5.024 in/s
def inToRad(v, isLeft):
    motor = "Right"
    if isLeft : motor = "Left "

    v = v/w_r                # input=inches
    if v > 6.28:             # max rotational velocity
        print(f'ERROR: Invalid {motor} input ({v:.2f} > 6.28), setting it to 6.28')
        v = 6.28
    elif v < 0:
        print(f'ERROR: Invalid {motor} input ({v:.2f} < 0), setting it to 0')
        v = 0

    return v

# move the left and right motors by a specified speed and period of time
def dualMotorMotionLRT(l, r, t):
    l = inToRad(l, True)        
    r = inToRad(r, False) 
    
    t_prev = robot.getTime()
    print(f'Task [vl: {l:.2f}, vr: {r:.2f}, t: {t:.2f}] - Start-time: {t_prev}')
    singleMotorMotionV(l, True)
    singleMotorMotionV(r, False)
    # the default timestep is 32 seconds, however, 
    # you can use the step function to pass time in miliseconds instead of the default 32
    robot.step(int(t*1000))   # conversion to ms
    t_cur = robot.getTime()
    print(f'Task [vl: {l:.2f}, vr: {r:.2f}, t: {t:.2f}] - End-time:   {t_cur}')
    print(f'Task Time duration: {(t_cur-t_prev):.2f}')
    
    singleMotorMotionV(0, True)
    singleMotorMotionV(0, False)
    return (t_cur-t_prev)


def robotControl(A):
    total_time = 0
    robot.step(timestep)    # used to buffer the robot sensors
    for vec in A:
        if vec[2] <= 0:
            continue
        total_time += dualMotorMotionLRT(vec[0], vec[1], vec[2])
        print("---------------------------------------------------------------")
    print(f'Stopping time: {robot.getTime():.2f}, Total time to complete tasks: {total_time:.2f}')

def main():
    test_input = [[3, 3, 2], [1, 2, 1], [2,1,1], [0.5,0.5, 1], [5, 5, 2],
            [2, 1.1, 1], [1.1, 2, 2.3], [2,3,3.23], [-1,-2, 1], [8, 8, 2]]
    robotControl(test_input)

if __name__ == "__main__":
    main()