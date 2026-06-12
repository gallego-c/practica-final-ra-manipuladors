import urx
import math
import time

robot = urx.Robot("10.10.73.239")
try:
    joints = robot.getj()
    joints[5] -= math.pi/2 + 0.05
    robot.movej(joints, acc=0.5, vel=0.2, wait=False)
    time.sleep(0.5)
finally:
    robot.close()
