import stretch_body.robot
import time, os, subprocess
from stretch_body.hello_utils import free_body_filelock



print("Starting Robot Body")
robot = stretch_body.robot.Robot()
robot.startup()

print("Moving joints")
# robot.arm.move_to(0)
robot.end_of_arm.move_to('wrist_pitch', 0.5)
robot.end_of_arm.move_to('wrist_roll', 0)
robot.end_of_arm.move_to('wrist_yaw', 0)
# robot.lift.move_to(0.7)
robot.push_command()


time.sleep(2)

print("Stopping Robot")
robot.stop()

did_free = free_body_filelock()
if did_free:
    print('Robot process is free.')
    
