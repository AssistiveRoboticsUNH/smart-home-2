import stretch_body.robot
r = stretch_body.robot.Robot()
did_succeed = False
while not did_succeed:
    try:
        did_succeed = r.startup()
    finally:
        r.stop()
print('successfully connected to Stretch!')
