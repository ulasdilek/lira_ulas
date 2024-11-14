import robotic as ry
import time
import numpy as np

config = ry.Config()

# add base frame
config.addFrame("base").setPosition([0, 0, 0])

# add cylinder agent
# joint is XY translation
# color is stolen from svetlana's cylinder :D
config.addFrame("agent", "base") \
    .setJoint(ry.JT.transXY, [-10, 10, -10, 10]) \
    .setRelativePosition([0, 0, 0.1]) \
    .setShape(ry.ST.cylinder, size=[0.2, 0.1]) \
    .setColor([0.96875, 0.7421875, 0.30859375]) \
    .setContact(1)

# show point A for visualisation purposes
config.addFrame("point_A", "base") \
    .setJoint(ry.JT.rigid) \
    .setShape(ry.ST.box, size=[0.1, 0.1, 0.1]) \
    .setColor([0, 0, 1]) \
    .setRelativePosition([-2, 0, 0.05]) \
    .setContact(0)

# show point B for visualisation purposes
config.addFrame("point_B", "base") \
    .setJoint(ry.JT.rigid) \
    .setShape(ry.ST.box, size=[0.1, 0.1, 0.1]) \
    .setColor([1, 0, 0]) \
    .setRelativePosition([2, 0, 0.05]) \
    .setContact(0)

# add an obstacle in the middle
config.addFrame("obstacle" ,"base") \
    .setJoint(ry.JT.rigid) \
    .setShape(ry.ST.box, size=[0.2, 2.0, 0.1]) \
    .setColor([0.5, 0.2, 0.25]) \
    .setRelativePosition([0, 0, 0.05]) \
    .setContact(1)

# display the config
pressed_key = config.view(pause=True, message="press a key")

# get pointA and pointB positions
posA = config.getFrame(frameName="point_A").getRelativePosition()
posB = config.getFrame(frameName="point_B").getRelativePosition()

# position -> joint states
# these will be valid since points and agent share the same parent
qA = posA[:2]
qB = posB[:2]

# display the frames
print(qA)
print(qB)

# configure the rrt parameters
# can experiment with stepsize
ry.params_add({
    "rrt/stepsize":0.1,
    "rrt/verbose":3
})

# execute rrt
rrt = ry.PathFinder()
# given the configuration, find a path that takes the joint state from qA to qB
rrt.setProblem(config, [qA], [qB]) 
solution = rrt.solve()
print(solution)
path = solution.x
print("path length:", path.shape[0])
print("path:", path)
del rrt

# display the path and step sizes (cuz I think there is something here)
prev_state = qA
for state in path:
    print(np.linalg.norm(state - prev_state)) # print actual step size
    prev_state = state

    config.setJointState(state)
    config.view()
    time.sleep(3./path.shape[0]) # animate in 3 seconds

config.view_close()
del config