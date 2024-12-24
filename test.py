import robotic as ry

# config = ry.Config()
# config.addFile("puzzles/p6-wall.g")
# config.view_setCamera(config.getFrame("camera_top"))
# config.view(True)

config = ry.Config()
config.addFrame("box") \
        .setShape(ry.ST.box, [0.1, 0.1, 0.1]) \
        .setPosition([0, 0.2, 0]) \
        .setColor([1, 0, 0])
config.addFrame("cylinder") \
        .setShape(ry.ST.cylinder, [0.1, 0.1]) \
        .setPosition([0, -0.2, 0]) \
        .setColor([0, 1, 0])
config.view(True)

config.view_close()
del config