import robotic as ry

config = ry.Config()
config.addFile("puzzles/p9-cube-free.g")

key = config.view(pause=True, message="press a key")

del config