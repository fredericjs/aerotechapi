import math
from aerotechapi import A3200Controller

controller = A3200Controller(dummy=True)
controller.connect()

X, Y, Z = controller.create_axes(('X', 'Y', 'Z'))
controller.enable((X, Y, Z))
controller.home((X, Y, Z))

controller.moveabs(X, 0)
controller.moveabs(Z, 50)

# We create a table with x-values and corresponding z-values to make a sinusoidal movement of the z-axis when
# then x-axis moves
x_positions = [i for i in range(100)] # [0, 1, 2, 3, 4, ..., 99]
z_positions = [math.sin(x) for x in x_positions] # [0.0, 0.8414709848078965, 0.9092974268256817, ...]

# X will be the master axis, Z will be the slave axis
controller.write_cam_table(x_positions, z_positions, X, Z)
controller.load_camtable(X, Z, wrap=False)
controller.start_camsync(Z)

# Make the moves
X.moveabs(100, speed=10)

controller.stop_camsync(Z)
controller.free_camtable()

# We return to X=0
X.moveabs(0, speed=40)

# We can also use a contextmanager to take care of all the details:
with controller.camming(x_positions, z_positions, X, Z):
    X.moveabs(100, speed=10)
