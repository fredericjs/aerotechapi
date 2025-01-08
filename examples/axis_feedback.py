from aerotechapi import A3200Controller

controller = A3200Controller()
controller.connect()

X, Y, Z = axes = controller.create_axes(('X', 'Y', 'Z'))

# Check if axes are enabled and enable if necessary

# We can do this by looping over all axes and using the object oriented interface
for axis in axes:
    if not axis.is_enabled():
        axis.enable()
for axis in axes:
    if not axis.is_homed():
        axis.home()

# Or we can use the controller's function
axis_states = controller.is_enabled(axes)
# This returns an AxesDict, which works like a std dict. However, both strings and Axis objects can be used
# interchangeably to index the dict
# >>> axis states
# AxesDict({X: False, Y: False, Z: False})
for axis in axes:
    if not axis_states[axis]:
        axis.enable()

# Or we could use a list comprehension
[axis.enable() if not enabled for axis, enabled in axis_states.items()]

# We can obtain the position of each axis through the axis object
xpos = X.get_position()

# We can obtain the values for multiple axes similar to the enabled or homed state
# This also returns an AxisDict
positions = controller.get_positions((X, Y, Z))
# We can either retrieve the values be indexing with the respective axis
xpos = positions['X'] # We can use  strings
ypos = positions[Y] # And we can use Axis objects
zpos = positions[Z]
# Or we can retrieve the values in the order of the axes in the status query
xpos, ypos, zpos = positions.values

# For example, we can use this to move an axis non-blocking with the freerun command and stop when a specific position
# is reached:
X.freerun_start(speed=20)
while X.get_position() < 200:
    pass
X.freerun_stop()

# We can also pass an AxisDict to controller functions like moveabs and moverel:
start_positions = controller.get_positions((X, Y, Z))
# Now we do some operation
X.moveabs(10)
Z.moverel(20)
# Then we use the AxisDict to return to our recorded starting position
controller.moveabs(start_positions)
# Instead of having to write:
controller.moveabs(start_positions.axes, start_positions.values)
