import aerotechapi.axis
from aerotechapi import A3200Controller

# We create an instance of the controller
controller = A3200Controller()
controller.connect()

# We can enable and home different axes
controller.enable(('X', 'Y', 'Z'))
controller.set_home_speed(30)
controller.home(('X', 'Y', 'Z'))

# In the case of one axis, the function can be called with the arguments dirctly
controller.moveabs('X', 100, speed=20)
controller.moverel('Y', 20, speed=10)

# In the case of a coordinated move, the function is calleed with tuples for both arguments
controller.moveabs(('X', 'Y'), (5, 8), speed=10)
controller.moverel(('X', 'Y', 'Z'), (1, 1, 1))

# For convenience, axis objects can be created
from aerotechapi import Axis
X = Axis('X')
Y = Axis('Y')
Z = Axis('Z')

# Or using the controller's convenience function:
X, Y, Z = controller.create_axes(('X', 'Y', 'Z'))

# Axis objects can be used interchangibly with their string representation:
controller.moveabs((X, Y, Z), (1, 1, 1))

# They also offer an object oriented interface for single axis movements:
X.moveabs(10, speed=5)
Y.moverel(5)

# MultiAxes objects can be constructred using operator overloading and serve as an abstraction
# on top of the controller functions:

XY = X+Y
# >>> type(XY)
# aerotechapi.axis.MultiAxes

# In the case of a MultiAxes object containing two axes, a tuple of length 2 has to be supplied to the moveabs method
XY.moveabs((10, 20))

# They can also be constructed on the fly, which is their primary use case:
(X+Y+Z).enable()
(X+Y+Z).home()
(X+Y).moverel((10, 10))


