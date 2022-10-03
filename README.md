# aerotechapi

## Overview
This module provides a pure python interface to the [**Aerotech A3200 Stoftware-Based Machine Controller**][controller] based on a TCP/IP
connection. It is aimed to facilitate CNC-control of the Aerotech axes system through Python and allow for combination of complex on-the-go generation of
motion commands and the Python ecosystem. This package is built as an abstration on top of the AeroBasic language features, but also allows
the user to dispatch arbitrary AeroBasic commands. The AeroBasic documentation can be found [**here**][here].

This module is not extensively documented, nor is it built to reflect the complete functionality of the A3200. It is a private project that was designed by myself for myself.
For example, clockwise and counterclockwise motion are not implemented as of yet. Since some people have expressed, I am sharing my code here, feel free to adapt it to suit your 
purposes. I cannot give any guarantee for completeness, functionality or safety. 


[controller]: https://www.aerotech.com/product-catalog/motion-controller/a3200.aspx
[here]: http://www.aerotechmotioncontrol.com/ftp/pwpsoftware/manuals_helpfiles/Controllers/Automation%203200/A3200SoftwareHelpFiles.zip

## Installation
Clone the git repository, navigate to the root directory and run:
```sh
pip install setup.py
```

## Enabling TCP/IP-communication on the controller
This package requires the controller to accept commands over TCP/IP. Therefore, the ASCII-Command-Interface must be enabled in the A3200 Configuration Manager.
Navigate to your current confiuration file, then go to "System" -> "Communication" -> "ASCII". Ensure that the "CommandSetup" parameter is set to
a value of 0x00010004, apply the changes and reset the controller. Standard port is set to 8000.

## Basic Example

This package is built around the A3200Controller class. In order to issue commands to the physical controller, first instantiate an object
of that class.

```Python
from aerotechapi import A3200Controller

controller = A3200Controller()
controller.connect()

# Enalbe all three axes
controller.enable(('X', 'Y', 'Z'))

# Home all three axes
controller.home(('X', 'Y', 'Z'))

# Move the X- and Y- axes to the specified 
# absolute positions 100 and 150 respectively
controller.moveabs(('X', 'Y'), (100, 150))

# Move the X-axis by 50 units of length 
# relative to its current position and a 
# speed of 20 units per second
controller.moverel('X', 50, speed=20)

controller.disconnect()
```

## Advanced Examples

For testing purposes, the controller can be set to dummy-mode. Instead of connecting to a socket, it will print
all commands sent to the socket. This mode does not currently work for most methods that retrieve data from the controller.

```Python
>>> controller = A3200Controller(dummy=True)
connect
>>> controller.connect()
>>> controller.enable(('X', 'Y', 'Z'))
Sending message: ENABLE X Y Z
>>> controller.home(('X', 'Y', 'Z'))
Sending message: HOME X Y Z
>>> controller.moveabs(('X', 'Y', 'Z'), (100, 100, -70.5))
Sending message: G90 G1 X100 Y100 Z-70.5
>>> controller.disconnect()
disconnect
```

Although axes can be referred to by their string representation, dedicated axis objects can be created
by passing the string representation and a reference to the controller. The axes objects can be used as
parameters for every function or object that accepts the corresponding string representation.

```Python
from aerotechpi import Axis
X = Axis('X', controller)
Y = Axis('Y', controller)
Z = Axis('Y', controller)

controller.home((X, Y, Z))
```
Or by using a convenience function of the A3200Controller class:
```Python
X, Y, Z = controller.create_axes(('X', 'Y', 'Z'))
```

Additionally, Axis objects provide a subset of functions from the controller to be called directly on the 
respective axis.
```Python
X.moveabs(20, speed=5)
X.moverel(10, speed=10)
```

Multiple Axis objects can be stacked with the overloaded '+' operator in order to create a MultiAxes object,
that behaves similar to an Axis object. 

```Python
>>> X + Y
MultiAxes((X, Y))
>>> (X + Y).moveabs((10, 20))
Sending message: G90 G1 X10 Y20
>>> XY = X + Y
>>> XY + Z
MultiAxes((X, Y, Z))
>>> (X+Y+Z).home()
Sending message: HOME X Y Z
```

Information about the axes can also be retrieved from the controller:

```Python
from AeroTechAPI import A3200Controller

with A3200Controller() as controller:
    X, Y, Z = axes = controller.create_axes(('X', 'Y', 'Z'))
    for axis in axes:
        if not axis.is_enabled():
            axis.enable()
        if not axis.is_homed():
            axis.home()

```