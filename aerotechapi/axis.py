from collections.abc import MutableMapping


class Axis:

    def __init__(self, identifier, controller):
        self._identifier = identifier
        self._controller = controller

    def __repr__(self):
        return f'Axis({self._identifier})'

    def __str__(self):
        return self._identifier

    def __add__(self, other):
        if self._identifier == other._identifier:
            raise ValueError('Cannot add multiple instances of the same axis')
        return MultiAxes((self, other), self._controller)

    def __radd__(self, other):
        if self._identifier == other._identifier:
            raise ValueError('Cannot add multiple instances of the same axis')
        return MultiAxes((self, other), self._controller)

    def __bool__(self):
        if self.is_enabled and self.is_homed:
            return True
        return False

    def enable(self):
        self._controller.enable(self)

    def disable(self):
        self._controller.disable(self)

    def home(self):
        self._controller.home(self)

    def set_home_speed(self, speed):
        self._controller.set_home_speed(self, speed)

    def moveabs(self, position, speed=None):
        self._controller.moveabs(self, position, speed)

    def moverel(self, distance, speed=None):
        self._controller.moverel(self, distance, speed)

    def freerun_start(self, speed):
        self._controller.freerun_start(self, speed)

    def freerun_stop(self):
        self._controller.freerun_stop(self)

    def get_feedback(self, parameter):
        return self._controller.get_feedback(self, parameter)[self]

    def get_drive_status(self):
        return self._controller.get_drive_status(self)[self]

    def get_axis_fault(self):
        return self._controller.get_axis_fault(self)[self]

    def get_axis_status(self):
        return self._controller.get_axis_status(self)[self]

    def get_position(self, program_position=True):
        return self._controller.get_positions(self, program_position=program_position)[self]

    def is_homed(self):
        return self._controller.is_homed(self)[self]

    def is_enabled(self):
        return self._controller.is_enabled(self)[self]

    def is_jogging(self):
        return self._controller.is_jogging(self)[self]

    def set_axis_dominant(self):
        return self._controller.set_axis_dominant(self)

    def set_axis_dependent(self):
        return self._controller.set_axis_dependent(self)


class MultiAxes:

    def __init__(self, axes, controller):
        self._axes = axes
        self._controller = controller

    def __repr__(self):
        return f'MultiAxes(({", ".join([str(axis) for axis in self._axes])}))'

    def __add__(self, other):
        if isinstance(other, Axis):
            if str(other) in [str(axis) for axis in self._axes]:
                raise ValueError('Cannot add multiple instances ' \
                                 'of the same axis')
            return MultiAxes((*self._axes, other), self._controller)
        elif isinstance(other, MultiAxes):
            if len(set([str(axis) for axis in self._axes]) &
                   set([str(axis) for axis in other._axes])) > 0:
                raise ValueError('Cannot add multiple instances ' \
                                 'of the same axis')
            return MultiAxes((*self._axes, *other._axes), self._controller)

        else:
            raise ValueError('Can only add Axis or MultiAxis objects')

    def __radd__(self, other):
        return MultiAxes((*self._axes, other), self._controller)

    def enable(self):
        self._controller.enable(self._axes)

    def disable(self):
        self._controller.disable(self._axes)

    def home(self):
        self._controller.home(self._axes)

    def set_home_speed(self, speed):
        self._controller.set_home_speed(self._axes, speed)

    def moveabs(self, positions, speed=None):
        self._controller.moveabs(self._axes, positions, speed)

    def moverel(self, distances, speed=None):
        self._controller.moverel(self._axes, distances, speed)

    def get_positions(self):
        return self._controller.get_positions(self._axes)

    def is_homed(self):
        return self._controller.is_homed(self._axes)

    def is_enabled(self):
        return self._controller.is_enabled(self._axes)


