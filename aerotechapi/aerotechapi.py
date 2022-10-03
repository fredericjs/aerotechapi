import os
import socket
import time
from inspect import signature
from collections.abc import MutableMapping
import traceback
import functools
from contextlib import contextmanager
from utils import is_container, str_to_num
from enum import Enum, IntFlag, auto


COMMAND_SUCCESS_CHAR = '%'
COMMAND_FAULT_CHAR = '#'
COMMAND_INVALID_CHAR = '!'
EOS_CHAR = '\n'


class AxisStatus(IntFlag):
    Homed = auto()
    Profiling = auto()
    WaitDone = auto()
    CommandValid = auto()
    Homing = auto()
    Enabling = auto()
    JogGenerating = auto()
    Jogging = auto()
    DrivePending = auto()
    DriveAbortPending = auto()
    TrajectoryFiltering = auto()
    IFOVEnabled = auto()
    NotVirtual = auto()
    CalEnabled1D = auto()
    CalEnabled2D = auto()
    MasterSlaveControl = auto()
    JoystickControl = auto()
    BacklashActive = auto()
    GainMappingEnabled = auto()
    Stability0 = auto()
    MotionBlocked = auto()
    MoveDone = auto()
    MotionClamped = auto()
    GantryAligned = auto()
    GantryRealigning = auto()
    Stability1 = auto()
    ThermoCompEnabled = auto()

    
class DriveStatus(IntFlag):
    Enabled = auto()
    CwEOTLimit = auto()
    CcwEOTLimit = auto()
    HomeLimit = auto()
    MarkerInput = auto()
    HallAInput = auto()
    HallBInput = auto()
    HallCInput = auto()
    SineEncoderError = auto()
    CosineEncoderError = auto()
    ESTOPInput = auto()
    BrakeOutput = auto()
    GalvoPowerCorrection = auto()
    NoMotorSupply = auto()
    CurrentClamp = auto()
    MarkerLatch = auto()
    PowerLimiting = auto()
    PSOHaltLatch = auto()
    HighResMode = auto()
    GalvoCalEnabled = auto()
    AutofocusActive = auto()
    ProgramFlash = auto()
    ProgramMXH = auto()
    ServoControl = auto()
    InPosition = auto()
    MoveActive = auto()
    AccelPhase = auto()
    DecelPhase = auto()
    EncoderClipping = auto()
    DualLoopActive = auto()
    InPosition2 = auto()
    
    
class AxisFault(IntFlag):
    PositionError = auto()
    OverCurrent = auto()
    CwEOTLimit = auto()
    CcwEOTLimit = auto()
    CwSoftLimit = auto()
    CcwSoftLimit = auto()
    AmplifierFault = auto()
    PositionFbk = auto()
    VelocityFbk = auto()
    HallFault = auto()
    MaxVelocity = auto()
    EstopFault = auto()
    VelocityError = auto()
    ProbeFault = auto()
    ExternalFault = auto()
    MotorTemp = auto()
    AmplifierTemp = auto()
    EncoderFault = auto()
    CommLost = auto()
    GantryMisalign = auto()
    FbkScalingFault = auto()
    MrkSearchFault = auto()
    SafeZoneFault = auto()
    InPosTimeout = auto()
    VoltageClamp = auto()
    PowerSupply = auto()
    MissedInterrupt = auto()
    Internal = auto()


class TaskStatus(Enum):
    Unavailable = auto()
    Inactive = auto()
    Idle = auto()
    ProgramReady = auto()
    ProgramRunning = auto()
    ProgramFeedheld = auto()
    ProgramPaused = auto()
    ProgramComplete = auto()
    Error  = auto()
    Queue = auto()
    
    
class QueueStatus(Enum):
    QueueActive = auto()
    QueueEmpty = auto()
    QueueFull = auto()
    QueueStarted = auto()
    QueuePaused = auto()
    QueueLargeProgram = auto()
    

class ConnectionError(Exception):
    
    def __init__(self, message):
        super().__init__(message)
        

class CommandFaultError(Exception):
    '''
    Exception is raised when the controller returns a command fault error 
    flag.
    '''
    def __init__(self, message):
        super().__init__(message)
        

class CommandInvalidError(Exception):
    '''
    Exception is raised when the controller returns a command invalid error
    flag.
    '''
    def __init__(self, message):
        super().__init__(message)
        

class Decorators:
    
    @staticmethod
    def accept_multiple_axes(_func=None, *, transform_second_arg=False):
        '''
        This function is used to decorate a function that accepts an axes 
        argument.
        If the argument is not a container, it is converted to a tuple with a 
        single entry.
        Args:
            original_function (function): Function that accepts an axes 
            argument
        Returns:
            function: Function with modified axes argument
        '''
        def decorator_accept_multiple_axes(original_function, *args):
            @functools.wraps(original_function)
            def wrapper_function(self, axes, *args, **kwargs):
                if isinstance(axes, AxesDict):
                    arg = axes.values
                    axes = axes.axes
                    if len(args) > 0:
                        args = (arg, *args)
                    else:
                        args = (arg, )
                if not is_container(axes):
                    axes = (axes, )
                    if len(args) > 0 and transform_second_arg:
                        if not is_container(args[0]):
                            args = list(args)
                            args[0] = (args[0], )
                            args = tuple(args)
                return original_function(self, axes, *args, **kwargs)
            return wrapper_function

        if _func is None:
            return decorator_accept_multiple_axes
        else:
            return decorator_accept_multiple_axes(_func)
    
    @staticmethod
    def validate_axes(original_function):
        '''
        This function is used to decorate a function that accepts an axes 
        argument. It validates the axes against a list of valid axes stored in 
        the A3200Controller Class. Raises a ValueError if any axis is not a 
        valid.
        Args:
            original_function (function): Function that accepts an axes 
            argument
        Returns:
            function: Function with same parameters if all axes are valid
        '''
        def wrapper_function(self, axes, *args, **kwargs):
            axes_valid = all([str(axis) in A3200Controller.valid_axes 
                              for axis in axes])
            if axes_valid:
                return original_function(self, axes, *args, **kwargs)
            else:
                raise ValueError('Invalid axis specified')
        return wrapper_function
    
    @staticmethod
    def validate_task(original_function):
        @functools.wraps(original_function)
        def wrapper_function(*args, **kwargs):
            arguments = signature(original_function).bind(*args, **kwargs).arguments
            if ('task_id' in arguments and 
                    (arguments['task_id'] not in A3200Controller.valid_tasks and
                     arguments['task_id'] != None)):
                raise ValueError('Invalid task specified')
            return original_function(*args, **kwargs)
        return wrapper_function


class SocketDummy:
    '''
    Tcp socket dummy Class that emulates the core functionality of a Tcp socket
    for testing purposes.
    '''
    
    def __init__(self, adress='127.0.0.1', port=8000, timeout=0.2):
        pass
    
    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_value, tb):
        if exc_type is not None:
            traceback.print_exception(exc_type, exc_value, tb)
        self.disconnect()
    
    def send(self, message, blocking=True):
        print(f'Sending message: {message}')
        return Message('%Test\n')
        
    def connect(self):
        print('connect')
        
    def disconnect(self):
        print('disconnect')
        

class Socket(socket.socket):
    '''
    Class that manages the TCP/IP connection to the socket running on the
    local host on port 8000. 
    '''

    def __init__(self, address='127.0.0.1', port=8000, timeout=0.2):
        super().__init__(socket.AF_INET, socket.SOCK_STREAM)
        self._server_address = address
        self._port = port
        self.settimeout(timeout)

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_value, tb):
        if exc_type is not None:
            traceback.print_exception(exc_type, exc_value, tb)
        self.disconnect()
    
    def connect(self):
        super().connect((self._server_address, self._port))

    def disconnect(self):
        self.close()

    def send(self, message):
        if not message.endswith(EOS_CHAR):
            message += EOS_CHAR
        super().send(message.encode())
        while True:
            ret = self._receive()
            if ret == None:
                continue
            else:
                break
        return ret[-1]

    def _receive(self):
        try:
            ret = self.recv(1024).decode()
            messages = [Message(msg) for msg in ret.split('\n') if msg != '']
            return messages
        except socket.timeout:
            return None
        

class A3200Controller:

    valid_axes = ['X', 'Y', 'Z', 'A', 'U']
    valid_tasks = (0, 1, 2, 3, 4)

    def __init__(self, address='127.0.0.1', port=8000, timeout=0.2, dummy=False, redirect_output=False):
        if dummy:
            self._tcp_socket = SocketDummy()
        else:
            self._tcp_socket = Socket(address=address, port=port, timeout=timeout)

        self.redirect_path = 'program.pgm'
        self.redirect_exists = False
        
        self.PSO = PSO(self)
            
    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_value, tb):
        if exc_type is not None:
            traceback.print_exception(exc_type, exc_value, tb)
        self.disconnect()
        
    def set_redirect_path(self, path):
        self.redirect_path = path
        with open(self.redirect_path, 'w') as f:
            pass
        self.send_command = self.redirect_command
    
    @staticmethod
    def join_axes(axes, positions=None):
        if positions:
            return ' '.join([''.join([str(val) for val in axpos]) 
                             for axpos in zip(axes, positions)])
        else:
            return ' '.join([str(axis) for axis in axes])
    
    def connect(self):
        self._tcp_socket.connect()

    def disconnect(self):
        self._tcp_socket.disconnect()
        
    def send_command(self, command):
        try:
            ret = self._tcp_socket.send(command)
        except socket.timeout:
            raise ConnectionError('Controller is not connected.')
        if ret.status == 'CommandSuccess':
            return ret
        elif ret.status == 'CommandInvalid':
            raise CommandInvalidError('The command is not syntactically '\
                                      'correct')
        elif ret.status == 'CommandFault':
            error = self.get_last_error()
            if error:
                raise CommandFaultError(error)
            else:
                raise CommandFaultError('Command could not execute '\
                                        'successfully')
                    
    def redirect_command(self, command):
        with open(self.redirect_path, 'a') as file:
            file.write(command + '\n') 
            
    def get_last_error(self):
        ret = self.send_command('~LASTERROR')
        if ret != None and ret.status == 'CommandSuccess':
            return ret.data

    def reset(self):
        self.send_command('~RESETCONTROLLER')
    
    @Decorators.validate_task
    def switch_task(self, task_id):
        self.send_command(f'~TASK {task_id}')
      
    @Decorators.validate_task
    def stop_task(self, task_id=None):
        command = '~STOPTASK'
        command += f' {task_id}' if task_id else ''
        self.send_command(command)
    
    @Decorators.validate_task
    def init_queue(self, task_id=1):
        command = '~INITQUEUE'
        command += f' {task_id}' if task_id else ''
        self.send_command(command)
    
    @contextmanager
    @Decorators.validate_task
    def queue_mode(self, task_id=1, block=True):
        self.init_queue(task_id=task_id)
        self.switch_task(task_id)
        try:
            yield
        finally:
            if block:
                while True:
                    state = self.get_queue_state(task_id)
                    if 'Queue Buffer Empty' in state:
                        break
                    time.sleep(0.1)
                self.stop_task(task_id=task_id)        

                                        
    def acknowledgeall(self):
        self.send_command('ACKNOWLEDGEALL')
        
    def dwell(self, time):
        self.send_command(f'DWELL {time}')
     
    @Decorators.accept_multiple_axes
    @Decorators.validate_axes
    def enable(self, axes):
        data = A3200Controller.join_axes(axes)
        command = f'ENABLE {data}'
        self.send_command(command)

    @Decorators.accept_multiple_axes
    @Decorators.validate_axes
    def disable(self, axes):
        data = A3200Controller.join_axes(axes)
        command = f'DISABLE {data}'
        self.send_command(command)

    @Decorators.accept_multiple_axes
    @Decorators.validate_axes
    def home(self, axes):
        data = self.join_axes(axes)
        command = f'HOME {data}'
        self.send_command(command)
    
    @Decorators.accept_multiple_axes
    @Decorators.validate_axes
    def set_home_speed(self, axes, speed):
        data = A3200Controller.join_axes(axes)
        command = f'SETPARM {data} HomeSpeed {speed}'
        self.send_command(command)

    @Decorators.accept_multiple_axes(transform_second_arg=True)
    @Decorators.validate_axes
    def moveabs(self, axes, positions, speed=None):
        data = A3200Controller.join_axes(axes, positions)
        command = f'G90 G1 {data}'
        if speed:
            command += f' F{speed}'
        self.send_command(command)

    @Decorators.accept_multiple_axes(transform_second_arg=True)
    @Decorators.validate_axes
    def moverel(self, axes, distances, speed=None):
        data = A3200Controller.join_axes(axes, positions=distances)
        command = f'G91 G1 {data}'
        if speed:
            command += f' F{speed}'
        self.send_command(command)
    
    def set_speed(self, speed):
        command = f'F{speed}'
        self.send_command(command)
    
    @Decorators.accept_multiple_axes
    @Decorators.validate_axes
    def set_parm(self, axes, parameter, value):
        data = A3200Controller.join_axes(axes)
        command = f'SETPARM {data} {parameter} {value}'
        self.send_command(command)
        
    @Decorators.accept_multiple_axes
    @Decorators.validate_axes
    def read_parm(self, axes, parameter):
        result = AxesDict()
        for axis in axes:
            command = f'$global[0] = {parameter}.{axis}'
            self.send_command(command)
            result[axis] = self.send_command('~GETVARIABLE $global[0]').data
        return result

    def freerun_start(self, axis, speed):
        command = f'FREERUN {axis} {speed}'
        self.send_command(command)

    def freerun_stop(self, axis):
        command = f'FREERUN {axis} STOP'
        self.send_command(command)
        
    @Decorators.accept_multiple_axes(transform_second_arg=True)
    @Decorators.validate_axes
    def posoffset_set(self, axes, offsets):
        data = A3200Controller.join_axes(axes, positions=offsets)
        command = f'POSOFFSET SET {data}'
        self.send_command(command)
    
    @Decorators.accept_multiple_axes(transform_second_arg=True)
    @Decorators.validate_axes
    def posoffset_clear(self, axes):
        data = A3200Controller.join_axes(axes)
        command = f'POSOFFSET CLEAR {data}'
        self.send_command(command)
    
    @Decorators.accept_multiple_axes
    @Decorators.validate_axes
    def get_feedback(self, axes, parameter):
        template = '({}, {})'
        command = '~STATUS'
        for axis in axes:
            command += ' ' + template.format(axis, parameter)
        message = self.send_command(command)
        data = [str_to_num(value) for value in message.data.split(' ')]
        response = AxesDict(zip(axes, data))
        return response
    
    def get_positions(self, axes):
        data = self.get_feedback(axes, 'PositionFeedback')
        return data
             
    def get_drive_status(self, axes):
        data = self.get_feedback(axes, 'DriveStatus')
        data = {k: DriveStatus(v) for k, v in data.items()}
        return DriveStatus(data)
    
    def get_axis_status(self, axes):
        data = self.get_feedback(axes, 'AxisStatus')
        data = {k: AxisStatus(v) for k, v in data.items()}
        return AxisStatus(data)
    
    def get_axis_fault(self, axes):
        data = self.get_feedback(axes, 'AxisFault')
        data = {k: AxisFault(v) for k, v in data.items()}
        return data
    
    def get_task_state(self):
        template = '({}, {})'
        command = '~STATUS'
        for i in range(5):
            command += template.format(i, 'TaskState')
        ret = [str_to_num(val) for val in 
               self.send_command(command).data.split(' ')]
        status = [TaskStatus(val) for val in ret]
        return status
    
    def get_queue_state(self, task_id=1):
        ret = self.send_command(f'~STATUS ({task_id}, QueueStatus)').data
        ret = str_to_num(ret)
        status = [TaskStatus(val) for val in ret]
        return status

    def load_camtable(self, master_axis, slave_axis, path=None, wrap=False):
        if not path:
            import tempfile
            path = os.path.join(tempfile.gettempdir(), 'cam_table.cam')
        command = f'LOADCAMTABLE {master_axis}, 1, {slave_axis},' \
            f' 1, 1, "{path}"'
        command += ' WRAP' if wrap else ' NOWRAP'
        self.send_command(command)

    def start_camsync(self, slave_axis):
        self.send_command(f'CAMSYNC {slave_axis}, 1, 1')

    def stop_camsync(self, slave_axis):
        self.send_command(f'CAMSYNC {slave_axis}, 1, 0')

    def free_camtable(self):
        self.send_command('FREECAMTABLE 1')
        
    @staticmethod
    def write_cam_table(xs, ys, path=None, master_multiplier=None, 
                        slave_multiplier=None):
        if not path:
            import tempfile
            path = os.path.join(tempfile.gettempdir(), 'cam_table.cam')
        with open(path, 'w') as file:
            if not len(xs) == len(ys):
                raise ValueError('xs and ys must be the same size')
            num_points = len(xs)
            file.write(f'Number of Points   {num_points}\n')
            if master_multiplier:
                file.write('Master Units (PRIMARY/{master_multiplier})\n')
            else:
                file.write('Master Units (PRIMARY)\n')
            if slave_multiplier:
                file.write('Slave Units (PRIMARY/{slave_multiplier})\n')
            else:
                file.write('Slave Units (PRIMARY)\n')
            for i, (x, y) in enumerate(zip(xs, ys)):
                file.write(f'{i:04d} {x:.4f} {y:.4f}\n')
                
    @contextmanager
    def camming(self, xs, ys, master_axis, slave_axis, wrap=False,
                master_multiplier=None, slave_multiplier=None):
        '''
        Convenience context manager that enables camming motion for a block
        of motion commands. Internally calls write_cam_table and load_camsync
        before the block of code is executed and stop_camsync and 
        free_camtable afterwards.

        Parameters
        ----------
        xs : list-like
            contains master axis positions.
        ys : list-like
            contains slave axis positions corresponding to xs.
        master_axis : Axis, str
            master axis for the camming motion.
        slave_axis : Axis, str
            slave axis for the camming motion.
        wrap : bool, optional
            Specify whether the camtable wraps around after exceeding its 
            maximum value. The default is False.
        master_multiplier : float, optional
            DESCRIPTION. The default is None.
        slave_multiplier : float, optional
            DESCRIPTION. The default is None.

        Returns
        -------
        None.

        '''
        
        self.write_cam_table(xs, ys, master_multiplier=master_multiplier, 
                              slave_multiplier=slave_multiplier)
        self.load_camtable(master_axis, slave_axis, wrap=wrap)
        self.start_camsync(slave_axis)
        try:
            yield
        finally:
            self.stop_camsync(slave_axis)
            self.free_camtable()

    def set_axis_dominant(self, axis):
        self.send_command(f'SETPARM {axis} AxisType 0')

    def set_axis_dependent(self, axis):
        self.send_command(f'SETPARM {axis} AxisType 1')

    def create_axes(self, axes=('X', 'Y', 'Z')):
        return [Axis(ax, self) for ax in axes]

    def is_enabled(self, axes):
        response = self.get_drive_status(axes)
        for axis, status in response.items():
            if DriveStatus.Enabled in status:
                response[axis] = True
            else:
                response[axis] = False
        return response

    def is_homed(self, axes):
        response = self.get_axis_status(axes)
        for axis, status in response.items():
            if AxisStatus.Homed in status:
                response[axis] = True
            else:
                response[axis] = False
        return response

    def is_jogging(self, axes):
        response = self.get_axis_status(axes)
        for axis, status in response.items():
            if AxisStatus.Jogging in status:
                response[axis] = True
            else:
                response[axis] = False
        return response
    
    def program_running(self):
        if 'Program Running' in self.get_task_state():
            return True
        return False
    
    def velocity_on(self):
        self.send_command('VELOCITY ON')
        
    def velocity_off(self):
        self.send_command('VELOCITY OFF')
        
    def critical_start(self):
        self.send_command('CRITICAL START')
        
    def critical_end(self):
        self.send_command('CRITICAL END')
        
    @contextmanager
    def velocity_on_mode(self, critical=False):
        self.velocity_on()
        if critical:
            self.critical_start()

        try:
            yield
        finally:
            if critical:
                self.critical_end()
            self.velocity_off()
        
        
class PSO:
    
    def __init__(self, controller):
        self.controller = controller

    def arm(self, axis):
        self.controller.send_command(f'PSOCONTROL {axis} ARM')
    
    def off(self, axis):
        self.controller.send_command(f'PSOCONTROL {axis} OFF')
    
    def reset(self, axis):
        self.controller.send_command(f'PSOCONTROL {axis} RESET')
    
    def track_input(self, axis, input_):
        self.controller.send_command(f'PSOTRACK {axis} INPUT {input_}')
    
    def window_input(self, axis, input_):
        self.controller.send_command(f'PSOWINDOW {axis} 1 INPUT {input_}')
    
    def output_control(self, axis):
        self.controller.send_command(f'PSOOUTPUT {axis} CONTROL 0 1')
    
    def pulse(self, axis, time_on=1, time_off=1, cycles=1):
        self.controller.send_command(f'PSOPULSE {axis} TIME {time_on},'\
                                     f'{time_off} CYCLES {cycles}')
    
    def window_load(self, axis, value):
        self.controller.send_command(f'PSOWINDOW {axis} 1 LOAD {value}')
    
    def distance_fixed(self, axis, distance):
        self.controller.send_command(f'PSODISTANCE {axis} FIXED'\
                                     f'UNITSTOCOUNTS({axis}, {distance})')
    
    def window_range(self, axis, lower, upper):
        command = f'PSOWINDOW {axis} 1 RANGE UNITSTOCOUNTS({axis}, {lower}),'\
            f'UNITSTOCOUNTS({axis}, {upper})'
        self.controller.send_command(command)

    def output_pulse(self, axis):
        self.controller.send_command(f'PSOOUTPUT {axis} PULSE')
    
    def output_pulse_window_mask(self, axis):
        self.controller.send_command(f'PSOOUTPUT {axis} PULSE WINDOW MASK')
    
    def init(self, axis):
        self.reset(axis)
        self.track_input(axis, 0)
        self.window_input(axis, 0)
        self.output_control(axis)
        self.pulse(axis)
        
    def load_window(self, axis, pulse_sep, lower, upper):
        self.window_load(axis, 0)
        self.distance_fixed(axis, pulse_sep)
        self.window_range(axis, lower, upper)
        self.output_pulse_window_mask(axis)

            
            
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

    def get_position(self):
        return self._controller.get_positions(self)[self]
    
    def is_homed(self):
        return self._controller.is_homed(self)[self]
    
    def is_enabled(self):
        return self._controller.is_enabled(self)[self]

    def is_jogging(self):
        return self._controller.is_jogging(self)[self]
    
    
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

  
class Message:

    command_parameters = {COMMAND_SUCCESS_CHAR: 'CommandSuccess',
                          COMMAND_INVALID_CHAR: 'CommandInvalid',
                          COMMAND_FAULT_CHAR: 'CommandFault'}

    def __init__(self, msg_str):
        self.data, self.status = self.deconstruct_msg_str(msg_str)

    def __repr__(self):
        if self.data:
            return f'{self.status}, {self.data}'
        else:
            return f'{self.status}'

    def __str__(self):
        if self.data:
            return self.data
        else:
            return self.status

    def deconstruct_msg_str(self, msg_str):
        if msg_str.endswith(EOS_CHAR):
            msg_str = msg_str.rstrip(EOS_CHAR)
        data = msg_str[1:]
        command_parameter = self.command_parameters[msg_str[0]]
        return data, command_parameter
    
    
class AxesDict(MutableMapping):
    
    def __init__(self, *args, **kwargs):
        self._storage = dict(*args, **kwargs)
        self._storage = {str(k) if isinstance(k, Axis) else k: v 
                         for k, v in self._storage.items()}
    
    @classmethod
    def from_dict(cls, dict_):
        return cls(dict_.items())
    
    def __getitem__(self, key):
        if isinstance(key, Axis):
            key = str(key)
        return self._storage[key]
    
    def __setitem__(self, key, item):
        if isinstance(key, Axis):
            key = str(key)
        self._storage[key] = item
        
    def __delitem__(self, key):
        if isinstance(key, Axis):
            key = str(key)
        del self._storage[key]
        
    def __iter__(self):
        return iter(self._storage)
    
    def __len__(self):
        return len(self._storage)
    
    def __repr__(self):
        return f"{type(self).__name__}({self._storage})"
    
    @property
    def axes(self):
        return tuple(self._storage.keys())

    @property
    def values(self):
        return tuple(self._storage.values())
     
        
if __name__ == '__main__':
    controller = A3200Controller(dummy=False)
    X, Y, Z = controller.create_axes(('X', 'Y', 'Z'))
    a = AxesDict({X: 10, Y: 20, Z: 30})
    controller.connect()



        
