import os
import socket
import time
import logging
from inspect import signature
import traceback
from contextlib import contextmanager

from .axis import Axis, MultiAxes
from .axesdict import AxesDict
from .status import AxisStatus, DriveStatus, AxisFault, TaskStatus, QueueStatus
from .errors import ConnectionError, CommandFaultError, CommandInvalidError
from .utils import str_to_num
from .decorators import accept_multiple_axes
from .socket import _TcpIpSocket, _TcpIpSocketDummy
from .constants import EOS_CHAR


DEFAULT_ADDRESS = '127.0.0.1'
DEFAULT_PORT = 8000

class A3200Controller:

    def __init__(self, dummy=False, address=DEFAULT_ADDRESS, port=DEFAULT_PORT,  redirect_output=False):
        if dummy:
            self._tcp_socket = _TcpIpSocketDummy()
        else:
            self._tcp_socket = _TcpIpSocket(port=port, address=address)

        self._buffered = False
        self._command_buffer = ''
        self.redirect_path = 'program.pgm'
        self.redirect_exists = False
        self._active_cam_tables = 0
        
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
        try:
            self._tcp_socket.connect()
        except TimeoutError:
            raise ConnectionError("Failed to connect to A3200 controller.")
        logging.debug('Connected')

    def disconnect(self):
        self._tcp_socket.disconnect()
        logging.debug('Disconnected')
        
    def send_command(self, command):
        if self._buffered:
            self._command_buffer += command  + EOS_CHAR
            return
        try:
            ret = self._tcp_socket.send(command)
            logging.debug(f'Sent command: {command}')
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

    def switch_task(self, task_id):
        self.send_command(f'~TASK {task_id}')

    def stop_task(self, task_id=None):
        command = '~STOPTASK'
        command += f' {task_id}' if task_id else ''
        self.send_command(command)
    

    def init_queue(self, task_id=1):
        command = '~INITQUEUE'
        command += f' {task_id}' if task_id else ''
        self.send_command(command)
    
    @contextmanager
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

    @contextmanager
    def buffer_commands(self):
        self._buffered = True
        yield
        self._buffered = False
        response = self.send_command(self._command_buffer)
        self._command_buffer = ''
        return response
                                        
    def acknowledgeall(self):
        self.send_command('ACKNOWLEDGEALL')
        
    def dwell(self, time):
        self.send_command(f'DWELL {time}')
     
    @accept_multiple_axes
    def enable(self, axes):
        data = A3200Controller.join_axes(axes)
        command = f'ENABLE {data}'
        self.send_command(command)

    @accept_multiple_axes
    def disable(self, axes):
        data = A3200Controller.join_axes(axes)
        command = f'DISABLE {data}'
        self.send_command(command)

    @accept_multiple_axes
    def home(self, axes):
        data = self.join_axes(axes)
        command = f'HOME {data}'
        self.send_command(command)
    
    @accept_multiple_axes
    def set_home_speed(self, axes, speed):
        data = A3200Controller.join_axes(axes)
        command = f'SETPARM {data} HomeSpeed {speed}'
        self.send_command(command)

    @accept_multiple_axes(transform_second_arg=True)
    def moveabs(self, axes, positions, speed=None, blocking=True):
        """
        Performs a coordinated absolute move on one or more axis by a specified distance.
        >>> controller.moveabs(('X', 'Y'), (10, 20))

        Parameters
        ----------
        axes: str or Axis
            The axis parameter supplies a tuple of axes to perform the moves on, where the axes are either represented
            as strings or Axis object.
        positions: float or tuple
            The positions parameter supplies a tuple of floats or ints with the same length as axes that indicates the
            absolute positions for each axis in the same order.
        speed: float
            Speed of the coordinated move, which applies only to dominant axes.
        blocking: bool
            If true, controller executes blocking command. If flase, controller executes an asynchronous command. In
            that case, the method returns immediately and a wait inpos command has to be issued in order to wait for
            the move to complete.
        Returns
        -------
        None
        """
        data = A3200Controller.join_axes(axes, positions)
        base_command = 'G90 G1' if blocking else 'MOVEABS'
        command = f'{base_command} {data}'
        if speed:
            command += f' F{speed}'
        self.send_command(command)

    @accept_multiple_axes(transform_second_arg=True)
    def moverel(self, axes, distances, speed=None, blocking=True):
        """
        Performs a coordinated incremental move on one or more axis by a specified distance.
        >>> controller.moverel(('X', 'Y'), (10, 20))

        Parameters
        ----------
        axes: str or Axis
            The axis parameter supplies a tuple of axes to perform the moves on, where the axes are either represented
            as strings or Axis object.
        distances: float or tuple
            The distances parameter supplies a tuple of floats or ints with the same length as axes that indicates the
            relative distances for each axis in the same order.
        speed: float
            Speed of the coordinated move, which applies only to dominant axes.
        blocking: bool
            If true, controller executes blocking command. If flase, controller executes an asynchronous command. In
            that case, the method returns immediately and a wait inpos command has to be issued in order to wait for
            the move to complete.

        Returns
        -------
        None
        """
        data = A3200Controller.join_axes(axes, positions=distances)
        base_command = 'G90 G1' if blocking else 'MOVEINC'
        command = f'{base_command} {data}'
        if speed:
            command += f' F{speed}'
        self.send_command(command)

    def wait_movedone(self, axis):
        """
        When you use the wait_movedone method, the program waits for the preceding motion to finish before it executes
        the next program line. The controller determines that the preceding motion is finished when the commanded
        velocity reaches zero. This command is different from the wait_inpos method. The wait_inpus method waits for the
        commanded velocity to reach zero and for the position error to reach a specified value. But, the wait_movedone
        method waits only for the commanded velocity to reach zero.

        Parameters
        ----------
        axis : str or Axis
            The axis parameter supplies an axis to perform the moves on, where the axis is either represented
            as a string or Axis object.
        """
        command = f'WAIT MOVEDONE {axis}'
        self.send_command(command)

    def wait_inpos(self, axis):
        """
        When you use the wait_inpos method, the program waits for the preceding motion to finish and reach its
        specified position before it executes the next program line. The controller determines that the preceding motion
        is finished when the commanded velocity reaches zero. Then, the controller determines that the preceding motion
        reached its specified position by using the values specified by the InPositionDistance Parameter and the
        InPositionTime Parameter. This command is different from the wait_movedone method. The wait_movedone method
        waits only for the commanded velocity to reach zero. The wait_inpos method waits for the commanded velocity to
        reach zero and for the position error to reach a specified value.

        Parameters
        ----------
        axis : str or Axis
            The axis parameter supplies an axis to perform the moves on, where the axis is either represented
            as a string or Axis object.
        """
        command = f'WAIT MOVEDONE {axis}'
        self.send_command(command)
    
    def set_speed(self, speed):
        command = f'F{speed}'
        self.send_command(command)
    
    @accept_multiple_axes
    def set_parm(self, axes, parameter, value):
        data = A3200Controller.join_axes(axes)
        command = f'SETPARM {data} {parameter} {value}'
        self.send_command(command)
        
    @accept_multiple_axes
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
        
    @accept_multiple_axes(transform_second_arg=True)
    def posoffset_set(self, axes, offsets):
        data = A3200Controller.join_axes(axes, positions=offsets)
        command = f'POSOFFSET SET {data}'
        self.send_command(command)
    
    @accept_multiple_axes(transform_second_arg=True)
    def posoffset_clear(self, axes):
        data = A3200Controller.join_axes(axes)
        command = f'POSOFFSET CLEAR {data}'
        self.send_command(command)
    
    @accept_multiple_axes
    def get_feedback(self, axes, parameter):
        template = '({}, {})'
        command = '~STATUS'
        for axis in axes:
            command += ' ' + template.format(axis, parameter)
        message = self.send_command(command)
        data = [str_to_num(value) for value in message.data.split(' ')]
        response = AxesDict(zip(axes, data))
        return response
    
    def get_positions(self, axes, program_position=True):
        parameter = 'ProgramPosition' if program_position else 'PositionFeedback'
        data = self.get_feedback(axes, parameter)
        return data
             
    def get_drive_status(self, axes):
        data = self.get_feedback(axes, 'DriveStatus')
        status = AxesDict({k: DriveStatus(v) for k, v in data.items()})
        return status
    
    def get_axis_status(self, axes):
        data = self.get_feedback(axes, 'AxisStatus')
        status = AxesDict({k: AxisStatus(v) for k, v in data.items()})
        return status
    
    def get_axis_fault(self, axes):
        data = self.get_feedback(axes, 'AxisFault')
        status = AxesDict({k: AxisFault(v) for k, v in data.items()})
        return status
    
    def get_task_state(self, task_id=1):
        template = '({}, {})'
        command = '~STATUS'
        if task_id is not None:
            command += template.format(task_id, 'TaskState')
        else:
            for i in range(5):
                command += template.format(i, 'TaskState')
        ret = [str_to_num(val) for val in 
               self.send_command(command).data.split(' ')]
        status = [TaskStatus(val) for val in ret]
        if task_id is not None:
            status = status[0]
        return status
    
    def get_queue_state(self, task_id=1):
        ret = self.send_command(f'~STATUS ({task_id}, QueueStatus)').data
        ret = str_to_num(ret)
        status = [TaskStatus(val) for val in ret]
        return status

    def load_camtable(self, master_axis, slave_axis, table_num=1, path=None, wrap=False):
        if not path:
            import tempfile
            path = os.path.join(tempfile.gettempdir(), 'cam_table.cam')
        command = f'LOADCAMTABLE {master_axis}, {table_num}, {slave_axis},' \
            f' 1, 1, "{path}"'
        command += ' WRAP' if wrap else ' NOWRAP'
        self.send_command(command)

    def start_camsync(self, slave_axis, table_num=0, sync_mode='relative'):
        if sync_mode == 'relative':
            sync_mode = 1
        elif sync_mode == 'absolute':
            sync_mode = 2
        elif sync_mode == 'veloctiy':
            sync_mode = 3
        else:
            raise ValueError('Sync mode muste be relative, absolute or velocity.')
        self.send_command(f'CAMSYNC {slave_axis}, {table_num}, {sync_mode}')

    def stop_camsync(self, slave_axis, table_num=0):
        self.send_command(f'CAMSYNC {slave_axis}, {table_num}, 0')

    def free_camtable(self, table_num=0):
        self.send_command(f'FREECAMTABLE {table_num}')
        
    @staticmethod
    def write_cam_table(xs, ys, path=None, master_multiplier=None, 
                        slave_multiplier=None, table_num=0):
        if not path:
            import tempfile
            path = os.path.join(tempfile.gettempdir(), f'cam_table_{table_num}.cam')
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
    def camming(self, xs, ys, master_axis, slave_axis, table_num=None, wrap=False,
                master_multiplier=None, slave_multiplier=None, sync_mode='relative'):
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
        table_num: int
            number of the cam table to occupy (0..99). Default 0.
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
        if table_num is None:
            table_num = self._active_cam_tables
            self._active_cam_tables += 1
        self.write_cam_table(xs, ys, master_multiplier=master_multiplier, 
                              slave_multiplier=slave_multiplier, table_num=table_num)
        self.load_camtable(master_axis, slave_axis, table_num=table_num, wrap=wrap)
        self.start_camsync(slave_axis, table_num=table_num, sync_mode=sync_mode)
        try:
            yield
        finally:
            self.stop_camsync(slave_axis, table_num=table_num)
            self.free_camtable(table_num=table_num)
            self._active_cam_tables -= 1

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
    
    def program_running(self, task_id=1):
        if TaskStatus.ProgramRunning in self.get_task_state(task_id=task_id):
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
        """
        Wraps the code in a velocity on and velocity off command to perform motion blending.

        Parameters
        ----------
        critical : bool
            Wraps the mode in an additional critical start and end command if specified True
        """
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
    
    def window_input(self, axis, input_, window_counter=1):
        self.controller.send_command(f'PSOWINDOW {axis} {window_counter} INPUT {input_}')
    
    def output_control(self, axis):
        self.controller.send_command(f'PSOOUTPUT {axis} CONTROL 0 1')
    
    def pulse(self, axis, time_on=1, time_off=1, cycles=1):
        self.controller.send_command(f'PSOPULSE {axis} TIME {time_on},'\
                                     f'{time_off} CYCLES {cycles}')
    
    def window_load(self, axis, value, window_counter=1):
        self.controller.send_command(f'PSOWINDOW {axis} {window_counter} LOAD {value}')
    
    def distance_fixed(self, axis, distance):
        self.controller.send_command(f'PSODISTANCE {axis} FIXED UNITSTOCOUNTS({axis}, {distance})')
    
    def window_range(self, axis, lower, upper, window_counter=1):
        command = f'PSOWINDOW {axis} {window_counter} RANGE UNITSTOCOUNTS({axis}, {lower}) UNITSTOCOUNTS({axis}, {upper})'
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

if __name__ == '__main__':
    controller = A3200Controller(dummy=False)
    X, Y, Z = controller.create_axes(('X', 'Y', 'Z'))
    a = AxesDict({X: 10, Y: 20, Z: 30})
    controller.connect()



        
