import functools
from .axesdict import AxesDict
from .utils import is_container

def accept_multiple_axes(_func=None, *, transform_second_arg=False):
    '''
    This function is used to decorate a function that accepts an axes
    argument.
    If the argument is not a container, it is converted to a tuple with a
    single entry.
    Args:
        original_function (function): Function that accepts an axes argument
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
                    args = (arg,)
            if not is_container(axes):
                axes = (axes,)
                if len(args) > 0 and transform_second_arg:
                    if not is_container(args[0]):
                        args = list(args)
                        args[0] = (args[0],)
                        args = tuple(args)
            return original_function(self, axes, *args, **kwargs)

        return wrapper_function

    if _func is None:
        return decorator_accept_multiple_axes
    else:
        return decorator_accept_multiple_axes(_func)
