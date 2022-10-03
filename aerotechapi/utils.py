from collections.abc import Iterable


def is_container(arg):
    '''Returns True if argument is an iterable but not a string, else False'''
    return isinstance(arg, Iterable) and not isinstance(arg, str)

def str_to_num(string):
    '''
    This function checks wether a string should be converted to int
    or to float.
    Returns either an int, a float or the original string, depending on
    which conversion is possible.
    '''
    try:
        result = float(string.replace(',', '.'))
        if result.is_integer():
            result = int(string.replace(',', '.'))
    except ValueError:
        result = string
    return result

def accept_strings(f):
    '''
    Decorator function that makes functions accept numerical values 
    as strings and converts them to float or int
    '''
    def wrapper(self, *args, **kwargs):
        _temp = []
        for arg in args:
            if isinstance(arg, str):
                if '.' in arg:
                    _temp.append(int(float(arg)))
                else:
                    _temp.append(int(arg))
            elif isinstance(arg, float):
                _temp.append(float)
            elif isinstance(arg, int):
                _temp.append(arg)
            else:
                raise ValueError  
        args = tuple(_temp)
        return f(self, *args, **kwargs)
    return wrapper

def linspace(start, stop, num=50):
    '''
    Creates a list of evenly spaced numbers
    Args:
        start (int, float): First value of the sequence
        stop (int, float): Last value of the sequence
    Kwargs:
        stop (int, float): Number of values in the sequence, defaults to 50
    Returns:
        linspace (list)
    '''
    spacing = (stop - start) / (num - 1)
    return [start + spacing * i for i in range(num)]

def vectorize(func):
    '''
    Decorator function that transforms a function so that it accepts list-like
    objects. The returned function maps the input function to all values of
    the list-like parameters.
    '''
    def wrapper_func(*args, **kwargs):
        iterable_args = []
        for arg in args:
            if isinstance(arg, Iterable):
                iterable_args.append(arg)                
        if not all([len(arg) == len(iterable_args[0]) for arg in iterable_args]):
            raise ValueError('Array-like arguments must have the same length')
        return [func(*values) for values in zip(*iterable_args)]
    return wrapper_func
    