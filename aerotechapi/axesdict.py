from collections.abc import MutableMapping
from .axis import Axis

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
