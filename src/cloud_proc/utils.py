from __future__ import absolute_import, division, print_function
from importlib import import_module

__all__ = [
    'col',
    'resolve_class'
]


def col(x):
    return x.reshape((x.size, 1))


def resolve_class(name):
    i = name.rfind('.')
    if i < 0:
        return eval(name)
    mod_name = name[:i]
    # TODO: Check multi package path.
    mod = import_module(mod_name)
    return eval(name, {}, {mod_name: mod})
