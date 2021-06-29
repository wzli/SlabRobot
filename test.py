#!/usr/bin/env python3
from controller.build import slab_ctypes
import ctypes
import numpy as np

lib = np.ctypeslib.load_library('libslab_controller', 'controller/build')
print(dir(slab_ctypes))
print(dir(lib))

ctx = slab_ctypes.SlabContext()

slab_ctypes._libs["libslab_controller.so"].test_func()
#buf = slab_ctypes.test_func()
#print(buf)
