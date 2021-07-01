#!/usr/bin/env python3
from controller.build import slab_ctypes

slab_lib = slab_ctypes._libs["libslab_controller.so"]

print(dir(slab_ctypes))
print(dir(slab_lib))

ctx = slab_ctypes.SlabContext()
slab_lib.test_func()
# buf = slab_ctypes.test_func()
# print(buf)
