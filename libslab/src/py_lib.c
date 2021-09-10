// This file exposes inline functions to python via ctypes
// It should not be included in normal C code.
#include "slab.h"

#define SERIALIZATION_IMPL(MSG)                                                              \
    int MSG##__serialize(const MSG* msg, uint8_t* buf) { return MSG##_serialize(msg, buf); } \
    int MSG##__deserialize(MSG* msg, const uint8_t* buf) { return MSG##_deserialize(msg, buf); }

SERIALIZATION_IMPL(GamepadMsg)
