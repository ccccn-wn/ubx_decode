import struct


def bytes_to_Doublefloat(bytes):
    return struct.unpack(f'<d', bytes)[0]
def bytes_to_float(bytes):
    return struct.unpack(f'<f', bytes)[0]

def bytes_to_U32(bytes):
    return struct.unpack(f'<I', bytes)[0]

def bytes_to_U16(bytes):
    return struct.unpack(f'<H', bytes)[0]

def bytes_to_U8(bytes):
    return struct.unpack(f'B', bytes)[0]

