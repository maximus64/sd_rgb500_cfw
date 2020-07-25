#!/usr/bin/env python3
import serial
import time
import struct
import sys

ser = serial.Serial('/dev/ttyACM0', 115200)

def dump_memory(address):
    ser.write(struct.pack(">I", address))
    buff = b''
    while True:
        if b'<<DONE>>' in buff:
            break
        data = ser.read(ser.inWaiting())
        sys.stdout.buffer.write(data)
        buff += data
    start_magic = b'<<STARTDUMP>>'
    end_magic = b'<<DONE>>'
    start = buff.find(start_magic) + len(start_magic)
    end = buff.find(end_magic, start)
    return buff[start:end]


fout = open('out.bin', 'wb')

for addr in range(0x08000000, 0x08000000 + 0x40000, 256):
    print("Reading Address: %08x" % addr)
    out = dump_memory(addr)
    print(out)
    binout = bytes.fromhex(out.decode("utf-8"))
    assert(len(binout) == 256)
    fout.write(binout)
    fout.flush()

fout.close()
