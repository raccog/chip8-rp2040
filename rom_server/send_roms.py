from pathlib import Path
import serial
import os
import time


class Rom:
    def __init__(self, path):
        self.path = path
        with open(path, 'rb') as f:
            self.data = f.read()


METADATA_BYTES = 6
rom_count = 0
roms = []
flash_size = 2


for path in Path('.').glob('**/*.ch8'):
    rom = Rom(path)
    roms.append(rom)
    rom_count += 1
    flash_size += len(str(rom.path)) + 1 + METADATA_BYTES + len(rom.data)


print(f"writing roms of size {flash_size} over uart")
ser = serial.Serial('/dev/serial0', baudrate=115200)
ser.flush()
ser.write(flash_size.to_bytes(4, 'little'))
ser.flush()
ser.write(rom_count.to_bytes(2, 'little'))


flash_offset = METADATA_BYTES * rom_count + 2


for rom in roms:
    rom_size = len(rom.data)
    print(f'rom size is {rom_size}')
    print(f'offset is {flash_offset}')
    ser.flush()
    ser.write(rom_size.to_bytes(2, 'little'))
    ser.flush()
    ser.write(flash_offset.to_bytes(4, 'little'))
    flash_offset += rom_size + len(str(rom.path)) + 1


for rom in roms:
    ser.flush()
    ser.write(str(rom.path).encode('ascii'))
    ser.flush()
    ser.write(b'\x00')
    for i in range(len(rom.data) // 32):
        ser.flush()
        ser.write(rom.data[i * 32:i*32 + 32])
    ser.flush()
    if len(rom.data) % 32 != 0:
        ser.write(rom.data[len(rom.data) // 32 * 32:])
        ser.flush()


while ser.out_waiting > 0:
    time.sleep(0.001)
ser.close()
