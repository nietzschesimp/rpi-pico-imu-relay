import smbus2
import json
import time

bus = smbus2.SMBus(1)
ADDR = 0x69


def main():
    read_bytes = list()
    while True:
        time.sleep(0.25)
        
        # Try to read sensor measurements
        msg_len = 0
        try:
            msg = bus.read_i2c_block_data(ADDR, 0, 1)
            msg_len = int(msg[0])
        except:
            continue
            
        # check if there bytes to read
        if msg_len < 1:
            continue
        
        # Read sensor measurements in blocks
        print(f"reading {msg_len} bytes")
        index = 0
        block_size = 32
        blocks = msg_len // block_size
        data = list()
        for block_no in range(blocks):
            data += bus.read_i2c_block_data(ADDR, index, 32)
            index += block_size

        # Grab remaining bytes
        remainder = msg_len % block_size
        if remainder > 0:
            data += bus.read_i2c_block_data(ADDR, index, remainder)

        # Correct for random MSB bit flip
        data_correct = [(x & 0x7F) for x in data]
        msg = bytearray(data_correct).decode()
        print(msg)


if __name__ == "__main__":
    main()
