import smbus2
import json
import time

bus = smbus2.SMBus(1)
ADDR = 0x69


def main():
    read_bytes = list()
    while True:
        time.sleep(0.2)
        
        # Try to read sensor measurements
        msg_len = 0
        try:
            msg = bus.read_byte_data(ADDR, 0x00)
            msg_len = int(msg)
        except:
            continue
            
        # check if there bytes to read
        if (msg_len < 1) or (msg_len > 100):
            continue
        
        # Read sensor measurements
        print(f"reading {msg_len} bytes")
        msg = smbus2.i2c_msg.read(ADDR, msg_len)
        bus.i2c_rdwr(msg)
        data = str(msg)
        print(data)


if __name__ == "__main__":
    main()
