import smbus2
import json
import time

bus = smbus2.SMBus(1)
ADDR = 0x69


def main():
    while True:
        time.sleep(1)
        
        # Try to read sensor measurements
        msg_len = 0
        try:
            msg = bus.read_byte_data(ADDR, 0)
            msg_len = int(msg);
        except:
            continue
            
        # check if there bytes to read
        if msg_len < 1:
            continue
        
        # Read sensor measurements
        print(f"reading {msg_len} bytes")
        msg = smbus2.i2c_msg.read(ADDR, msg_len)
        bus.i2c_rdwr(msg)
        data = str(msg)
        print(data)


if __name__ == "__main__":
    main()
