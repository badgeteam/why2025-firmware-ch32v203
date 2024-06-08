import smbus, sys, time
bus = smbus.SMBus(3)

#from pynput.keyboard import Key, Controller
#keyboard = Controller()

def process_change(key_number, value):
    print(key_number, value)

def process_changes(curr, prev):
    for i in range(len(curr)):
        if curr[i] != prev[i]:
            process_change(i, curr[i])

prev = [0] * 5 * 4

bus.write_i2c_block_data(0x5f, 18, [0, 0, 0, 0])

while 1:
    data = bus.read_i2c_block_data(0x5f, 0, 22)
    bkp_data = bus.read_i2c_block_data(0x5f, 22, 21)
    bkp_data.extend(bus.read_i2c_block_data(0x5f, 22 + 21, 21))
    bkp_data.extend(bus.read_i2c_block_data(0x5f, 22 + 42, 21))
    bkp_data.extend(bus.read_i2c_block_data(0x5f, 22 + 63, 21))
    rtc = data[18] + (data[19] << 8) + (data[20] << 16) + (data[21] << 24)
    print(data[:18], bkp_data, rtc)

