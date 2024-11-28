import machine, neopixel, time
from machine import UART, Pin
import struct


kRegProductId = 0x00
kRegHwVersion = 0x01
kRegFwVersion = 0x02
kRegScalarLinear = 0x04
kRegScalarAngular = 0x05
kRegImuCalib = 0x06
kRegReset = 0x07
kRegSignalProcess = 0x0E
kRegSelfTest = 0x0F
kRegOffXL = 0x10
kRegOffXH = 0x11
kRegOffYL = 0x12
kRegOffYH = 0x13
kRegOffHL = 0x14
kRegOffHH = 0x15
kRegStatus = 0x1F
kRegPosXL = 0x20
kRegPosXH = 0x21
kRegPosYL = 0x22
kRegPosYH = 0x23
kRegPosHL = 0x24
kRegPosHH = 0x25
kRegVelXL = 0x26
kRegVelXH = 0x27
kRegVelYL = 0x28
kRegVelYH = 0x29
kRegVelHL = 0x2A
kRegVelHH = 0x2B
kRegAccXL = 0x2C
kRegAccXH = 0x2D
kRegAccYL = 0x2E
kRegAccYH = 0x2F
kRegAccHL = 0x30
kRegAccHH = 0x31
kRegPosStdXL = 0x32
kRegPosStdXH = 0x33
kRegPosStdYL = 0x34
kRegPosStdYH = 0x35
kRegPosStdHL = 0x36
kRegPosStdHH = 0x37
kRegVelStdXL = 0x38
kRegVelStdXH = 0x39
kRegVelStdYL = 0x3A
kRegVelStdYH = 0x3B
kRegVelStdHL = 0x3C
kRegVelStdHH = 0x3D
kRegAccStdXL = 0x3E
kRegAccStdXH = 0x3F
kRegAccStdYL = 0x40
kRegAccStdYH = 0x41
kRegAccStdHL = 0x42
kRegAccStdHH = 0x43

kRadianToDegree = 180.0 / 3.14159
kDegreeToRadian = 3.14159 / 180.0
kMeterToInch = 39.3701
kInchToMeter = 1.0 / kMeterToInch

kMeterToInt16 = 32768.0 / 10.0
kInt16ToMeter = 1.0 / kMeterToInt16
kInt16ToInch = kInt16ToMeter * kMeterToInch
kInchToInt16 = 1.0 / kInt16ToInch

kRadToInt16 = 32768.0 / 3.14159
kInt16ToRad = 1.0 / kRadToInt16
kInt16ToDeg = kInt16ToRad * kRadianToDegree
kDegToInt16 = 1.0 / kInt16ToDeg

kMpsToInt16 = 32768.0 / 5.0
kInt16ToMps = 1.0 / kMpsToInt16

kRpsToInt16 = 32768.0 / (2000.0 * kDegreeToRadian)
kInt16ToRps = 1.0 / kRpsToInt16



pixel_pin = 16
pixel = neopixel.NeoPixel(machine.Pin(pixel_pin), 1)
pixel[0] = (50, 0, 0)
pixel.write()
dir = machine.Pin(10, machine.Pin.OUT)
led1 = machine.Pin(26, machine.Pin.OUT)

time.sleep(1)



i2c=machine.I2C(1,sda=machine.Pin(26), scl=machine.Pin(27), freq=400000)
def read_byte(addr):
    return int(i2c.readfrom_mem(0x17, addr, 1)[0])
def read_block(addr, length):
    return [int(i) for i in i2c.readfrom_mem(0x17, addr, length)]
def write_byte(addr, byte):
    # i2c.writeto_mem(0x17, addr, bytes([byte]))
    i2c.writeto_mem(0x17, addr, byte.to_bytes(1, 'little'))
def write_block(addr, data):
    i2c.writeto_mem(0x17, addr, bytes(data))

print(hex(read_byte(kRegProductId)))


(hwVersion, fwVersion) = read_block(kRegHwVersion, 2)

# Extract the major and minor version numbers
hwMajor = (hwVersion >> 4) & 0x0F
hwMinor = hwVersion & 0x0F
fwMajor = (fwVersion >> 4) & 0x0F
fwMinor = fwVersion & 0x0F

# Print the hardware and firmware version
print("OTOS Hardware Version: v{}.{}".format(hwMajor, hwMinor))
print("OTOS Firmware Version: v{}.{}".format(fwMajor, fwMinor))


def calibrate():
    write_byte(kRegImuCalib, 255) # 255 is calibration sample count
    time.sleep(0.003)
    for numAttempts in range(255, 0, -1):
        # Read the gryo calibration register value
        calibrationValue = read_byte(kRegImuCalib)

        # Check if calibration is done
        if calibrationValue == 0:
            return True

        # Give a short delay between reads. As of firmware v1.0, samples take
        # 2.4ms each, so 3ms should guarantee the next sample is done. This
        # also ensures the max attempts is not exceeded in normal operation
        time.sleep(0.003)
    return False
def reset_tracking():
    write_byte(kRegReset, 0x01) # Reset Tracking

def regs_to_pose(rawData, rawToXY, rawToH):
    # Store raw data
    rawX = (rawData[1] << 8) | rawData[0]
    rawY = (rawData[3] << 8) | rawData[2]
    rawH = (rawData[5] << 8) | rawData[4]

    # Convert raw data to signed 16-bit integers
    if rawX > 32767:
        rawX = rawX - 65536
    if rawY > 32767:
        rawY = rawY - 65536
    if rawH > 32767:
        rawH = rawH - 65536

    # Store in pose and convert to units
    x = rawX * rawToXY
    y = rawY * rawToXY
    h = rawH * rawToH
    return (x, y, h)

def pose_to_regs(pose, xyToRaw, hToRaw):
    rawX = int(pose[0] * xyToRaw)
    rawY = int(pose[1] * xyToRaw)
    rawH = int(pose[2] * hToRaw)
    rawData = [0] * 6

    rawData[0] = rawX & 0xFF
    rawData[1] = (rawX >> 8) & 0xFF

    rawData[2] = rawY & 0xFF
    rawData[3] = (rawY >> 8) & 0xFF

    rawData[4] = rawH & 0xFF
    rawData[5] = (rawH >> 8) & 0xFF

    return rawData

def read_pose_regs(reg, xyToRaw, hToRaw):
    data = read_block(reg, 6)
    return regs_to_pose(data, xyToRaw, hToRaw)
def write_pose_regs(reg, pose, xyToRaw, hToRaw):
    data = pose_to_regs(pose, xyToRaw, hToRaw)
    write_block(reg, data)

def get_position():
    return read_pose_regs(kRegPosXL, kInt16ToInch, kInt16ToDeg)

def set_offset(pose):
    write_pose_regs(kRegOffXL, pose, kInchToInt16, kDegToInt16)

def set_position(pose):
    write_pose_regs(kRegPosXL, pose, kInchToInt16, kDegToInt16)

def set_linear_scaler(scaler):
    rawScaler = int((scaler - 1.0) * 1000 + 0.5)
    write_byte(kRegScalarLinear, rawScaler)

def set_angular_scaler(scaler):
    rawScaler = int((scaler - 1.0) * 1000 + 0.5)
    write_byte(kRegScalarAngular, rawScaler)

uart1 = UART(1, baudrate=921600, tx=Pin(8), rx=Pin(9))
uart1.init(bits=8, parity=None, stop=1)

i = 0
while True:
    dir.value(0)
    # uart1.write(bytes("Hello") + bytes((i, )))
    # i += 1
    # time.sleep(0.1)

    if uart1.any():
    # if True:
        try:
            readVal = uart1.read(1)[0]
            # readVal = 0x01
            # print("readVal:", readVal)
            if readVal == 0x01:
                length = uart1.read(1)[0]
                # print("Length:", length)
                buffer = []
                while len(buffer) < length:
                    buffer.extend(uart1.read(1))
                # buffer = [0x50, 0x00, 0x50, 0x00]
                # crc = uart1.read(2)
                crc = buffer[-3:]
                endbyte = buffer[-1]
                # endByte = uart1.read(1)[0]
                # print("CRC:", crc)
                # print("Buffer: ", buffer)
                command = chr(buffer[0])
                # command = 'P'
                # print("Command:", command)
                if command == 'C':
                    print("Calibrating")
                    calibrate()
                elif command == 'R':
                    print("Resetting")
                    reset_tracking()
                elif command == 'P':
                    print("Sending Position")
                    pos = get_position()
                    # print("sending position", pos)
                    data = struct.pack("<xfff", *pos)
                    # print("data", " ".join([hex(i) for i in data]))
                    crc = sum(struct.pack("fff", *pos)) & 0xFFFF
                    data += struct.pack("<Hxb", crc, 1)
                    dir.value(1)
                    uart1.write(data)
                    uart1.flush()
                elif command == 'S':
                    print("Setting Pose")
                    set_position(struct.unpack("<3f", bytes(buffer[1:])))
                elif command == 'O':
                    print("Setting Offset")
                    set_offset(struct.unpack("<3f", bytes(buffer[1:])))
                elif command == 'L':
                    print("Setting Linear Scaler")
                    set_linear_scaler(struct.unpack("<f", bytes(buffer[1:]))[0])
                elif command == 'A':
                    print("Setting Angular Scaler")
                    set_angular_scaler(struct.unpack("<f", bytes(buffer[1:]))[0])

            # time.sleep(0.01)
        except Exception as e:
            print(e)