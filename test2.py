from socket import timeout
import serial
import time
import RPi.GPIO as GPIO
import curses
import math

#Serial port variables
SERIAL_PORT = "/dev/ttyAMA0"
SERIAL_BAUDRATE = 115200

GPIO.setmode(GPIO.BCM)
GPIO.setup(18,GPIO.OUT)
pi_pwm = GPIO.PWM(18,1000)		#create PWM instance with frequency
pi_pwm.start(100)
time.sleep(1)	


def draw_circle(window, points, aspect_ratio=2.0):
    window.clear()
    centerx = 40
    centery = 12
    for angle, radius, quality in points:
        x = int(centerx + aspect_ratio * radius * 20 * math.cos(math.radians(angle)))
        y = int(centery + radius * 20 * math.sin(math.radians(angle)))
        try:
            window.addch(y, x, "*")
        except curses.error:
            pass  # ignore errors for simplicity
    window.addch(centery, centerx, "O")
    window.refresh()



class PI:
    
    def __init__(self, kp, ki):
        self.kp = kp
        self.ki = ki
        self.integral = 0

    def update(self, error):
        self.integral += error
        return self.kp * error + self.ki * self.integral

pi = PI(0.6, 0.01)


def update_rpm_controller(rpm):
    target_speed = 420
    error = target_speed - rpm
    #print("error: %f" % error)
    controller = pi.update(error)
    #print("update: %f" % controller)
    pi_pwm.ChangeDutyCycle(max(min(controller, 100), 0))



#Scan variables
scanSamplesSignalQuality= [0.0]
scanSamplesRange = [0.0]
composedSamples = []

#Delta-2G Frame Characteristics
#Constant Frame Parts values
FRAME_HEADER = 0xAA            #Frame Header value
PROTOCOL_VERSION = 0x01        #Protocol Version value
FRAME_TYPE = 0x61            #Frame Type value
#Scan Characteristics
SCAN_STEPS = 15                #How many steps/frames each full 360deg scan is composed of
#Received value scaling
ROTATION_SPEED_SCALE = 0.05 * 60    #Convert received value to RPM (LSB: 0.05 rps)
ANGLE_SCALE = 0.01            #Convert received value to degrees (LSB: 0.01 degrees)
RANGE_SCALE = 0.25 * 0.001    #Convert received value to meters (LSB: 0.25 mm)

#Delta-2G frame structure
class Delta2GFrame:
    frameHeader = 0            #Frame Header: 1 byte
    frameLength = 0            #Frame Length: 2 bytes, from frame header to checksum (excluded)
    protocolVersion = 0        #Protocol Version: 1 byte
    frameType = 0            #Frame Type: 1 byte
    commandWord = 0            #Command Word: 1 byte, identifier to distinguish parameters
    parameterLength = 0        #Parameter Length: 2 bytes, length of the parameter field
    parameters = [0]        #Parameter Field
    checksum = 0            #Checksum: 2 bytes

def LiDARFrameProcessing(frame: Delta2GFrame, stdscr):
    global composedSamples
    if frame.commandWord == 0xAE:
            #Device Health Information: Speed Failure
            rpm = frame.parameters[0] * ROTATION_SPEED_SCALE
            #print("RPM: %f" % rpm)
            stdscr.addstr(0, 0, f"RPM: {rpm:.2f}")
            update_rpm_controller(rpm)
    elif frame.commandWord == 0xAD:
            #1st: Rotation speed (1 byte)
            rpm = frame.parameters[0] * ROTATION_SPEED_SCALE
            #print("RPM: %f" % rpm)
            stdscr.addstr(0, 0, f"RPM: {rpm:.2f}")
            update_rpm_controller(rpm)
            #2nd: Zero Offset angle (2 bytes)
            offsetAngle = (frame.parameters[1] << 8) + frame.parameters[2]
            offsetAngle = offsetAngle * ANGLE_SCALE

            #3rd: Start angle of current data freame (2 bytes)
            startAngle = (frame.parameters[3] << 8) + frame.parameters[4]
            startAngle = startAngle * ANGLE_SCALE

            #Calculate number of samples in current frame
            sampleCnt = int((frame.parameterLength - 5) / 3)

            #Calculate current angle index of a full frame: For Delta-2G each full rotation has 15 frames
            frameIndex = int(startAngle / (360.0 / SCAN_STEPS))

            if frameIndex == 0:
                #New scan started
                scanSamplesRange.clear()
                scanSamplesSignalQuality.clear()
                composedSamples.clear()

            #4th: LiDAR samples, each sample has: Signal Value/Quality (1 byte), Distance Value (2 bytes)
            for i in range(sampleCnt):
                signalQuality = frame.parameters[5 + (i * 3)]
                distance = (frame.parameters[5 + (i * 3) + 1] << 8) + frame.parameters[5 + (i * 3) + 2]
                scanSamplesSignalQuality.append(signalQuality)
                scanSamplesRange.append(distance * RANGE_SCALE)
                angle = startAngle + i * (360 / SCAN_STEPS / sampleCnt)
                composedSamples.append((angle, distance * RANGE_SCALE, signalQuality))
                #composedSamples.append((distance * RANGE_SCALE * 50, startAngle + i * (360 / SCAN_STEPS)))
            if frameIndex == (SCAN_STEPS - 1):
                draw_circle(stdscr, composedSamples)
                composedSamples = []



def main(stdscr):
    curses.curs_set(0)
    stdscr.nodelay(1)  # don't block for user input

    try:
        lidarSerial = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=0)
    except serial.serialutil.SerialException:
        print("ERROR: Serial Connect Error")
        return

    status = 0
    checksum = 0
    lidarFrame = Delta2GFrame()
    while True:
        time.sleep(0.003)
        c = stdscr.getch()
        if c == ord('q'):
            pi_pwm.ChangeDutyCycle(0)
            break  # Quit the loop if 'q' is pressed

          # Did the user close the window?
        # for event in pygame.event.get():
        #     if event.type == pygame.QUIT:
        #         exit()
        rx = lidarSerial.read(100)

        for by in rx:
                if status == 0:
                    #1st frame byte: Frame Header
                    lidarFrame.frameHeader = by
                    if lidarFrame.frameHeader == FRAME_HEADER:
                        #Valid Header
                        status = 1
                    else:
                        print("ERROR: Frame Header Failed")
                    #Reset checksum, new frame start
                    checksum = 0
                elif status == 1:
                    #2nd frame byte: Frame Length MSB
                    lidarFrame.frameLength = (by << 8)
                    status = 2
                elif status == 2:
                    #3rd frame byte: Frame Length LSB
                    lidarFrame.frameLength += by
                    status = 3
                elif status == 3:
                    #4th frame byte: Protocol Version
                    lidarFrame.protocolVersion = by
                    if lidarFrame.protocolVersion == PROTOCOL_VERSION:
                        #Valid Protocol Version
                        status = 4
                    else:
                        print("ERROR: Frame Protocol Version Failed")
                        status = 0
                elif status == 4:
                    #5th frame byte: Frame Type
                    lidarFrame.frameType = by
                    if lidarFrame.frameType == FRAME_TYPE:
                        #Valid Frame Type
                        status = 5
                    else:
                        print("ERROR: Frame Type Failed")
                        status = 0
                elif status == 5:
                    #6th frame byte: Command Word
                    lidarFrame.commandWord = by
                    status = 6
                elif status == 6:
                    #7th frame byte: Parameter Length MSB
                    lidarFrame.parameterLength = (by << 8)
                    status = 7
                elif status == 7:
                    #8th frame byte: Parameter Length LSB
                    lidarFrame.parameterLength += by
                    lidarFrame.parameters.clear()
                    status = 8
                elif status == 8:
                    #9th+ frame bytes: Parameters
                    lidarFrame.parameters.append(by)
                    if len(lidarFrame.parameters) == lidarFrame.parameterLength:
                        #End of parameter frame bytes
                        status = 9
                elif status == 9:
                    #N+1 frame byte: Checksum MSB
                    lidarFrame.checksum = (by << 8)
                    status = 10
                elif status == 10:
                    #N+2 frame byte: Checksum LSB
                    lidarFrame.checksum += by
                    #End of frame reached
                    #Compare received and calculated frame checksum
                    if lidarFrame.checksum == checksum:
                        #Checksum match: Valid frame
                        LiDARFrameProcessing(lidarFrame, stdscr)
                    else:
                        #Checksum missmatach: Invalid frame
                        print("ERROR: Frame Checksum Failed");
                    status = 0
                #Calculate current frame checksum, all bytes excluding the last 2, which are the checksum
                if status < 10:
                    checksum = (checksum + by) % 0xFFFF

if __name__ == "__main__":
    curses.wrapper(main)
