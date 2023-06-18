import numpy as np
import cv2
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
import atexit
import time
import Adafruit_SSD1306  # This is the driver chip for the Adafruit PiOLED
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont
import subprocess

def get_network_interface_state(interface):
    return subprocess.check_output(
        "cat /sys/class/net/%s/operstate" % interface, shell=True
    ).decode("ascii")[:-1]

def get_ip_address(interface):
    if get_network_interface_state(interface) == "down":
        return None
    cmd = (
        "ifconfig %s | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1'"
        % interface
    )
    return subprocess.check_output(cmd, shell=True).decode("ascii")[:-1]

# Return a string representing the percentage of CPU in use

def get_cpu_usage():
    # Shell scripts for system monitoring from here : https://unix.stackexchange.com/questions/119126/command-to-display-memory-usage-disk-usage-and-cpu-load
    cmd = "top -bn1 | grep load | awk '{printf \"CPU Load: %.2f\", $(NF-2)}'"
    CPU = subprocess.check_output(cmd, shell=True)
    return CPU

def get_gpu_usage():
    GPU = 0.0
    with open("/sys/devices/gpu.0/load", encoding="utf-8") as gpu_file:
        GPU = gpu_file.readline()
        GPU = int(GPU) / 10
    return GPU

# 128x32 display with hardware I2C:
# setting gpio to 1 is hack to avoid platform detection
disp = Adafruit_SSD1306.SSD1306_128_32(rst=None, i2c_bus=0, gpio=1, i2c_address=0x3C)

# Initialize library.
disp.begin()

# Clear display.
disp.clear()
disp.display()

# Create blank image for drawing.
# Make sure to create image with mode '1' for 1-bit color.
width = disp.width
height = disp.height
image = Image.new("1", (width, height))

# Get drawing object to draw on image.
draw = ImageDraw.Draw(image)

# Draw a black filled box to clear the image.
draw.rectangle((0, 0, width, height), outline=0, fill=0)

# Draw some shapes.
# First define some constants to allow easy resizing of shapes.
padding = -2
top = padding
bottom = height - padding
# Move left to right keeping track of the current x position for drawing shapes.
x = 0

# Load default font.
font = ImageFont.load_default()

while True:
    # Draw a black filled box to clear the image.
    draw.rectangle((0, 0, width, height), outline=0, fill=0)

    # Shell scripts for system monitoring from here : https://unix.stackexchange.com/questions/119126/command-to-display-memory-usage-disk-usage-and-cpu-load
    cmd = "free -m | awk 'NR==2{printf \"Mem:  %.0f%% %s/%s M\", $3*100/$2, $3,$2 }'"
    MemUsage = subprocess.check_output(cmd, shell=True)
    cmd = 'df -h | awk \'$NF=="/"{printf "Disk: %d/%dGB %s", $3,$2,$5}\''
    Disk = subprocess.check_output(cmd, shell=True)

    # Print the IP address
    # Two examples here, wired and wireless
    draw.text((x, top), "eth0: " + str(get_ip_address("eth0")), font=font, fill=255)
    # draw.text((x, top+8),     "wlan0: " + str(get_ip_address('wlan0')), font=font, fill=255)

    # Alternate solution: Draw the GPU usage as text
    # draw.text((x, top+8),     "GPU:  " +"{:3.1f}".format(GPU)+" %", font=font, fill=255)
    # We draw the GPU usage as a bar graph
    string_width, string_height = font.getsize("GPU:  ")
    # Figure out the width of the bar
    full_bar_width = width - (x + string_width) - 1
    gpu_usage = get_gpu_usage()
    # Avoid divide by zero ...
    if gpu_usage == 0.0:
        gpu_usage = 0.001
    draw_bar_width = int(full_bar_width * (gpu_usage / 100))
    draw.text((x, top + 8), "GPU:  ", font=font, fill=255)
    draw.rectangle(
        (x + string_width, top + 12, x + string_width + draw_bar_width, top + 14),
        outline=1,
        fill=1,
    )

    # Show the memory Usage
    draw.text((x, top + 16), str(MemUsage.decode("utf-8")), font=font, fill=255)
    # Show the amount of disk being used
    draw.text((x, top + 25), str(Disk.decode("utf-8")), font=font, fill=255)

    # Display image.
    # Set the SSD1306 image to the PIL image we have made, then dispaly
    disp.image(image)
    disp.display()
    # 1.0 = 1 second; The divisor is the desired updates (frames) per second
    time.sleep(1.0 / 4)
    break

mh = Adafruit_MotorHAT(i2c_bus=1, addr=0x60)

def gstreamer_pipeline(
    capture_width=1280,
    capture_height=720,
    display_width=1280,
    display_height=720,
    framerate=60,
    flip_method=0,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

cam_pipeline = gstreamer_pipeline(flip_method=2)
video_capture = cv2.VideoCapture(cam_pipeline, cv2.CAP_GSTREAMER)

# recommended for auto-disabling motors on shutdown!
def turnOffMotors():
    mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)


atexit.register(turnOffMotors)

################################# DC motor test!
myMotor = mh.getMotor(1)
myMotor2 = mh.getMotor(2)

# set the speed to start, from 0 (off) to 255 (max speed)
myMotor.setSpeed(500)
myMotor2.setSpeed(500)
myMotor.run(Adafruit_MotorHAT.FORWARD)
# turn on motor
myMotor.run(Adafruit_MotorHAT.RELEASE)
myMotor2.run(Adafruit_MotorHAT.RELEASE)

while(1):
    ret, frame = video_capture.read()
    #cv2.imshow('frame', frame)
    crop_img = frame[650:720, 0:320]
    gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    ret, thresh = cv2.threshold(blur, 60, 255, cv2.THRESH_BINARY_INV)
    contours, hierachy = cv2.findContours(thresh.copy(), 1, cv2.CHAIN_APPROX_NONE)

    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)
        if M['m00'] == 0:
            M['m00'] = 1

        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])

        cv2.line(crop_img, (cx, 0), (cx, 720), (255, 0, 0), 1)
        cv2.line(crop_img, (0, cy), (1280, cy), (255, 0, 0), 1)

        cv2.drawContours(crop_img, contours, -1, (0, 255, 0), 1)

        if cx >= 280:
            myMotor.run(Adafruit_MotorHAT.FORWARD)
            myMotor2.run(Adafruit_MotorHAT.RELEASE)
        if cx < 280 and cx > 50:
            myMotor.run(Adafruit_MotorHAT.FORWARD)
            myMotor2.run(Adafruit_MotorHAT.FORWARD)
        if cx <= 50:
            myMotor.run(Adafruit_MotorHAT.RELEASE)
            myMotor2.run(Adafruit_MotorHAT.FORWARD)

    #cv2.imshow('frame', crop_img)
    #cv2.imshow('frame', crop_img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break