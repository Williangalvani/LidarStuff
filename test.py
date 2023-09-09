import RPi.GPIO as GPIO
import time

# Set up the GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(18, GPIO.OUT)

while True:
    # Turn the GPIO pin on
    GPIO.output(18, GPIO.HIGH)
    print("GPIO 18 is HIGH")
    
    # Wait for 5 seconds
    time.sleep(5)
    
    # Turn the GPIO pin off
    GPIO.output(18, GPIO.LOW)
    print("GPIO 18 is LOW")
    
    # Wait for 5 seconds
    time.sleep(5)
