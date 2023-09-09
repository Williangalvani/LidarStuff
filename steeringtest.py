import RPi.GPIO as GPIO
import time

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(19, GPIO.OUT)

# Initialize PWM on pin 19 with 50Hz frequency
pwm = GPIO.PWM(19, 50)

# Start PWM with 0% duty cycle
pwm.start(0)

try:
    while True:
        # Sweep from 0 to 180 degrees
        for angle in range(0, 181):
            duty_cycle = (angle / 18) + 2
            pwm.ChangeDutyCycle(duty_cycle)
            time.sleep(0.02)

        # Sweep from 180 to 0 degrees
        for angle in range(180, -1, -1):
            duty_cycle = (angle / 18) + 2
            pwm.ChangeDutyCycle(duty_cycle)
            time.sleep(0.02)

except KeyboardInterrupt:
    # Stop the PWM
    pwm.stop()

    # Reset GPIO settings
    GPIO.cleanup()
