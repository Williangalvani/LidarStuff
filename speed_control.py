import RPi.GPIO as GPIO

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(7, GPIO.OUT)  # Enable pin
GPIO.setup(8, GPIO.OUT)  # Forward pin
GPIO.setup(25, GPIO.OUT) # Backward pin

# Initialize PWM on enable pin with 50Hz frequency
pwm = GPIO.PWM(7, 50)

# Start PWM with 0% duty cycle
pwm.start(0)

try:
    while True:
        try:
            # Take input from user
            speed = int(input("Enter a value between -10 and 10 (or type 'exit' to quit): "))
            
            if speed < -10 or speed > 10:
                print("Invalid input. Please enter a value between -10 and 10.")
                continue
            
            # Calculate duty cycle
            duty_cycle = abs(speed) * 10
            
            # Set direction and speed
            if speed > 0:
                GPIO.output(8, True)  # Forward
                GPIO.output(25, False) # Backward off
            elif speed < 0:
                GPIO.output(8, False) # Forward off
                GPIO.output(25, True)  # Backward
            else:
                GPIO.output(8, False) # Forward off
                GPIO.output(25, False) # Backward off
            
            pwm.ChangeDutyCycle(duty_cycle)
        
        except ValueError:
            command = input("Invalid input. Type 'exit' to quit or press Enter to continue: ").strip().lower()
            if command == 'exit':
                break

except KeyboardInterrupt:
    pass

finally:
    # Stop the PWM
    pwm.stop()

    # Reset GPIO settings
    GPIO.cleanup()

    print("Exiting program and cleaning up GPIO.")
