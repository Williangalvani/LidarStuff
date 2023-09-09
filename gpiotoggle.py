import RPi.GPIO as GPIO

def toggle_gpio(pin):
    current_state = GPIO.input(pin)
    new_state = not current_state
    GPIO.output(pin, new_state)
    print(f"Toggled GPIO {pin} to {'HIGH' if new_state else 'LOW'}")

def main():
    GPIO.setmode(GPIO.BCM)
    
    # Dictionary to keep track of pin states
    pin_states = {}
    
    while True:
        try:
            pin = int(input("Enter the GPIO pin number to toggle (or type 'exit' to quit): "))
            
            if pin not in pin_states:
                GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
                pin_states[pin] = GPIO.LOW
            
            toggle_gpio(pin)
            pin_states[pin] = GPIO.input(pin)
        
        except ValueError:
            command = input("Invalid input. Type 'exit' to quit or press Enter to continue: ").strip().lower()
            if command == 'exit':
                break

    GPIO.cleanup()
    print("Exiting program and cleaning up GPIO.")

if __name__ == "__main__":
    main()
