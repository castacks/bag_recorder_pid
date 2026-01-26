import Jetson.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)

LED_PIN = 36   # choose a known GPIO output pin (physical pin)

GPIO.setup(LED_PIN, GPIO.OUT, initial=GPIO.LOW)

print("Toggling LED every 5 seconds (CTRL+C to exit)")

try:
    state = False
    while True:
        state = not state
        GPIO.output(LED_PIN, state)

        if state:
            print("LED is ON")
        else:
            print("LED is OFF")

        time.sleep(5)

except KeyboardInterrupt:
    print("\nExiting...")

finally:
    GPIO.cleanup()
