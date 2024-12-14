import time
from sense_hat import SenseHat
import RPi.GPIO as GPIO

# GPIO configuration for the L298N motor driver
MOTOR_PIN1 = 23  # IN1 on L298N
MOTOR_PIN2 = 24  # IN2 on L298N
ENABLE_PIN = 25  # ENA on L298N

# Pressure thresholds in hPa (Sense HAT measures in hPa, 1 hPa = 0.0145038 psi)
PRESSURE_MIN = 700  # Equivalent to ~10 psi
PRESSURE_MAX = 930  # Equivalent to ~13.5 psi

# Initialize Sense HAT and GPIO
sense = SenseHat()
GPIO.setmode(GPIO.BCM)
GPIO.setup(MOTOR_PIN1, GPIO.OUT)
GPIO.setup(MOTOR_PIN2, GPIO.OUT)
GPIO.setup(ENABLE_PIN, GPIO.OUT)

# PWM setup for motor speed control
motor_pwm = GPIO.PWM(ENABLE_PIN, 100)  # 100 Hz PWM frequency
motor_pwm.start(0)  # Start with 0% duty cycle (motor off)

# Function to simulate motor activation
def activate_motor(duration=5):
    """
    Activates the DC gear motor for a specified duration.
    Args:
        duration (int): Time in seconds to keep the motor running.
    """
    print("Turning on the motor...")
    GPIO.output(MOTOR_PIN1, GPIO.HIGH)
    GPIO.output(MOTOR_PIN2, GPIO.LOW)
    motor_pwm.ChangeDutyCycle(100)  # Set motor speed to 100%
    time.sleep(duration)
    print("Turning off the motor.")
    motor_pwm.ChangeDutyCycle(0)  # Stop the motor
    GPIO.output(MOTOR_PIN1, GPIO.LOW)
    GPIO.output(MOTOR_PIN2, GPIO.LOW)

# Main program loop
def main():
    try:
        while True:
            # Read pressure from the Sense HAT
            pressure = sense.get_pressure()  # Pressure in hPa
            print(f"Current pressure: {pressure:.2f} hPa")

            # Check if the pressure is below the minimum threshold
            if pressure < PRESSURE_MIN:
                print("Pressure too low! Activating motor to simulate air refill.")
                activate_motor(duration=5)  # Run the motor for 5 seconds

            # Wait 10 seconds before the next reading
            time.sleep(10)
    except KeyboardInterrupt:
        print("Simulation stopped by user.")
    finally:
        # Cleanup GPIO settings
        GPIO.cleanup()
        motor_pwm.stop()

# Run the main function
if _name_ == "_main_":
    main()