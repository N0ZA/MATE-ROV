import pygame
import time

# --- Setup Joystick ---
pygame.init()
pygame.joystick.init()

# Check if the Logitech is actually plugged in and recognized
if pygame.joystick.get_count() == 0:
    print("No joystick detected! Plug in the Logitech Extreme 3D Pro and restart.")
    exit()

# Connect to the first recognized joystick
logitech = pygame.joystick.Joystick(0)
logitech.init()

print(f"Connected to: {logitech.get_name()}")
print("Reading joystick data... Move the stick! (Press Ctrl+C to stop)\n")

try:
    while True:
        # Pygame requires this event pump to update the USB hardware state internally
        pygame.event.pump() 

        # Read the raw analog axes (Values range from -1.0 to 1.0)
        # Axis mappings for Logitech Extreme 3D Pro:
        # 0 = X Axis (Left/Right)
        # 1 = Y Axis (Forward/Backward)
        # 2 = Twist (Z Axis)
        # 3 = Little Throttle flap at the base
        
        x_axis = logitech.get_axis(0)
        y_axis = logitech.get_axis(1)
        twist = logitech.get_axis(2)
        throttle = logitech.get_axis(3)

        # Hat switch (8-way D-pad) returns a tuple (x, y), each value -1, 0, or 1
        hat = logitech.get_hat(0)
        hat_directions = {
            ( 0,  0): "CENTER",
            ( 0,  1): "UP",
            ( 0, -1): "DOWN",
            (-1,  0): "LEFT",
            ( 1,  0): "RIGHT",
            (-1,  1): "UP-LEFT",
            ( 1,  1): "UP-RIGHT",
            (-1, -1): "DOWN-LEFT",
            ( 1, -1): "DOWN-RIGHT",
        }
        hat_label = hat_directions.get(hat, f"UNKNOWN{hat}")

        # Print the formatted values to the console
        print(f"X: {x_axis:>6.2f} | Y: {y_axis:>6.2f} | Twist: {twist:>6.2f} | Throttle: {throttle:>6.2f} | Hat: {hat} {hat_label:<12}")

        # Pause for 1/10th of a second so the terminal doesn't become a blurry mess
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nTest cleanly stopped by user.")
finally:
    # Always clean up hardware resources when done
    pygame.quit()