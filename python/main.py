import serial
import uinput
import time
import struct

# Adjust the serial port and baud rate to match your HC-05 module
ser = serial.Serial('/dev/rfcomm0', 9600)  # Use '/dev/rfcomm0' for Bluetooth serial
# ser = serial.Serial('/dev/ttyUSB0', 9600)  # For USB serial connections

# Create a uinput device for mouse, keyboard, and buttons
device = uinput.Device([
    uinput.BTN_LEFT,
    uinput.BTN_RIGHT,
    uinput.REL_X,
    uinput.REL_Y,
    uinput.KEY_W,
    uinput.KEY_A,
    uinput.KEY_S,
    uinput.KEY_D,
    uinput.KEY_SPACE,
    uinput.KEY_Q,
])

def parse_packet(packet):
    # Unpack the packet data
    # '>bbbbB' means:
    #   >     : Big-endian
    #   bbbb  : Four signed chars (int8_t)
    #   B     : One unsigned char (uint8_t)
    x1_axis, y1_axis, x2_axis, y2_axis, buttons = struct.unpack('>bbbbB', packet)
    return x1_axis, y1_axis, x2_axis, y2_axis, buttons

def handle_joystick1(x, y):
    # Map joystick 1 to WASD keys
    # Threshold to determine if joystick is moved significantly
    threshold = 20  # Adjust as necessary

    # Movement flags
    forward = False
    backward = False
    left = False
    right = False

    if y < -threshold:
        # Joystick pushed forward
        forward = True
    elif y > threshold:
        # Joystick pulled backward
        backward = True

    if x < -threshold:
        # Joystick moved left
        left = True
    elif x > threshold:
        # Joystick moved right
        right = True

    # Emit key events
    # Forward (W)
    device.emit(uinput.KEY_W, 1 if forward else 0)
    # Backward (S)
    device.emit(uinput.KEY_S, 1 if backward else 0)
    # Left (A)
    device.emit(uinput.KEY_A, 1 if left else 0)
    # Right (D)
    device.emit(uinput.KEY_D, 1 if right else 0)

def handle_joystick2(x, y):
    # Use joystick 2 to move the mouse
    # Adjust sensitivity as necessary
    sensitivity = 1
    device.emit(uinput.REL_X, x * sensitivity, syn=False)
    device.emit(uinput.REL_Y, y * sensitivity)

def handle_buttons(buttons):
    # Map buttons as per the specification:
    # Button 1 (bit 0): Switch that turns on LED (handled in main.c)
    # Button 2 (bit 1): Shooting (left mouse button)
    # Button 3 (bit 2): Aiming (right mouse button)
    # Button 4 (bit 3): Jumping (spacebar)
    # Button 5 (bit 4): Grenade ('Q' key)

    # Shooting (left mouse button)
    if buttons == 2:
        print("Shooting")
        device.emit(uinput.BTN_LEFT, 1)
    else:
        device.emit(uinput.BTN_LEFT, 0)

    # Aiming (right mouse button)
    if buttons == 3:
        print("Aiming")
        device.emit(uinput.BTN_RIGHT, 1)
    else:
        device.emit(uinput.BTN_RIGHT, 0)

    # Jumping (spacebar)
    if buttons == 4:
        print("Jumping")
        device.emit(uinput.KEY_SPACE, 1)
    else:
        device.emit(uinput.KEY_SPACE, 0)

    # Grenade ('Q' key)
    if buttons == 1:
        print("Grenade")
        device.emit(uinput.KEY_Q, 1)
    else:
        device.emit(uinput.KEY_Q, 0)

def main():
    try:
        while True:
            # Wait for the header byte (0xAA)
            header = ser.read(1)
            if header == b'\xAA':
                # Read the next 5 bytes (data)
                data_bytes = ser.read(5)
                # Read the end-of-packet byte (0xFF)
                eop = ser.read(1)
                if eop == b'\xFF':
                    # Parse the packet
                    x1_axis, y1_axis, x2_axis, y2_axis, buttons = parse_packet(data_bytes)

                    # Debug print
                    print(f"X1: {x1_axis}, Y1: {y1_axis}, X2: {x2_axis}, Y2: {y2_axis}, Buttons: {buttons}")

                    # Handle joystick 1 (movement)
                    handle_joystick1(x1_axis, y1_axis)

                    # Handle joystick 2 (camera/mouse)
                    handle_joystick2(x2_axis, y2_axis)

                    # Handle buttons
                    handle_buttons(buttons)

                else:
                    # Invalid end-of-packet byte, discard and continue
                    continue
            else:
                # Discard bytes until we find the header byte
                continue

    except KeyboardInterrupt:
        print("Program terminated by user")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
