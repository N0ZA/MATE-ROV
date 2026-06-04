#!/usr/bin/env python3
"""
Quick DualSense controller test — reads /dev/input/js1 and prints axes/buttons.
Move sticks and press buttons to verify inputs. Ctrl+C to quit.
"""
import struct, sys, os

DEVICE = '/dev/input/js1'
JS_EVENT_FMT = 'IhBB'  # time, value, type, number
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FMT)
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS   = 0x02
JS_EVENT_INIT   = 0x80

NUM_AXES    = 8   # 0-5 sticks/triggers, 6-7 D-Pad hat
NUM_BUTTONS = 13  # PS button not accessible via js interface

axes    = [0] * NUM_AXES
buttons = [0] * NUM_BUTTONS

AXIS_NAMES = {
    0: 'Left X  (sway/yaw?)',
    1: 'Left Y  (surge?)',
    2: 'Right X (sway/yaw?)',
    3: 'L2 trigger',
    4: 'R2 trigger',
    5: 'Right Y (vert?)',
    6: 'D-Pad X',
    7: 'D-Pad Y',
}
BTN_NAMES = {
    0: 'Square', 1: 'Cross', 2: 'Circle', 3: 'Triangle',
    4: 'L1', 5: 'R1', 6: 'L2', 7: 'R2',
    8: 'Share', 9: 'Options', 10: 'L3', 11: 'R3',
    12: 'PS (may not register via js)',
}

def render():
    os.system('clear')
    print(f"DualSense on {DEVICE}  |  Ctrl+C to quit\n")
    print("=== AXES (range -32767 to 32767) ===")
    for i, v in enumerate(axes):
        bar_len = int(abs(v) / 32767 * 20)
        bar = ('#' * bar_len).ljust(20)
        direction = '>' if v >= 0 else '<'
        name = AXIS_NAMES.get(i, f'Axis {i}')
        print(f"  {i}: [{direction}{bar}] {v:+6d}  {name}")
    print("\n=== BUTTONS ===")
    row = ''
    for i, v in enumerate(buttons):
        name = BTN_NAMES.get(i, f'Btn{i}')
        state = '[X]' if v else '[ ]'
        row += f"  {state} {name:<10}"
        if (i + 1) % 4 == 0:
            print(row); row = ''
    if row:
        print(row)

if not os.path.exists(DEVICE):
    print(f"Error: {DEVICE} not found. Is the DualSense connected?")
    sys.exit(1)

print(f"Opening {DEVICE}...")
try:
    with open(DEVICE, 'rb') as f:
        render()
        while True:
            data = f.read(JS_EVENT_SIZE)
            if not data:
                break
            _, value, typ, number = struct.unpack(JS_EVENT_FMT, data)
            typ &= ~JS_EVENT_INIT
            if typ == JS_EVENT_AXIS and number < NUM_AXES:
                axes[number] = value
            elif typ == JS_EVENT_BUTTON and number < NUM_BUTTONS:
                buttons[number] = value
            render()
except PermissionError:
    print(f"Permission denied on {DEVICE}. Try: sudo python3 test_controller.py")
except KeyboardInterrupt:
    print("\nDone.")
