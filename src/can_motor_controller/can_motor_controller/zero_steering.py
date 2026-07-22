#!/usr/bin/env python3
"""
Permanently resets the GIM steering motor origin to 0.00 rad (Command 0xB1).
"""

import math
import sys
import os

# Bootstrap package identity
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
__package__ = 'can_motor_controller'

from .can_interface import CANInterface
from .motor import Motor

def main():
    CAN_CHANNEL = 'can0'
    STEER_ID = 2  # Your steering motor ID

    print(f"Opening '{CAN_CHANNEL}' to zero Steer Motor (ID: 0x{STEER_ID:02X})...")
    with CANInterface(channel=CAN_CHANNEL, baudrate=1000000) as can_if:
        steer_motor = Motor(can_if, dev_addr=STEER_ID)
        
        # 1. Read current angle
        before = steer_motor.read_angle()
        print(f"Current Multi-Turn Angle : {math.radians(before.multi_turn_angle):.4f} rad ({before.multi_turn_angle:.1f}°)")
        
        # 2. Instruct user
        input("\n👉 PHYSICALLY ALIGN your front steering wheel to DEAD CENTER by hand, then press [ENTER]...")
        
        # 3. Burn new origin (Command 0xB1)
        print("Sending SET_ORIGIN (0xB1) command to driver ROM...")
        steer_motor.set_origin()
        
        # 4. Verify
        after = steer_motor.read_angle()
        print(f"✅ NEW Multi-Turn Angle    : {math.radians(after.multi_turn_angle):.4f} rad ({after.multi_turn_angle:.4f}°)")
        print("The motor memory has been reset! You can now run ROS 2 tests safely.")

if __name__ == '__main__':
    main()