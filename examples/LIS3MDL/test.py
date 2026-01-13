#!/usr/bin/env python3
"""
LIS3MDL magnetometer reader
- Press SPACE to stop
- Ctrl+C also works
"""

import time
import sys
import termios
import tty
import select
import numpy as np
from pylisxmdl import pylisxmdl, FullScale, ODR

def key_pressed():
    """Non-blocking check if a key was pressed"""
    i, o, e = select.select([sys.stdin], [], [], 0.0)
    return i


def get_key():
    """Read one key without waiting (non-blocking, no echo)"""
    if key_pressed():
        return sys.stdin.read(1)
    return None


def main():
    print("LIS3MDL Magnetometer - Press SPACE to stop")
    print("-----------------------------------------\n")

    # Make stdin non-blocking and disable echo
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())

        # Initialize sensor - feel free to change parameters
        try:
            mag = pylisxmdl(FullScale.Gauss_4, ODR.Hz_80)
            print("Sensor initialized successfully")
        except Exception as e:
            print("Failed to initialize sensor:", e)
            print("(make sure pigpiod is running → sudo pigpiod)")
            return

        print("      X [G]        Y [G]        Z [G]     ")
        print("-------------------------------------------------------------------")

        running = True
        data = []
        while running:
            # Read data
            try:

                if(mag.data_ready()):
                    x, y, z = mag.read_gauss()
                    print(f"{x:12.5f}  {y:12.5f}  {z:12.5f} ",
                      end='\r', flush=True)
                    data.append([x,y,z])

            except RuntimeError as e:
                print(f"\nRead error: {e}")
                time.sleep(1)

            # Check for SPACE key (or Ctrl+C handled by except KeyboardInterrupt)
            key = get_key()
            if key == ' ':
                print("\n\nSPACE pressed → stopping...")
                running = False
            elif key == '\x03':  # Ctrl+C
                raise KeyboardInterrupt

            time.sleep(0.08)  # ~12–13 updates/sec

    except KeyboardInterrupt:
        print("\n\nStopped by Ctrl+C")
    except Exception as e:
        print("\nError:", e)
    finally:
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print("Terminal restored")
    return np.array(data)


if __name__ == "__main__":
    main()