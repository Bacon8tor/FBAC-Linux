#!/usr/bin/env python3
"""
Arduino Diagnostic Tool
Helps diagnose Arduino communication issues like 0xFF flooding
"""

import serial
import time
import sys

def diagnose_arduino(port: str):
    """Diagnose Arduino communication issues"""
    print(f"=== Arduino Diagnostic Tool ===")
    print(f"Port: {port}")

    try:
        # Open connection
        print("Opening connection...")
        ser = serial.Serial(
            port=port,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            bytesize=8,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,
            write_timeout=1.0
        )

        print("Connection opened successfully")
        time.sleep(2)  # Give Arduino time to reset

        # Clear buffers
        print("Clearing buffers...")
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        # Test 1: Check what Arduino is sending
        print("\n=== Test 1: Monitor incoming data ===")
        start_time = time.time()
        byte_counts = {}
        total_bytes = 0

        while time.time() - start_time < 5.0:  # Monitor for 5 seconds
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                for byte_val in data:
                    byte_counts[byte_val] = byte_counts.get(byte_val, 0) + 1
                    total_bytes += 1

                    # Print flood detection
                    if total_bytes % 100 == 0:
                        print(f"Received {total_bytes} bytes...")

            time.sleep(0.01)

        print(f"\nReceived {total_bytes} bytes in 5 seconds")
        if byte_counts:
            print("Byte frequency:")
            for byte_val, count in sorted(byte_counts.items()):
                percentage = (count / total_bytes) * 100
                print(f"  0x{byte_val:02X}: {count} times ({percentage:.1f}%)")

            # Check for flood pattern
            most_common = max(byte_counts, key=byte_counts.get)
            if byte_counts[most_common] > total_bytes * 0.8:
                print(f"WARNING: FLOOD DETECTED: 0x{most_common:02X} represents {byte_counts[most_common]/total_bytes*100:.1f}% of traffic")
        else:
            print("No data received - Arduino may not be sending")

        # Test 2: Try to send handshake
        print("\n=== Test 2: Handshake test ===")
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        print("Sending handshake 0xA5...")
        ser.write(bytes([0xA5]))
        ser.flush()

        # Wait for response
        response_data = []
        start_time = time.time()
        while time.time() - start_time < 2.0:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                response_data.extend(data)
                if len(response_data) >= 10:  # Limit response collection
                    break
            time.sleep(0.01)

        if response_data:
            print(f"Handshake response: {[f'0x{b:02X}' for b in response_data[:10]]}")
            if 0x5A in response_data:
                print("[OK] Handshake successful - Arduino responded with 0x5A")
            else:
                print("[ERROR] Unexpected handshake response")
        else:
            print("[ERROR] No handshake response")

        # Test 3: Try to send a supported command
        print("\n=== Test 3: Command test ===")
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        print("Sending lightBits command 0x01...")
        ser.write(bytes([0x01]))
        ser.flush()

        # Wait for response
        response_data = []
        start_time = time.time()
        while time.time() - start_time < 1.0:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                response_data.extend(data)
                if len(response_data) >= 10:
                    break
            time.sleep(0.01)

        if response_data:
            print(f"Command response: {[f'0x{b:02X}' for b in response_data[:10]]}")
            print(f"Response length: {len(response_data)} bytes")
        else:
            print("[ERROR] No command response")

        # Recommendations
        print("\n=== Recommendations ===")
        if total_bytes > 1000:
            print("• Arduino is sending excessive data - check for infinite loops in Arduino code")
        if 0xFF in byte_counts and byte_counts.get(0xFF, 0) > total_bytes * 0.5:
            print("• 0xFF flood detected - Arduino may be in uninitialized state")
            print("• Try power cycling the Arduino")
            print("• Check Arduino code for proper initialization")
        if total_bytes == 0:
            print("• No data received - check Arduino power and code upload")

        ser.close()

    except Exception as e:
        print(f"Error: {e}")
        return False

    return True

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python arduino_diagnostic.py <port>")
        print("Example: python arduino_diagnostic.py COM4")
        sys.exit(1)

    port = sys.argv[1]
    diagnose_arduino(port)