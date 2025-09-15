#!/usr/bin/env python3
"""
Falcon BMS Multicast Client for Linux (Raspberry Pi)
Receives BMS shared memory data via UDP multicast and communicates with Arduino

Based on RTTServer.exe multicast protocol and ArduinoConnector.cs communication pattern
"""

import socket
import struct
import threading
import time
import serial
import serial.tools.list_ports
import argparse
import sys
from typing import Optional, Dict, Any, List
from dataclasses import dataclass
from enum import IntEnum


class SMEMDataType(IntEnum):
    """Shared memory data types as defined in RTTNetData.h"""
    FLIGHT_DATA = 0
    FLIGHT_DATA2 = 1
    OSB_DATA = 2
    INTELLIVIBE_DATA = 3


@dataclass
class MulticastHeader:
    """Multicast packet header structure"""
    identifier: bytes  # 'SHM'
    version: int       # Currently 1
    sequence: int      # Sequence counter
    data_type: int     # SMEMDataType


class FlightDataParser:
    """Parser for FlightData structure from BMS shared memory"""

    # FlightData structure layout (based on FlightData.h)
    FLIGHT_DATA_FORMAT = (
        'f' * 3 +      # x, y, z (position)
        'f' * 3 +      # xDot, yDot, zDot (velocity)
        'f' * 6 +      # alpha, beta, gamma, pitch, roll, yaw
        'f' * 7 +      # mach, kias, vt, gs, windOffset, nozzlePos, internalFuel
        'f' * 4 +      # externalFuel, fuelFlow, rpm, ftit
        'f' * 3 +      # gearPos, speedBrake, epuFuel, oilPressure
        'I' +          # lightBits (unsigned int)
        'f' * 3 +      # headPitch, headRoll, headYaw
        'I' * 2 +      # lightBits2, lightBits3
        'f' * 2 +      # ChaffCount, FlareCount
        'f' * 3 +      # NoseGearPos, LeftGearPos, RightGearPos
        'f' * 2 +      # AdiIlsHorPos, AdiIlsVerPos
        'i' * 3 +      # courseState, headingState, totalStates
        'f' * 9 +      # HSI values
        'f' * 3 +      # TrimPitch, TrimRoll, TrimYaw
        'I' +          # hsiBits
        '26s' * 5 +    # DEDLines[5][26]
        '26s' * 5 +    # Invert[5][26]
        '26s' * 5 +    # PFLLines[5][26]
        '26s' * 5 +    # PFLInvert[5][26]
        'i' * 2 +      # UFCTChan, AUXTChan
        'i' +          # RwrObjectCount
        'i' * 40 +     # RWRsymbol[40]
        'f' * 40 +     # bearing[40]
        'L' * 40 +     # missileActivity[40]
        'L' * 40 +     # missileLaunch[40]
        'L' * 40 +     # selected[40]
        'f' * 40 +     # lethality[40]
        'L' * 40 +     # newDetection[40]
        'f' * 3 +      # fwd, aft, total
        'i' +          # VersionNum
        'f' * 3 +      # headX, headY, headZ
        'i'            # MainPower
    )


class FlightData2Parser:
    """Parser for FlightData2 structure"""

    # Simplified FlightData2 format (key fields)
    FLIGHT_DATA2_FORMAT = (
        'f' * 4 +      # nozzlePos2, rpm2, ftit2, oilPressure2
        'B' +          # navMode
        'f' +          # AAUZ
        'B' * 2 +      # tacanInfo[2]
        'i' +          # AltCalReading
        'I' * 4 +      # altBits, powerBits, blinkBits, cmdsMode
        'i' * 2 +      # uhf_panel_preset, uhf_panel_frequency
        'f' * 3 +      # cabinAlt, hydPressureA, hydPressureB
        'i' * 2 +      # currentTime, vehicleACD, VersionNum
        'f' +          # fuelFlow2
        # ... more fields as needed
    )


class FalconBMSMulticastClient:
    """Main client class for receiving BMS multicast data and communicating with Arduino"""

    def __init__(self, multicast_group: str, port: int, arduino_port: str = None):
        self.multicast_group = multicast_group
        self.port = port
        self.arduino_port = arduino_port

        # Multicast socket
        self.socket = None
        self.running = False

        # Arduino serial connection
        self.arduino_serial: Optional[serial.Serial] = None
        self.arduino_connected = False

        # Data storage
        self.flight_data: Dict[str, Any] = {}
        self.flight_data2: Dict[str, Any] = {}
        self.last_sequence = 0

        # Threading
        self.multicast_thread = None
        self.arduino_thread = None

    def start(self):
        """Start the multicast client and Arduino communication"""
        print(f"Starting Falcon BMS Multicast Client")
        print(f"Multicast: {self.multicast_group}:{self.port}")

        # Setup multicast reception
        self._setup_multicast()

        # Setup Arduino connection if specified
        if self.arduino_port:
            self._setup_arduino()

        # Start threads
        self.running = True
        self.multicast_thread = threading.Thread(target=self._multicast_loop, daemon=True)
        self.multicast_thread.start()

        if self.arduino_connected:
            self.arduino_thread = threading.Thread(target=self._arduino_loop, daemon=True)
            self.arduino_thread.start()

        print("Client started successfully")

    def stop(self):
        """Stop the client"""
        print("Stopping client...")
        self.running = False

        if self.socket:
            self.socket.close()

        if self.arduino_serial:
            self.arduino_serial.close()

        if self.multicast_thread:
            self.multicast_thread.join(timeout=1.0)

        if self.arduino_thread:
            self.arduino_thread.join(timeout=1.0)

        print("Client stopped")

    def _setup_multicast(self):
        """Setup UDP multicast reception"""
        try:
            # Create socket
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

            # Bind to the multicast group
            self.socket.bind(('', self.port))

            # Join multicast group
            mreq = struct.pack("4sl", socket.inet_aton(self.multicast_group), socket.INADDR_ANY)
            self.socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

            # Set socket timeout for clean shutdown
            self.socket.settimeout(1.0)

            print(f"Multicast socket setup complete: {self.multicast_group}:{self.port}")

        except Exception as e:
            print(f"Failed to setup multicast: {e}")
            sys.exit(1)

    def _setup_arduino(self):
        """Setup Arduino serial connection with handshake"""
        try:
            print(f"Connecting to Arduino on {self.arduino_port}...")

            # Open serial connection
            self.arduino_serial = serial.Serial(
                port=self.arduino_port,
                baudrate=115200,
                parity=serial.PARITY_NONE,
                databits=8,
                stopbits=serial.STOPBITS_ONE,
                timeout=1.0,
                write_timeout=1.0
            )

            # Wait a moment for Arduino to initialize
            time.sleep(1.0)

            # Perform handshake
            if self._arduino_handshake():
                self.arduino_connected = True
                print("Arduino connection established successfully")
            else:
                print("Arduino handshake failed")
                self.arduino_serial.close()
                self.arduino_serial = None

        except Exception as e:
            print(f"Failed to connect to Arduino: {e}")

    def _arduino_handshake(self) -> bool:
        """Perform handshake with Arduino (matches ArduinoConnector.cs)"""
        try:
            # Send handshake byte 0xA5
            self.arduino_serial.write(bytes([0xA5]))
            print("Sent handshake byte (0xA5)")

            # Wait for response 0x5A
            start_time = time.time()
            while time.time() - start_time < 1.0:  # 1 second timeout
                if self.arduino_serial.in_waiting > 0:
                    response = self.arduino_serial.read(1)[0]
                    if response == 0x5A:
                        print("Handshake successful! Received 0x5A")
                        return True
                    else:
                        print(f"Unexpected handshake response: 0x{response:02X}")
                        return False
                time.sleep(0.01)

            print("Handshake timeout - no response from Arduino")
            return False

        except Exception as e:
            print(f"Handshake error: {e}")
            return False

    def _multicast_loop(self):
        """Main loop for receiving multicast data"""
        print("Multicast receive loop started")

        while self.running:
            try:
                # Receive multicast packet
                data, addr = self.socket.recvfrom(65536)  # Max UDP packet size

                # Parse header
                header = self._parse_header(data)
                if not header:
                    continue

                # Check sequence (detect out-of-order packets)
                if header.sequence <= self.last_sequence:
                    print(f"Out-of-order packet: seq={header.sequence}, last={self.last_sequence}")
                    continue

                self.last_sequence = header.sequence

                # Parse data based on type
                payload = data[9:]  # Skip header
                if header.data_type == SMEMDataType.FLIGHT_DATA:
                    self._parse_flight_data(payload)
                elif header.data_type == SMEMDataType.FLIGHT_DATA2:
                    self._parse_flight_data2(payload)
                elif header.data_type == SMEMDataType.INTELLIVIBE_DATA:
                    self._parse_intellivibe_data(payload)

            except socket.timeout:
                continue  # Normal timeout, check running flag
            except Exception as e:
                if self.running:
                    print(f"Multicast receive error: {e}")

    def _parse_header(self, data: bytes) -> Optional[MulticastHeader]:
        """Parse multicast packet header"""
        if len(data) < 9:
            return None

        # Check identifier
        if data[0:3] != b'SHM':
            return None

        version = data[3]
        sequence = struct.unpack('<I', data[4:8])[0]  # Little-endian uint32
        data_type = data[8]

        return MulticastHeader(
            identifier=data[0:3],
            version=version,
            sequence=sequence,
            data_type=data_type
        )

    def _parse_flight_data(self, data: bytes):
        """Parse FlightData structure"""
        try:
            if len(data) < 200:  # Minimum size check
                return

            # Parse position and attitude (first 12 floats)
            pos_att = struct.unpack('<12f', data[0:48])
            self.flight_data.update({
                'x': pos_att[0], 'y': pos_att[1], 'z': pos_att[2],
                'xDot': pos_att[3], 'yDot': pos_att[4], 'zDot': pos_att[5],
                'alpha': pos_att[6], 'beta': pos_att[7], 'gamma': pos_att[8],
                'pitch': pos_att[9], 'roll': pos_att[10], 'yaw': pos_att[11]
            })

            # Parse engine and flight data (next floats)
            engine_data = struct.unpack('<8f', data[48:80])
            self.flight_data.update({
                'mach': engine_data[0], 'kias': engine_data[1], 'vt': engine_data[2],
                'gs': engine_data[3], 'windOffset': engine_data[4], 'nozzlePos': engine_data[5],
                'internalFuel': engine_data[6], 'externalFuel': engine_data[7]
            })

            # Parse more engine data
            engine_data2 = struct.unpack('<4f', data[80:96])
            self.flight_data.update({
                'fuelFlow': engine_data2[0], 'rpm': engine_data2[1], 'ftit': engine_data2[2],
                'gearPos': engine_data2[3]
            })

            # Parse remaining floats
            misc_data = struct.unpack('<3f', data[96:108])
            self.flight_data.update({
                'speedBrake': misc_data[0], 'epuFuel': misc_data[1], 'oilPressure': misc_data[2]
            })

            # Parse lightBits (uint32)
            if len(data) >= 112:
                lightbits = struct.unpack('<I', data[108:112])[0]
                self.flight_data['lightBits'] = lightbits

            # Skip head position floats (12 bytes)
            offset = 124

            # Parse lightBits2 and lightBits3
            if len(data) >= offset + 8:
                light_bits_23 = struct.unpack('<II', data[offset:offset+8])
                self.flight_data.update({
                    'lightBits2': light_bits_23[0],
                    'lightBits3': light_bits_23[1]
                })
                offset += 8

            # Parse chaff and flare counts
            if len(data) >= offset + 8:
                counts = struct.unpack('<ff', data[offset:offset+8])
                self.flight_data.update({
                    'chaffCount': counts[0],
                    'flareCount': counts[1]
                })
                offset += 8

            # Parse gear positions
            if len(data) >= offset + 12:
                gear_pos = struct.unpack('<fff', data[offset:offset+12])
                self.flight_data.update({
                    'noseGearPos': gear_pos[0],
                    'leftGearPos': gear_pos[1],
                    'rightGearPos': gear_pos[2]
                })
                offset += 12

            # Skip ADI values (8 bytes) and HSI states (12 bytes)
            offset += 20

            # Parse HSI values
            if len(data) >= offset + 36:
                hsi_values = struct.unpack('<9f', data[offset:offset+36])
                self.flight_data.update({
                    'courseDeviation': hsi_values[0],
                    'desiredCourse': hsi_values[1],
                    'distanceToBeacon': hsi_values[2],
                    'bearingToBeacon': hsi_values[3],
                    'currentHeading': hsi_values[4],
                    'desiredHeading': hsi_values[5]
                })
                offset += 36

            # Skip trim values and hsiBits
            offset += 16

            # Parse DED lines (5 lines of 26 chars each)
            if len(data) >= offset + 260:  # 5 * 26 * 2 (DED + Invert)
                for i in range(5):
                    line_start = offset + (i * 26)
                    invert_start = offset + 130 + (i * 26)

                    ded_line = data[line_start:line_start+26].decode('ascii', errors='ignore').rstrip('\x00')
                    ded_invert = data[invert_start:invert_start+26]

                    self.flight_data[f'DEDLine_{i}'] = ded_line
                    self.flight_data[f'DEDInvert_{i}'] = ded_invert
                offset += 260

            # Parse PFL lines (5 lines of 26 chars each)
            if len(data) >= offset + 260:
                for i in range(5):
                    line_start = offset + (i * 26)
                    invert_start = offset + 130 + (i * 26)

                    pfl_line = data[line_start:line_start+26].decode('ascii', errors='ignore').rstrip('\x00')
                    pfl_invert = data[invert_start:invert_start+26]

                    self.flight_data[f'PFLLine_{i}'] = pfl_line
                    self.flight_data[f'PFLInvert_{i}'] = pfl_invert
                offset += 260

            # Skip TACAN channels
            offset += 8

            # Skip RWR data for now (large block)
            # Parse fuel quantities at the end
            fuel_offset = len(data) - 16  # Approximate location
            if fuel_offset > 0 and len(data) >= fuel_offset + 12:
                fuel_data = struct.unpack('<fff', data[fuel_offset:fuel_offset+12])
                self.flight_data.update({
                    'fwd': fuel_data[0],
                    'aft': fuel_data[1],
                    'total': fuel_data[2]
                })

        except Exception as e:
            print(f"Error parsing FlightData: {e}")

    def _parse_flight_data2(self, data: bytes):
        """Parse FlightData2 structure"""
        try:
            if len(data) < 50:
                return

            offset = 0

            # Parse engine data
            engine_data = struct.unpack('<ffff', data[offset:offset+16])
            self.flight_data2.update({
                'nozzlePos2': engine_data[0],
                'rpm2': engine_data[1],
                'ftit2': engine_data[2],
                'oilPressure2': engine_data[3]
            })
            offset += 16

            # Parse nav mode and AAUZ
            if len(data) >= offset + 5:
                nav_data = struct.unpack('<Bf', data[offset:offset+5])
                self.flight_data2.update({
                    'navMode': nav_data[0],
                    'AAUZ': nav_data[1]
                })
                offset += 5

            # Skip tacan info (2 bytes)
            offset += 2

            # Parse altimeter and system bits
            if len(data) >= offset + 20:
                system_data = struct.unpack('<iIIII', data[offset:offset+20])
                self.flight_data2.update({
                    'altCalReading': system_data[0],
                    'altBits': system_data[1],
                    'powerBits': system_data[2],
                    'blinkBits': system_data[3],
                    'cmdsMode': system_data[4]
                })
                offset += 20

            # Parse UHF data
            if len(data) >= offset + 8:
                uhf_data = struct.unpack('<ii', data[offset:offset+8])
                self.flight_data2.update({
                    'uhf_panel_preset': uhf_data[0],
                    'uhf_panel_frequency': uhf_data[1]
                })
                offset += 8

            # Parse hydraulic and cabin data
            if len(data) >= offset + 12:
                fluid_data = struct.unpack('<fff', data[offset:offset+12])
                self.flight_data2.update({
                    'cabinAlt': fluid_data[0],
                    'hydPressureA': fluid_data[1],
                    'hydPressureB': fluid_data[2]
                })
                offset += 12

            # Parse time and version
            if len(data) >= offset + 12:
                time_data = struct.unpack('<ihi', data[offset:offset+12])
                self.flight_data2.update({
                    'currentTime': time_data[0],
                    'vehicleACD': time_data[1],
                    'versionNum': time_data[2]
                })
                offset += 12

            # Parse fuel flow 2
            if len(data) >= offset + 4:
                fuel_flow2 = struct.unpack('<f', data[offset:offset+4])[0]
                self.flight_data2['fuelFlow2'] = fuel_flow2
                offset += 4

            # Skip RWR info and other fields for now
            # Look for instrument light and other fields at expected offsets
            if len(data) >= 200:  # Rough estimate
                try:
                    # Try to find instrument light (should be a single byte)
                    instr_offset = 180  # Approximate
                    if len(data) > instr_offset:
                        instr_light = data[instr_offset]
                        self.flight_data2['instrLight'] = instr_light

                    # Try to find flood console
                    flood_offset = 220  # Approximate
                    if len(data) > flood_offset:
                        flood_console = data[flood_offset]
                        self.flight_data2['floodConsole'] = flood_console

                    # Try to find ECM bits (array of 5 uint32)
                    ecm_offset = 240  # Approximate
                    if len(data) >= ecm_offset + 20:
                        ecm_bits = list(struct.unpack('<5I', data[ecm_offset:ecm_offset+20]))
                        self.flight_data2['ecmBits'] = ecm_bits

                    # Try to find IFF backup digits
                    iff_offset = 160  # Approximate
                    if len(data) >= iff_offset + 4:
                        iff_data = struct.unpack('<bbbb', data[iff_offset:iff_offset+4])
                        self.flight_data2.update({
                            'iffBackupMode1Digit1': iff_data[0],
                            'iffBackupMode1Digit2': iff_data[1],
                            'iffBackupMode3ADigit1': iff_data[2],
                            'iffBackupMode3ADigit2': iff_data[3]
                        })

                except:
                    pass  # Skip if can't parse optional fields

        except Exception as e:
            print(f"Error parsing FlightData2: {e}")

    def _parse_intellivibe_data(self, data: bytes):
        """Parse IntellivibeData structure"""
        try:
            if len(data) >= 32:
                # Parse basic structure
                values = struct.unpack('<BBBBBB', data[0:6])
                print(f"IntellivibeData: AA missiles={values[0]}, Bullets={values[5]}")
        except Exception as e:
            print(f"Error parsing IntellivibeData: {e}")

    def _arduino_loop(self):
        """Main loop for Arduino communication"""
        print("Arduino communication loop started")

        while self.running and self.arduino_connected:
            try:
                if self.arduino_serial.in_waiting > 0:
                    # Read command from Arduino
                    command = self.arduino_serial.read(1)[0]
                    print(f"Received Arduino command: 0x{command:02X}")

                    # Process command and send response
                    self._process_arduino_command(command)

                time.sleep(0.01)  # Small delay to prevent busy waiting

            except Exception as e:
                print(f"Arduino communication error: {e}")
                self.arduino_connected = False
                break

    def _process_arduino_command(self, command: int):
        """Process command from Arduino and send response (matches ArduinoConnector.cs)"""
        try:
            response_data = bytes([0x00])  # Default response

            # Check if flight data is available
            if not self.flight_data:
                self._send_arduino_response(command, bytes([0x00]))
                return

            if command == 0x01:  # lightBits
                response_data = struct.pack('<I', self.flight_data.get('lightBits', 0))

            elif command == 0x02:  # lightBits2
                response_data = struct.pack('<I', self.flight_data2.get('lightBits2', 0))

            elif command == 0x03:  # lightBits3
                response_data = struct.pack('<I', self.flight_data2.get('lightBits3', 0))

            elif command == 0x04:  # blinkBits
                response_data = struct.pack('<I', self.flight_data2.get('blinkBits', 0))

            elif command == 0x05:  # DED Lines (merged 120 bytes)
                merged_ded = bytearray(120)
                for i in range(5):
                    line_key = f'DEDLine_{i}'
                    invert_key = f'DEDInvert_{i}'
                    if line_key in self.flight_data and invert_key in self.flight_data:
                        norm = self._normalize_line(self.flight_data[line_key], self.flight_data[invert_key])
                        merged_ded[i*24:(i+1)*24] = norm
                response_data = bytes(merged_ded)

            elif command == 0x06:  # fuelFlow
                response_data = struct.pack('<f', self.flight_data.get('fuelFlow', 0.0))

            elif command == 0x07:  # instrLight
                response_data = struct.pack('<B', self.flight_data2.get('instrLight', 0))

            elif command == 0x08:  # PFL Lines (merged 120 bytes)
                merged_pfl = bytearray(120)
                for i in range(5):
                    line_key = f'PFLLine_{i}'
                    invert_key = f'PFLInvert_{i}'
                    if line_key in self.flight_data and invert_key in self.flight_data:
                        norm = self._normalize_line(self.flight_data[line_key], self.flight_data[invert_key])
                        merged_pfl[i*24:(i+1)*24] = norm
                response_data = bytes(merged_pfl)

            elif command == 0x09:  # chaffCount
                response_data = struct.pack('<f', self.flight_data.get('chaffCount', 0.0))

            elif command == 0x10:  # flareCount
                response_data = struct.pack('<f', self.flight_data.get('flareCount', 0.0))

            elif command == 0x11:  # floodConsole
                response_data = struct.pack('<B', self.flight_data2.get('floodConsole', 0))

            elif command == 0x12:  # rpm
                response_data = struct.pack('<f', self.flight_data.get('rpm', 0.0))

            elif command == 0x13:  # ecmBits (array of 5 uint32)
                ecm_bits = self.flight_data2.get('ecmBits', [0] * 5)
                response_data = b''.join(struct.pack('<I', bit) for bit in ecm_bits[:5])

            elif command == 0x14:  # oilPressure
                response_data = struct.pack('<f', self.flight_data.get('oilPressure', 0.0))

            elif command == 0x15:  # oilPressure2
                response_data = struct.pack('<f', self.flight_data2.get('oilPressure2', 0.0))

            elif command == 0x16:  # nozzlePos
                response_data = struct.pack('<f', self.flight_data.get('nozzlePos', 0.0))

            elif command == 0x17:  # nozzlePos2
                response_data = struct.pack('<f', self.flight_data2.get('nozzlePos2', 0.0))

            elif command == 0x18:  # ftit
                response_data = struct.pack('<f', self.flight_data.get('ftit', 0.0))

            elif command == 0x19:  # ftit2
                response_data = struct.pack('<f', self.flight_data2.get('ftit2', 0.0))

            elif command == 0x20:  # cabinAlt
                response_data = struct.pack('<f', self.flight_data2.get('cabinAlt', 0.0))

            elif command == 0x21:  # kias
                response_data = struct.pack('<f', self.flight_data.get('kias', 0.0))

            elif command == 0x22:  # internalFuel
                response_data = struct.pack('<f', self.flight_data.get('internalFuel', 0.0))

            elif command == 0x23:  # externalFuel
                response_data = struct.pack('<f', self.flight_data.get('externalFuel', 0.0))

            elif command == 0x24:  # epuFuel
                response_data = struct.pack('<f', self.flight_data.get('epuFuel', 0.0))

            elif command == 0x25:  # hydPressureA
                response_data = struct.pack('<f', self.flight_data2.get('hydPressureA', 0.0))

            elif command == 0x26:  # hydPressureB
                response_data = struct.pack('<f', self.flight_data2.get('hydPressureB', 0.0))

            elif command == 0x27:  # cmdsMode
                response_data = struct.pack('<i', self.flight_data2.get('cmdsMode', 0))

            elif command == 0x28:  # uhf_panel_preset
                response_data = struct.pack('<i', self.flight_data2.get('uhf_panel_preset', 0))

            elif command == 0x29:  # uhf_panel_frequency
                response_data = struct.pack('<i', self.flight_data2.get('uhf_panel_frequency', 0))

            elif command == 0x30:  # speedBrake
                response_data = struct.pack('<f', self.flight_data.get('speedBrake', 0.0))

            elif command == 0x31:  # iffBackupMode1Digit1
                response_data = struct.pack('<b', self.flight_data2.get('iffBackupMode1Digit1', 0))

            elif command == 0x32:  # iffBackupMode1Digit2
                response_data = struct.pack('<b', self.flight_data2.get('iffBackupMode1Digit2', 0))

            elif command == 0x33:  # iffBackupMode3ADigit1
                response_data = struct.pack('<b', self.flight_data2.get('iffBackupMode3ADigit1', 0))

            elif command == 0x34:  # iffBackupMode3ADigit2
                response_data = struct.pack('<b', self.flight_data2.get('iffBackupMode3ADigit2', 0))

            elif command == 0x35:  # fwd fuel
                response_data = struct.pack('<f', self.flight_data.get('fwd', 0.0))

            elif command == 0x36:  # aft fuel
                response_data = struct.pack('<f', self.flight_data.get('aft', 0.0))

            elif command == 0x37:  # total fuel
                response_data = struct.pack('<f', self.flight_data.get('total', 0.0))

            elif command == 0x38:  # desiredCourse
                response_data = struct.pack('<f', self.flight_data.get('desiredCourse', 0.0))

            elif command == 0x39:  # courseDeviation
                response_data = struct.pack('<f', self.flight_data.get('courseDeviation', 0.0))

            elif command == 0x40:  # distanceToBeacon
                response_data = struct.pack('<f', self.flight_data.get('distanceToBeacon', 0.0))

            elif command == 0x41:  # bearingToBeacon
                response_data = struct.pack('<f', self.flight_data.get('bearingToBeacon', 0.0))

            elif command == 0x99:  # Packet Failed
                print("Arduino reported packet checksum failure")
                return  # Don't send response

            elif command == 0x0F:  # Ping response
                response_data = bytes([0xAB])

            elif command == 0x5A:  # Handshake response
                response_data = bytes([0x5A])

            else:
                print(f"Unknown command: 0x{command:02X}")
                response_data = bytes([0x00])

            # Send response using same protocol as ArduinoConnector.cs
            self._send_arduino_response(command, response_data)

        except Exception as e:
            print(f"Error processing Arduino command 0x{command:02X}: {e}")

    def _send_arduino_response(self, command_type: int, data: bytes):
        """Send response to Arduino (matches ArduinoConnector.cs SendResponse method)"""
        try:
            # Build packet: [0xAA][type][length][data][checksum]
            packet = bytearray()
            packet.append(0xAA)  # Start byte
            packet.append(command_type)  # Command type
            packet.append(len(data))  # Data length
            packet.extend(data)  # Data payload

            # Calculate checksum: type + length + sum(data)
            checksum = (command_type + len(data) + sum(data)) & 0xFF
            packet.append(checksum)

            # Send packet
            self.arduino_serial.write(packet)
            print(f"Sent Arduino response for type 0x{command_type:02X}, length={len(data)}")

        except Exception as e:
            print(f"Error sending Arduino response: {e}")

    def _normalize_line(self, disp_line: str, invert_line: bytes) -> bytes:
        """
        Normalize DED/PFL line with inversion handling
        Based on ArduinoConnector.cs NormalizeLine method
        """
        norm_line = bytearray(24)  # Arduino expects 24 characters

        # Ensure we don't exceed the display line length
        line_len = min(len(disp_line), 24, len(invert_line))

        for j in range(line_len):
            char = disp_line[j] if j < len(disp_line) else ' '
            inv = invert_line[j] if j < len(invert_line) else 0

            if inv == 2:  # Character is inverted
                if char.isupper():  # Uppercase letter
                    norm_line[j] = ord(char.lower())  # Lowercase = inverted in custom font
                elif ord(char) == 1:  # Selection arrows
                    norm_line[j] = 192
                elif ord(char) == 2:  # DED "*"
                    norm_line[j] = 170
                elif ord(char) == 3:  # DED "_"
                    norm_line[j] = 223
                elif char == '~':  # Arrow down (PFD)
                    norm_line[j] = 252
                elif char == '^':  # Degree symbol
                    norm_line[j] = 222
                else:  # Everything else - add 128 to ASCII
                    norm_line[j] = (ord(char) + 128) & 0xFF
            else:  # Non-inverted
                if ord(char) == 1:  # Selector double arrow
                    norm_line[j] = ord('@')
                elif ord(char) == 2:  # DED "*"
                    norm_line[j] = ord('*')
                elif ord(char) == 3:  # DED "_"
                    norm_line[j] = ord('_')
                elif char == '~':  # Arrow down (PFD)
                    norm_line[j] = ord('|')
                else:
                    norm_line[j] = ord(char)

        # Fill remaining positions with spaces
        for j in range(line_len, 24):
            norm_line[j] = ord(' ')

        return bytes(norm_line)


def list_serial_ports():
    """List available serial ports"""
    ports = serial.tools.list_ports.comports()
    print("Available serial ports:")
    for port in ports:
        print(f"  {port.device} - {port.description}")


def main():
    parser = argparse.ArgumentParser(description='Falcon BMS Multicast Client for Linux')
    parser.add_argument('--multicast-group', default='224.0.0.44',
                       help='Multicast group IP address (default: 224.0.0.44)')
    parser.add_argument('--port', type=int, default=44000,
                       help='Multicast port (default: 44000)')
    parser.add_argument('--arduino-port',
                       help='Arduino serial port (e.g., /dev/ttyUSB0)')
    parser.add_argument('--list-ports', action='store_true',
                       help='List available serial ports and exit')

    args = parser.parse_args()

    if args.list_ports:
        list_serial_ports()
        return

    # Create and start client
    client = FalconBMSMulticastClient(
        multicast_group=args.multicast_group,
        port=args.port,
        arduino_port=args.arduino_port
    )

    try:
        client.start()

        # Keep running until interrupted
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nShutdown requested...")
    finally:
        client.stop()


if __name__ == '__main__':
    main()