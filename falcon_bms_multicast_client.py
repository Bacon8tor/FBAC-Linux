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
    FLIGHT_DATA = 0    # F4 in ESP32 code
    FLIGHT_DATA2 = 1   # BMS in ESP32 code
    OSB_DATA = 2       # OSB in ESP32 code
    INTELLIVIBE_DATA = 3  # IVIBE in ESP32 code


@dataclass
class MulticastHeader:
    """Multicast packet header structure"""
    identifier: bytes  # 'SHM'
    version: int       # Currently 1
    sequence: int      # Sequence counter
    data_type: int     # SMEMDataType


class FlightDataParser:
    """Parser for FlightData structure from BMS shared memory"""

    # Complete FlightData structure layout (based on FlightData.h)
    # Total size should be approximately 2348 bytes
    FLIGHT_DATA_FORMAT = (
        # Basic position and attitude data (12 floats = 48 bytes)
        'f' * 12 +     # x, y, z, xDot, yDot, zDot, alpha, beta, gamma, pitch, roll, yaw

        # Engine and flight data (7 floats = 28 bytes)
        'f' * 7 +      # mach, kias, vt, gs, windOffset, nozzlePos, internalFuel

        # More engine data (4 floats = 16 bytes)
        'f' * 4 +      # externalFuel, fuelFlow, rpm, ftit

        # Gear and systems (3 floats = 12 bytes)
        'f' * 3 +      # gearPos, speedBrake, epuFuel, oilPressure

        # Light bits (1 uint32 = 4 bytes)
        'I' +          # lightBits

        # Head position (3 floats = 12 bytes)
        'f' * 3 +      # headPitch, headRoll, headYaw

        # More light bits (2 uint32 = 8 bytes)
        'I' * 2 +      # lightBits2, lightBits3

        # Chaff/Flare (2 floats = 8 bytes)
        'f' * 2 +      # ChaffCount, FlareCount

        # Gear positions (3 floats = 12 bytes)
        'f' * 3 +      # NoseGearPos, LeftGearPos, RightGearPos

        # ADI values (2 floats = 8 bytes)
        'f' * 2 +      # AdiIlsHorPos, AdiIlsVerPos

        # HSI states (3 int32 = 12 bytes)
        'i' * 3 +      # courseState, headingState, totalStates

        # HSI values (9 floats = 36 bytes)
        'f' * 9 +      # courseDeviation through totalValues

        # Trim values (3 floats = 12 bytes)
        'f' * 3 +      # TrimPitch, TrimRoll, TrimYaw

        # HSI bits (1 uint32 = 4 bytes)
        'I' +          # hsiBits

        # DED Lines (5 * 26 bytes = 130 bytes)
        '26s' * 5 +    # DEDLines[5][26]

        # DED Invert (5 * 26 bytes = 130 bytes)
        '26s' * 5 +    # Invert[5][26]

        # PFL Lines (5 * 26 bytes = 130 bytes)
        '26s' * 5 +    # PFLLines[5][26]

        # PFL Invert (5 * 26 bytes = 130 bytes)
        '26s' * 5 +    # PFLInvert[5][26]

        # TACAN channels (2 int32 = 8 bytes)
        'i' * 2 +      # UFCTChan, AUXTChan

        # RWR data (1 int32 + 40*6 values = 964 bytes)
        'i' +          # RwrObjectCount
        'i' * 40 +     # RWRsymbol[40]
        'f' * 40 +     # bearing[40]
        'L' * 40 +     # missileActivity[40]
        'L' * 40 +     # missileLaunch[40]
        'L' * 40 +     # selected[40]
        'f' * 40 +     # lethality[40]
        'L' * 40 +     # newDetection[40]

        # Fuel values (3 floats = 12 bytes)
        'f' * 3 +      # fwd, aft, total

        # Version and final fields (4 values = 16 bytes)
        'i' +          # VersionNum
        'f' * 3 +      # headX, headY, headZ
        'i'            # MainPower
    )

    # Expected total size (for validation)
    EXPECTED_SIZE = struct.calcsize('<' + ''.join(FLIGHT_DATA_FORMAT))


class FlightData2Parser:
    """Parser for FlightData2 structure"""

    # Complete FlightData2 structure layout (based on FlightData.h)
    # This structure has grown significantly in BMS 4.35+
    FLIGHT_DATA2_FORMAT = (
        # VERSION 1 - Initial engine data (4 floats + 1 byte + 1 float + 2 bytes)
        'f' * 4 +      # nozzlePos2, rpm2, ftit2, oilPressure2
        'B' +          # navMode (NavModes enum)
        'f' +          # AAUZ (barometric altitude)
        'B' * 2 +      # tacanInfo[2] (band/mode for UFC and AUX)

        # VERSION 2/7 - Altimeter and power systems
        'i' +          # AltCalReading
        'I' * 4 +      # altBits, powerBits, blinkBits, cmdsMode
        'i' * 2 +      # uhf_panel_preset, uhf_panel_frequency

        # VERSION 3 - Additional systems
        'f' * 3 +      # cabinAlt, hydPressureA, hydPressureB
        'i' +          # currentTime (seconds in day)
        'h' +          # vehicleACD (aircraft type index)
        'i' +          # VersionNum

        # VERSION 4 - Additional fuel flow
        'f' +          # fuelFlow2

        # VERSION 5/8 - RWR and flight surfaces (512 + 2 floats)
        '512s' +       # RwrInfo[512] - Raw RWR data
        'f' * 2 +      # lefPos, tefPos

        # VERSION 6 - VTOL
        'f' +          # vtolPos

        # VERSION 9 - Multiplayer pilot info
        'B' +          # pilotsOnline
        '12s' * 32 +   # pilotsCallsign[32][12]
        'B' * 32 +     # pilotsStatus[32]

        # VERSION 10 - Taxi effects
        'f' +          # bumpIntensity

        # VERSION 11 - GPS coordinates
        'f' * 2 +      # latitude, longitude

        # VERSION 12 - RTT (render-to-texture) info
        'H' * 2 +      # RTT_size[2] (width, height)
        'H' * 28 +     # RTT_area[7][4] (left, top, right, bottom for each area)

        # VERSION 13 - IFF backup digits
        'b' * 4 +      # iffBackupMode1Digit1/2, iffBackupMode3ADigit1/2

        # VERSION 14 - Instrument lighting
        'B' +          # instrLight

        # VERSION 15 - Betty, misc bits, radar alt, bingo, bullseye, version info
        'I' * 2 +      # bettyBits, miscBits
        'f' * 5 +      # RALT, bingoFuel, caraAlow, bullseyeX, bullseyeY
        'i' * 4 +      # BMSVersionMajor/Minor/Micro, BMSBuildNumber
        'I' * 3 +      # StringAreaSize, StringAreaTime, DrawingAreaSize

        # VERSION 16 - Turn rate
        'f' +          # turnRate

        # VERSION 18 - Flood console lighting
        'B' +          # floodConsole

        # VERSION 19 - Magnetic deviation and ECM
        'f' * 2 +      # magDeviationSystem, magDeviationReal
        'I' * 5 +      # ecmBits[5] - ECM program states
        'B' +          # ecmOper (ECM operator state)
        'B' * 40 +     # RWRjammingStatus[40] - Jamming status for each RWR object

        # VERSION 20 - Radio 2 and IFF transponder
        'i' * 2 +      # radio2_preset, radio2_frequency
        'b' +          # iffTransponderActiveCode1
        'h' * 4 +      # iffTransponderActiveCode2/3A/C/4

        # VERSION 21 - TACAN ILS
        'i' +          # tacan_ils_frequency

        # VERSION 22 - RTT FPS and side slip
        'i' +          # desired_RTT_FPS
        'f'            # sideSlipdeg
    )

    # Expected total size (for validation)
    EXPECTED_SIZE = struct.calcsize('<' + ''.join(FLIGHT_DATA2_FORMAT))


class FalconBMSMulticastClient:
    """Main client class for receiving BMS multicast data and communicating with Arduino"""

    def __init__(self, multicast_group: str, port: int, arduino_port: str = None, debug: bool = False, print_data: bool = False):
        self.multicast_group = multicast_group
        self.port = port
        self.arduino_port = arduino_port
        self.debug = debug
        self.print_data = print_data

        # Multicast socket
        self.socket = None
        self.running = False

        # Arduino serial connection
        self.arduino_serial: Optional[serial.Serial] = None
        self.arduino_connected = False

        # Data storage
        self.flight_data: Dict[str, Any] = {}
        self.flight_data2: Dict[str, Any] = {}
        self.intellivibe_data: Dict[str, Any] = {}
        self.osb_data: Dict[str, Any] = {}
        self.last_sequence = 0
        self.first_packet_received = False
        self.last_packet_time = 0
        self.udp_timeout = 5.0  # 5 seconds timeout like ESP32

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
            print("[INFO] Disconnecting from multicast group...")
            self.socket.close()

        if self.arduino_serial:
            print("[INFO] Disconnecting from Arduino...")
            self.arduino_serial.close()

        if self.multicast_thread:
            self.multicast_thread.join(timeout=1.0)

        if self.arduino_thread:
            self.arduino_thread.join(timeout=1.0)

        print("[OK] Client stopped")

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

            print(f"[OK] Connected to multicast group: {self.multicast_group}:{self.port}")

        except Exception as e:
            print(f"[ERROR] Failed to connect to multicast group {self.multicast_group}:{self.port}: {e}")
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
                bytesize=8,
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
            # Clear any existing data in the buffer
            self.arduino_serial.reset_input_buffer()
            self.arduino_serial.reset_output_buffer()

            # Send handshake byte 0xA5
            self.arduino_serial.write(bytes([0xA5]))
            self.arduino_serial.flush()  # Ensure data is sent

            if self.debug:
                print("Sent handshake")

            # Wait for response 0x5A
            start_time = time.time()
            received_bytes = []

            while time.time() - start_time < 3.0:  # 3 second timeout
                if self.arduino_serial.in_waiting > 0:
                    response_byte = self.arduino_serial.read(1)
                    if len(response_byte) > 0:
                        response = response_byte[0]
                        received_bytes.append(response)

                        if self.debug:
                            print(f"Received byte: 0x{response:02X} ('{chr(response) if 32 <= response <= 126 else '?'}')")

                        if response == 0x5A:
                            print("Handshake successful!")
                            return True
                        elif len(received_bytes) > 10:  # Avoid infinite loop
                            if self.debug:
                                print(f"Too many unexpected bytes received: {[hex(b) for b in received_bytes]}")
                            break

                time.sleep(0.01)

            if received_bytes:
                if self.debug:
                    print(f"All received bytes: {[hex(b) for b in received_bytes]}")
                    print(f"As ASCII: {''.join(chr(b) if 32 <= b <= 126 else '?' for b in received_bytes)}")
                print("Handshake failed - unexpected response from Arduino")
            else:
                print("Handshake timeout - no response from Arduino")

            return False

        except Exception as e:
            print(f"Handshake error: {e}")
            return False

    def _multicast_loop(self):
        """Main loop for receiving multicast data"""
        print("[OK] Multicast receive loop started - listening for BMS data...")
        self.last_packet_time = time.time()  # Initialize timing

        while self.running:
            try:
                # Receive multicast packet
                data, addr = self.socket.recvfrom(65536)  # Max UDP packet size

                # Update packet timing
                self.last_packet_time = time.time()

                # Show first packet received message
                if not self.first_packet_received:
                    print(f"[OK] First BMS data packet received from {addr[0]}")
                    self.first_packet_received = True

                # Parse header
                header = self._parse_header(data)
                if not header:
                    continue

                if self.debug:
                    print(f"Got SHM packet: Version={header.version}, Seq={header.sequence}, Type={header.data_type}, Size={len(data)}")

                # Check sequence (detect out-of-order packets)
                if header.sequence <= self.last_sequence:
                    if self.debug:
                        print(f"Out-of-order packet: seq={header.sequence}, last={self.last_sequence}")
                    continue

                self.last_sequence = header.sequence

                # Parse data based on type
                payload = data[9:]  # Skip header
                if header.data_type == SMEMDataType.FLIGHT_DATA:
                    if self.print_data:
                        print(f"\n[Packet] FlightData: Size={len(payload)} bytes, Seq={header.sequence}")
                    self._parse_flight_data(payload)
                elif header.data_type == SMEMDataType.FLIGHT_DATA2:
                    if self.print_data:
                        print(f"\n[Packet] FlightData2: Size={len(payload)} bytes, Seq={header.sequence}")
                    self._parse_flight_data2(payload)
                elif header.data_type == SMEMDataType.OSB_DATA:
                    if self.print_data:
                        print(f"\n[Packet] OSBData: Size={len(payload)} bytes, Seq={header.sequence}")
                    self._parse_osb_data(payload)
                elif header.data_type == SMEMDataType.INTELLIVIBE_DATA:
                    if self.print_data:
                        print(f"\n[Packet] IntellivibeData: Size={len(payload)} bytes, Seq={header.sequence}")
                    self._parse_intellivibe_data(payload)

            except socket.timeout:
                # Check if we haven't received packets for too long (like ESP32)
                if self.first_packet_received and time.time() - self.last_packet_time > self.udp_timeout:
                    print(f"[WARNING] No packets received for {self.udp_timeout}s - connection may be lost")
                continue  # Normal timeout, check running flag
            except Exception as e:
                if self.running:
                    print(f"[ERROR] Multicast receive error: {e}")
                    print("[ERROR] Lost connection to multicast group")

    def _parse_header(self, data: bytes) -> Optional[MulticastHeader]:
        """Parse multicast packet header"""
        if len(data) < 9:
            return None

        # Check identifier
        if data[0:3] != b'SHM':
            return None

        version = data[3]
        if version != 1:
            if self.debug:
                print(f"Unsupported multicast version: {version}")
            return None

        sequence = struct.unpack('<I', data[4:8])[0]  # Little-endian uint32 (matches ESP32)
        data_type = data[8]

        return MulticastHeader(
            identifier=data[0:3],
            version=version,
            sequence=sequence,
            data_type=data_type
        )

    def _parse_flight_data(self, data: bytes):
        """Parse FlightData structure using complete field mapping"""
        try:
            # Validate minimum expected size - be more flexible
            if len(data) < 20:
                print(f"FlightData too small: {len(data)} bytes (minimum 20 required)")
                return

            offset = 0

            # Parse fields safely with bounds checking
            # Basic position and attitude data (12 floats = 48 bytes)
            if len(data) >= offset + 48:
                pos_att = struct.unpack('<12f', data[offset:offset+48])
                self.flight_data.update({
                    'x': pos_att[0], 'y': pos_att[1], 'z': pos_att[2],
                    'xDot': pos_att[3], 'yDot': pos_att[4], 'zDot': pos_att[5],
                    'alpha': pos_att[6], 'beta': pos_att[7], 'gamma': pos_att[8],
                    'pitch': pos_att[9], 'roll': pos_att[10], 'yaw': pos_att[11]
                })
                offset += 48

            # Engine and flight data (7 floats = 28 bytes)
            if len(data) >= offset + 28:
                engine_data = struct.unpack('<7f', data[offset:offset+28])
                self.flight_data.update({
                    'mach': engine_data[0], 'kias': engine_data[1], 'vt': engine_data[2],
                    'gs': engine_data[3], 'windOffset': engine_data[4], 'nozzlePos': engine_data[5],
                    'internalFuel': engine_data[6]
                })
                offset += 28

            # More engine data (4 floats = 16 bytes)
            if len(data) >= offset + 16:
                engine_data2 = struct.unpack('<4f', data[offset:offset+16])
                self.flight_data.update({
                    'externalFuel': engine_data2[0], 'fuelFlow': engine_data2[1],
                    'rpm': engine_data2[2], 'ftit': engine_data2[3]
                })
                offset += 16

            # Gear and systems (3 floats = 12 bytes)
            if len(data) >= offset + 12:
                gear_sys = struct.unpack('<3f', data[offset:offset+12])
                self.flight_data.update({
                    'gearPos': gear_sys[0], 'speedBrake': gear_sys[1], 'epuFuel': gear_sys[2]
                })
                offset += 12

            # Oil pressure (1 float = 4 bytes)
            if len(data) >= offset + 4:
                oil_pressure = struct.unpack('<f', data[offset:offset+4])[0]
                self.flight_data['oilPressure'] = oil_pressure
                offset += 4

            # Light bits (1 uint32 = 4 bytes)
            if len(data) >= offset + 4:
                lightbits = struct.unpack('<I', data[offset:offset+4])[0]
                self.flight_data['lightBits'] = lightbits
                offset += 4

            # Head position (3 floats = 12 bytes)
            if len(data) >= offset + 12:
                head_pos = struct.unpack('<3f', data[offset:offset+12])
                self.flight_data.update({
                    'headPitch': head_pos[0], 'headRoll': head_pos[1], 'headYaw': head_pos[2]
                })
                offset += 12

            # More light bits (2 uint32 = 8 bytes)
            if len(data) >= offset + 8:
                light_bits_23 = struct.unpack('<2I', data[offset:offset+8])
                self.flight_data.update({
                    'lightBits2': light_bits_23[0], 'lightBits3': light_bits_23[1]
                })
                offset += 8

            # Chaff/Flare (2 floats = 8 bytes)
            if len(data) >= offset + 8:
                counts = struct.unpack('<2f', data[offset:offset+8])
                self.flight_data.update({
                    'chaffCount': counts[0], 'flareCount': counts[1]
                })
                offset += 8

            # Gear positions (3 floats = 12 bytes)
            if len(data) >= offset + 12:
                gear_pos = struct.unpack('<3f', data[offset:offset+12])
                self.flight_data.update({
                    'noseGearPos': gear_pos[0], 'leftGearPos': gear_pos[1], 'rightGearPos': gear_pos[2]
                })
                offset += 12

            # ADI values (2 floats = 8 bytes)
            if len(data) >= offset + 8:
                adi_values = struct.unpack('<2f', data[offset:offset+8])
                self.flight_data.update({
                    'adiIlsHorPos': adi_values[0], 'adiIlsVerPos': adi_values[1]
                })
                offset += 8

            # HSI states (3 int32 = 12 bytes)
            if len(data) >= offset + 12:
                hsi_states = struct.unpack('<3i', data[offset:offset+12])
                self.flight_data.update({
                    'courseState': hsi_states[0], 'headingState': hsi_states[1], 'totalStates': hsi_states[2]
                })
                offset += 12

            # HSI values (9 floats = 36 bytes)
            if len(data) >= offset + 36:
                hsi_values = struct.unpack('<9f', data[offset:offset+36])
                self.flight_data.update({
                    'courseDeviation': hsi_values[0], 'desiredCourse': hsi_values[1],
                    'distanceToBeacon': hsi_values[2], 'bearingToBeacon': hsi_values[3],
                    'currentHeading': hsi_values[4], 'desiredHeading': hsi_values[5],
                    'deviationLimit': hsi_values[6], 'halfDeviationLimit': hsi_values[7],
                    'localizerCourse': hsi_values[8]
                })
                offset += 36

            # Skip airbaseX, airbaseY, totalValues (would be 3 more floats)
            if len(data) >= offset + 12:
                airbase_values = struct.unpack('<3f', data[offset:offset+12])
                self.flight_data.update({
                    'airbaseX': airbase_values[0], 'airbaseY': airbase_values[1],
                    'totalValues': airbase_values[2]
                })
                offset += 12

            # Trim values (3 floats = 12 bytes)
            trim_values = struct.unpack('<3f', data[offset:offset+12])
            self.flight_data.update({
                'trimPitch': trim_values[0], 'trimRoll': trim_values[1], 'trimYaw': trim_values[2]
            })
            offset += 12

            # HSI bits (1 uint32 = 4 bytes)
            hsi_bits = struct.unpack('<I', data[offset:offset+4])[0]
            self.flight_data['hsiBits'] = hsi_bits
            offset += 4

            # DED Lines (5 * 26 bytes = 130 bytes)
            for i in range(5):
                line_data = data[offset:offset+26]
                ded_line = line_data.decode('ascii', errors='ignore').rstrip('\x00')
                self.flight_data[f'DEDLine_{i}'] = ded_line
                offset += 26

            # DED Invert (5 * 26 bytes = 130 bytes)
            for i in range(5):
                invert_data = data[offset:offset+26]
                self.flight_data[f'DEDInvert_{i}'] = invert_data
                offset += 26

            # PFL Lines (5 * 26 bytes = 130 bytes)
            for i in range(5):
                line_data = data[offset:offset+26]
                pfl_line = line_data.decode('ascii', errors='ignore').rstrip('\x00')
                self.flight_data[f'PFLLine_{i}'] = pfl_line
                offset += 26

            # PFL Invert (5 * 26 bytes = 130 bytes)
            for i in range(5):
                invert_data = data[offset:offset+26]
                self.flight_data[f'PFLInvert_{i}'] = invert_data
                offset += 26

            # TACAN channels (2 int32 = 8 bytes)
            tacan_channels = struct.unpack('<2i', data[offset:offset+8])
            self.flight_data.update({
                'UFCTChan': tacan_channels[0], 'AUXTChan': tacan_channels[1]
            })
            offset += 8

            # RWR data (1 int32 + 40*6 values = 964 bytes)
            rwr_count = struct.unpack('<i', data[offset:offset+4])[0]
            self.flight_data['rwrObjectCount'] = rwr_count
            offset += 4

            # RWR arrays (40 elements each)
            if len(data) >= offset + (40 * 24):  # 40 * (4+4+4+4+4+4)
                rwr_symbol = list(struct.unpack('<40i', data[offset:offset+160]))
                offset += 160
                rwr_bearing = list(struct.unpack('<40f', data[offset:offset+160]))
                offset += 160
                rwr_missile_activity = list(struct.unpack('<40L', data[offset:offset+160]))
                offset += 160
                rwr_missile_launch = list(struct.unpack('<40L', data[offset:offset+160]))
                offset += 160
                rwr_selected = list(struct.unpack('<40L', data[offset:offset+160]))
                offset += 160
                rwr_lethality = list(struct.unpack('<40f', data[offset:offset+160]))
                offset += 160
                rwr_new_detection = list(struct.unpack('<40L', data[offset:offset+160]))
                offset += 160

                self.flight_data.update({
                    'rwrSymbol': rwr_symbol, 'rwrBearing': rwr_bearing,
                    'rwrMissileActivity': rwr_missile_activity, 'rwrMissileLaunch': rwr_missile_launch,
                    'rwrSelected': rwr_selected, 'rwrLethality': rwr_lethality,
                    'rwrNewDetection': rwr_new_detection
                })

            # Fuel values (3 floats = 12 bytes)
            if len(data) >= offset + 12:
                fuel_data = struct.unpack('<3f', data[offset:offset+12])
                self.flight_data.update({
                    'fwd': fuel_data[0], 'aft': fuel_data[1], 'total': fuel_data[2]
                })
                offset += 12

            # Version and final fields (5 values = 20 bytes: i + 3f + i)
            if len(data) >= offset + 20:
                final_data = struct.unpack('<i3fi', data[offset:offset+20])
                self.flight_data.update({
                    'versionNum': final_data[0],
                    'headX': final_data[1], 'headY': final_data[2], 'headZ': final_data[3],
                    'mainPower': final_data[4]
                })

            # Print comprehensive flight data if requested
            if self.print_data:
                print("\n=== FLIGHTDATA ===")
                print(f"Position: X={self.flight_data.get('x', 0):.1f}ft N, Y={self.flight_data.get('y', 0):.1f}ft E, Z={self.flight_data.get('z', 0):.1f}ft Down")
                print(f"Velocity: XDot={self.flight_data.get('xDot', 0):.1f}, YDot={self.flight_data.get('yDot', 0):.1f}, ZDot={self.flight_data.get('zDot', 0):.1f} ft/sec")
                print(f"Attitude: Pitch={self.flight_data.get('pitch', 0):.3f}, Roll={self.flight_data.get('roll', 0):.3f}, Yaw={self.flight_data.get('yaw', 0):.3f} rad")
                print(f"AOA/Beta: Alpha={self.flight_data.get('alpha', 0):.1f}°, Beta={self.flight_data.get('beta', 0):.1f}°, Gamma={self.flight_data.get('gamma', 0):.3f} rad")

                print(f"Flight Performance: Mach={self.flight_data.get('mach', 0):.2f}, KIAS={self.flight_data.get('kias', 0):.1f} kts, VT={self.flight_data.get('vt', 0):.1f} ft/s")
                print(f"G-Force: {self.flight_data.get('gs', 0):.1f}G, Wind Offset={self.flight_data.get('windOffset', 0):.3f} rad")

                print(f"Engine: RPM={self.flight_data.get('rpm', 0):.1f}%, Nozzle={self.flight_data.get('nozzlePos', 0):.1f}%, FTIT={self.flight_data.get('ftit', 0):.0f}°C")
                print(f"Fuel: Internal={self.flight_data.get('internalFuel', 0):.1f}lbs, External={self.flight_data.get('externalFuel', 0):.1f}lbs, Flow={self.flight_data.get('fuelFlow', 0):.1f}lbs/hr")
                print(f"Fuel Tanks: Fwd={self.flight_data.get('fwd', 0):.1f}, Aft={self.flight_data.get('aft', 0):.1f}, Total={self.flight_data.get('total', 0):.1f} lbs")

                print(f"Systems: Gear={self.flight_data.get('gearPos', 0):.1f}, Speed Brake={self.flight_data.get('speedBrake', 0):.1f}, EPU Fuel={self.flight_data.get('epuFuel', 0):.1f}%")
                print(f"Oil Pressure: {self.flight_data.get('oilPressure', 0):.1f}%, Main Power: {self.flight_data.get('mainPower', 0)}")

                print(f"Landing Gear: Nose={self.flight_data.get('noseGearPos', 0):.2f}, Left={self.flight_data.get('leftGearPos', 0):.2f}, Right={self.flight_data.get('rightGearPos', 0):.2f}")

                print(f"Countermeasures: Chaff={self.flight_data.get('chaffCount', 0):.0f}, Flare={self.flight_data.get('flareCount', 0):.0f}")

                print(f"Head Tracking: Pitch={self.flight_data.get('headPitch', 0):.3f}, Roll={self.flight_data.get('headRoll', 0):.3f}, Yaw={self.flight_data.get('headYaw', 0):.3f} rad")
                print(f"Head Position: X={self.flight_data.get('headX', 0):.3f}, Y={self.flight_data.get('headY', 0):.3f}, Z={self.flight_data.get('headZ', 0):.3f} ft")

                print(f"ADI: ILS Hor={self.flight_data.get('adiIlsHorPos', 0):.3f}, ILS Ver={self.flight_data.get('adiIlsVerPos', 0):.3f}")

                print(f"HSI: Course Dev={self.flight_data.get('courseDeviation', 0):.1f}, Desired Course={self.flight_data.get('desiredCourse', 0):.1f}°")
                print(f"     Distance to Beacon={self.flight_data.get('distanceToBeacon', 0):.1f}, Bearing={self.flight_data.get('bearingToBeacon', 0):.1f}°")
                print(f"     Current Heading={self.flight_data.get('currentHeading', 0):.1f}°, Desired={self.flight_data.get('desiredHeading', 0):.1f}°")

                print(f"Trim: Pitch={self.flight_data.get('trimPitch', 0):.3f}, Roll={self.flight_data.get('trimRoll', 0):.3f}, Yaw={self.flight_data.get('trimYaw', 0):.3f}")

                print(f"TACAN: UFC Chan={self.flight_data.get('UFCTChan', 0)}, AUX Chan={self.flight_data.get('AUXTChan', 0)}")

                print(f"RWR: {self.flight_data.get('rwrObjectCount', 0)} objects detected")

                # Light bits in hex for easier reading
                print(f"Light Bits: 0x{self.flight_data.get('lightBits', 0):08X}, 0x{self.flight_data.get('lightBits2', 0):08X}, 0x{self.flight_data.get('lightBits3', 0):08X}")
                print(f"HSI Bits: 0x{self.flight_data.get('hsiBits', 0):08X}")

                print(f"Version: {self.flight_data.get('versionNum', 0)}")

                # Show DED lines if they have content
                for i in range(5):
                    ded_line = self.flight_data.get(f'DEDLine_{i}', '')
                    if ded_line.strip():
                        print(f"DED Line {i}: '{ded_line}'")

                # Show PFL lines if they have content
                for i in range(5):
                    pfl_line = self.flight_data.get(f'PFLLine_{i}', '')
                    if pfl_line.strip():
                        print(f"PFL Line {i}: '{pfl_line}'")

        except Exception as e:
            import traceback
            print(f"Error parsing FlightData: {e}")
            if self.debug:
                print(f"Debug: Data length={len(data)}, Error details:")
                traceback.print_exc()

    def _parse_flight_data2(self, data: bytes):
        """Parse complete FlightData2 structure based on FlightData.h (all fields)"""
        try:
            # Validate minimum expected size
            if len(data) < 50:
                print(f"FlightData2 too small: {len(data)} bytes")
                return

            offset = 0

            # Parse complete FlightData2 structure according to FlightData.h

            # VERSION 1 - Engine data and navigation (4f + B + f + 2B = 23 bytes)
            if len(data) >= offset + 23:
                v1_data = struct.unpack('<4fBf2B', data[offset:offset+23])
                self.flight_data2.update({
                    'nozzlePos2': v1_data[0],       # Ownship engine nozzle2 percent open (0-100)
                    'rpm2': v1_data[1],             # Ownship engine rpm2 (Percent 0-103)
                    'ftit2': v1_data[2],            # Ownship Forward Turbine Inlet Temp2 (Degrees C)
                    'oilPressure2': v1_data[3],     # Ownship Oil Pressure2 (Percent 0-100)
                    'navMode': v1_data[4],          # Current mode selected for HSI/eHSI
                    'AAUZ': v1_data[5],             # Ownship barometric altitude given by AAU
                    'tacanInfo_UFC': v1_data[6],    # Tacan band/mode settings for UFC
                    'tacanInfo_AUX': v1_data[7]     # Tacan band/mode settings for AUX
                })
                offset += 23

            # VERSION 2/7 - Altimeter and power systems (i + 4I + 2i = 28 bytes)
            if len(data) >= offset + 28:
                v2_data = struct.unpack('<i4I2i', data[offset:offset+28])
                self.flight_data2.update({
                    'altCalReading': v2_data[0],    # Barometric altitude calibration
                    'altBits': v2_data[1],          # Various altimeter bits
                    'powerBits': v2_data[2],        # Ownship power bus / generator states
                    'blinkBits': v2_data[3],        # Cockpit indicator lights blink status
                    'cmdsMode': v2_data[4],         # Ownship CMDS mode state
                    'uhf_panel_preset': v2_data[5],  # BUP UHF channel preset
                    'uhf_panel_frequency': v2_data[6] # BUP UHF channel frequency
                })
                offset += 28

            # VERSION 3 - Additional systems (3f + i + h + i = 22 bytes, with padding)
            if len(data) >= offset + 22:
                v3_data = struct.unpack('<3fihi', data[offset:offset+22])
                self.flight_data2.update({
                    'cabinAlt': v3_data[0],         # Ownship cabin altitude
                    'hydPressureA': v3_data[1],     # Ownship Hydraulic Pressure A
                    'hydPressureB': v3_data[2],     # Ownship Hydraulic Pressure B
                    'currentTime': v3_data[3],      # Current time in seconds (max 60 * 60 * 24)
                    'vehicleACD': v3_data[4],       # Ownship ACD index number
                    'versionNum': v3_data[5]        # Version of FlightData2 mem area
                })
                offset += 22

            # VERSION 4 - Additional fuel flow (f = 4 bytes)
            if len(data) >= offset + 4:
                fuel_flow2 = struct.unpack('<f', data[offset:offset+4])[0]
                self.flight_data2['fuelFlow2'] = fuel_flow2  # Ownship fuel flow2 (Lbs/Hour)
                offset += 4

            # VERSION 5/8 - RWR info and flight surfaces (512 + 2f = 520 bytes)
            if len(data) >= offset + 520:
                rwr_info = data[offset:offset+512]
                self.flight_data2['rwrInfo'] = rwr_info  # New RWR Info
                offset += 512

                lef_tef = struct.unpack('<2f', data[offset:offset+8])
                self.flight_data2.update({
                    'lefPos': lef_tef[0],           # Ownship LEF position
                    'tefPos': lef_tef[1]            # Ownship TEF position
                })
                offset += 8

            # VERSION 6 - VTOL (f = 4 bytes)
            if len(data) >= offset + 4:
                vtol_pos = struct.unpack('<f', data[offset:offset+4])[0]
                self.flight_data2['vtolPos'] = vtol_pos  # Ownship VTOL exhaust angle
                offset += 4

            # VERSION 9 - Multiplayer pilot info (B + 32*12B + 32B = 417 bytes)
            if len(data) >= offset + 417:
                pilots_online = struct.unpack('<B', data[offset:offset+1])[0]
                self.flight_data2['pilotsOnline'] = pilots_online  # Number of pilots in MP session
                offset += 1

                # Parse pilot callsigns (32 callsigns * 12 chars each)
                pilot_callsigns = []
                for i in range(32):
                    if len(data) >= offset + 12:
                        callsign_data = data[offset:offset+12]
                        callsign = callsign_data.decode('ascii', errors='ignore').rstrip('\x00')
                        pilot_callsigns.append(callsign)
                        offset += 12
                self.flight_data2['pilotsCallsign'] = pilot_callsigns

                # Parse pilot statuses (32 bytes)
                if len(data) >= offset + 32:
                    pilot_statuses = list(struct.unpack('<32B', data[offset:offset+32]))
                    self.flight_data2['pilotsStatus'] = pilot_statuses
                    offset += 32

            # VERSION 10 - Taxi effects (f = 4 bytes)
            if len(data) >= offset + 4:
                bump_intensity = struct.unpack('<f', data[offset:offset+4])[0]
                self.flight_data2['bumpIntensity'] = bump_intensity  # Intensity of bump while taxiing
                offset += 4

            # VERSION 11 - GPS coordinates (2f = 8 bytes)
            if len(data) >= offset + 8:
                gps_coords = struct.unpack('<2f', data[offset:offset+8])
                self.flight_data2.update({
                    'latitude': gps_coords[0],      # Ownship latitude in degrees
                    'longitude': gps_coords[1]      # Ownship longitude in degrees
                })
                offset += 8

            # VERSION 12 - RTT info (2H + 7*4H = 60 bytes)
            if len(data) >= offset + 60:
                rtt_size = list(struct.unpack('<2H', data[offset:offset+4]))
                self.flight_data2['RTT_size'] = rtt_size  # RTT overall width and height
                offset += 4

                # RTT areas (7 areas, 4 values each)
                rtt_areas = []
                for i in range(7):
                    if len(data) >= offset + 8:
                        area = list(struct.unpack('<4H', data[offset:offset+8]))
                        rtt_areas.append(area)  # left/top/right/bottom for each area
                        offset += 8
                self.flight_data2['RTT_area'] = rtt_areas

            # VERSION 13 - IFF backup digits (4b = 4 bytes)
            if len(data) >= offset + 4:
                iff_backup = struct.unpack('<4b', data[offset:offset+4])
                self.flight_data2.update({
                    'iffBackupMode1Digit1': iff_backup[0],   # IFF panel backup Mode1 digit 1
                    'iffBackupMode1Digit2': iff_backup[1],   # IFF panel backup Mode1 digit 2
                    'iffBackupMode3ADigit1': iff_backup[2],  # IFF panel backup Mode3A digit 1
                    'iffBackupMode3ADigit2': iff_backup[3]   # IFF panel backup Mode3A digit 2
                })
                offset += 4

            # VERSION 14 - Instrument lighting (B = 1 byte)
            if len(data) >= offset + 1:
                instr_light = struct.unpack('<B', data[offset:offset+1])[0]
                self.flight_data2['instrLight'] = instr_light  # Instrument backlight brightness
                offset += 1

            # VERSION 15 - Betty, misc bits, radar alt, etc. (2I + 5f + 4I + 3I = 56 bytes)
            if len(data) >= offset + 56:
                v15_data = struct.unpack('<2I5f4I3I', data[offset:offset+56])
                self.flight_data2.update({
                    'bettyBits': v15_data[0],        # Bitching Betty VMS sounds playing
                    'miscBits': v15_data[1],         # Various flags
                    'RALT': v15_data[2],             # Radar altitude
                    'bingoFuel': v15_data[3],        # Bingo fuel level
                    'caraAlow': v15_data[4],         # Cara alow setting
                    'bullseyeX': v15_data[5],        # Bullseye X in sim coordinates
                    'bullseyeY': v15_data[6],        # Bullseye Y in sim coordinates
                    'BMSVersionMajor': v15_data[7],  # BMS version major
                    'BMSVersionMinor': v15_data[8],  # BMS version minor
                    'BMSVersionMicro': v15_data[9],  # BMS version micro
                    'BMSBuildNumber': v15_data[10],  # BMS build number
                    'StringAreaSize': v15_data[11],  # Size of StringData area
                    'StringAreaTime': v15_data[12],  # Last time StringData changed
                    'DrawingAreaSize': v15_data[13]  # Size of DrawingData area
                })
                offset += 56

            # VERSION 16 - Turn rate (f = 4 bytes)
            if len(data) >= offset + 4:
                turn_rate = struct.unpack('<f', data[offset:offset+4])[0]
                self.flight_data2['turnRate'] = turn_rate  # Actual turn rate in degrees/second
                offset += 4

            # VERSION 18 - Flood console lighting (B = 1 byte)
            if len(data) >= offset + 1:
                flood_console = struct.unpack('<B', data[offset:offset+1])[0]
                self.flight_data2['floodConsole'] = flood_console  # Floodconsole brightness
                offset += 1

            # VERSION 19 - Magnetic deviation and ECM (2f + 5I + B + 40B = 69 bytes)
            if len(data) >= offset + 69:
                mag_dev = struct.unpack('<2f', data[offset:offset+8])
                self.flight_data2.update({
                    'magDeviationSystem': mag_dev[0],  # Current mag deviation of system
                    'magDeviationReal': mag_dev[1]     # Current mag deviation real
                })
                offset += 8

                # ECM bits (5 uint32)
                ecm_bits = list(struct.unpack('<5I', data[offset:offset+20]))
                self.flight_data2['ecmBits'] = ecm_bits
                offset += 20

                # ECM operator state
                ecm_oper = struct.unpack('<B', data[offset:offset+1])[0]
                self.flight_data2['ecmOper'] = ecm_oper
                offset += 1

                # RWR jamming status (40 bytes)
                rwr_jamming = list(struct.unpack('<40B', data[offset:offset+40]))
                self.flight_data2['RWRjammingStatus'] = rwr_jamming
                offset += 40

            # VERSION 20 - Radio 2 and IFF transponder (2i + b + 4h = 17 bytes)
            if len(data) >= offset + 17:
                radio_iff = struct.unpack('<2ib4h', data[offset:offset+17])
                self.flight_data2.update({
                    'radio2_preset': radio_iff[0],              # Radio 2 channel preset
                    'radio2_frequency': radio_iff[1],           # Radio 2 channel frequency
                    'iffTransponderActiveCode1': radio_iff[2],  # IFF mode 1
                    'iffTransponderActiveCode2': radio_iff[3],  # IFF mode 2
                    'iffTransponderActiveCode3A': radio_iff[4], # IFF mode 3A
                    'iffTransponderActiveCodeC': radio_iff[5],  # IFF mode C
                    'iffTransponderActiveCode4': radio_iff[6]   # IFF mode 4
                })
                offset += 17

            # VERSION 21 - TACAN ILS (i = 4 bytes)
            if len(data) >= offset + 4:
                tacan_ils = struct.unpack('<i', data[offset:offset+4])[0]
                self.flight_data2['tacan_ils_frequency'] = tacan_ils  # TACAN ILS frequency
                offset += 4

            # VERSION 22 - RTT FPS and side slip (i + f = 8 bytes)
            if len(data) >= offset + 8:
                fps_slip = struct.unpack('<if', data[offset:offset+8])
                self.flight_data2.update({
                    'desired_RTT_FPS': fps_slip[0],  # Configured RTT export FPS value
                    'sideSlipdeg': fps_slip[1]       # ADI side slip
                })
                offset += 8

            # Print comprehensive flight data2 if requested
            if self.print_data:
                print("\n=== FLIGHTDATA2 ===")
                print(f"Engine 2: RPM2={self.flight_data2.get('rpm2', 0):.1f}%, Nozzle2={self.flight_data2.get('nozzlePos2', 0):.1f}%, FTIT2={self.flight_data2.get('ftit2', 0):.0f}°C")
                print(f"Oil Pressure 2: {self.flight_data2.get('oilPressure2', 0):.1f}%, Fuel Flow 2: {self.flight_data2.get('fuelFlow2', 0):.1f} lbs/hr")

                print(f"Altitude: AAUZ={self.flight_data2.get('AAUZ', 0):.1f}ft, Cabin Alt={self.flight_data2.get('cabinAlt', 0):.1f}ft, RALT={self.flight_data2.get('RALT', 0):.1f}ft")
                print(f"Altimeter: Cal Reading={self.flight_data2.get('altCalReading', 0)}, Bits=0x{self.flight_data2.get('altBits', 0):08X}")

                print(f"Navigation: Mode={self.flight_data2.get('navMode', 0)}, TACAN UFC=0x{self.flight_data2.get('tacanInfo_UFC', 0):02X}, AUX=0x{self.flight_data2.get('tacanInfo_AUX', 0):02X}")
                print(f"TACAN ILS Freq: {self.flight_data2.get('tacan_ils_frequency', 0)}")

                print(f"Power: Bits=0x{self.flight_data2.get('powerBits', 0):08X}, Blink Bits=0x{self.flight_data2.get('blinkBits', 0):08X}")

                print(f"CMDS Mode: {self.flight_data2.get('cmdsMode', 0)}")

                print(f"Radio: UHF Preset={self.flight_data2.get('uhf_panel_preset', 0)}, Freq={self.flight_data2.get('uhf_panel_frequency', 0)}")
                print(f"Radio 2: Preset={self.flight_data2.get('radio2_preset', 0)}, Freq={self.flight_data2.get('radio2_frequency', 0)}")

                print(f"Hydraulics: A={self.flight_data2.get('hydPressureA', 0):.1f}, B={self.flight_data2.get('hydPressureB', 0):.1f}")

                print(f"Flight Surfaces: LEF={self.flight_data2.get('lefPos', 0):.2f}, TEF={self.flight_data2.get('tefPos', 0):.2f}")
                if 'vtolPos' in self.flight_data2:
                    print(f"VTOL: Position={self.flight_data2.get('vtolPos', 0):.2f}")

                print(f"Time: Current={self.flight_data2.get('currentTime', 0)}s, Vehicle ACD={self.flight_data2.get('vehicleACD', 0)}")

                if 'latitude' in self.flight_data2 and 'longitude' in self.flight_data2:
                    print(f"GPS: Lat={self.flight_data2.get('latitude', 0):.6f}°, Lon={self.flight_data2.get('longitude', 0):.6f}°")

                if 'pilotsOnline' in self.flight_data2:
                    print(f"Multiplayer: {self.flight_data2.get('pilotsOnline', 0)} pilots online")

                if 'bumpIntensity' in self.flight_data2:
                    print(f"Taxi Bump: {self.flight_data2.get('bumpIntensity', 0):.2f}")

                if 'turnRate' in self.flight_data2:
                    print(f"Turn Rate: {self.flight_data2.get('turnRate', 0):.1f}°/s")

                if 'sideSlipdeg' in self.flight_data2:
                    print(f"Side Slip: {self.flight_data2.get('sideSlipdeg', 0):.2f}°")

                print(f"IFF Backup: Mode1={self.flight_data2.get('iffBackupMode1Digit1', 0)}{self.flight_data2.get('iffBackupMode1Digit2', 0)}, Mode3A={self.flight_data2.get('iffBackupMode3ADigit1', 0)}{self.flight_data2.get('iffBackupMode3ADigit2', 0)}")

                print(f"IFF Active: M1={self.flight_data2.get('iffTransponderActiveCode1', 0)}, M2={self.flight_data2.get('iffTransponderActiveCode2', 0)}, M3A={self.flight_data2.get('iffTransponderActiveCode3A', 0)}, MC={self.flight_data2.get('iffTransponderActiveCodeC', 0)}, M4={self.flight_data2.get('iffTransponderActiveCode4', 0)}")

                print(f"Lighting: Instr={self.flight_data2.get('instrLight', 0)}, Flood Console={self.flight_data2.get('floodConsole', 0)}")

                print(f"Fuel: Bingo={self.flight_data2.get('bingoFuel', 0):.1f} lbs, Cara Alow={self.flight_data2.get('caraAlow', 0):.1f}")

                if 'bullseyeX' in self.flight_data2 and 'bullseyeY' in self.flight_data2:
                    print(f"Bullseye: X={self.flight_data2.get('bullseyeX', 0):.1f}, Y={self.flight_data2.get('bullseyeY', 0):.1f}")

                print(f"Betty Bits: 0x{self.flight_data2.get('bettyBits', 0):08X}, Misc Bits: 0x{self.flight_data2.get('miscBits', 0):08X}")

                if 'magDeviationSystem' in self.flight_data2:
                    print(f"Magnetic Dev: System={self.flight_data2.get('magDeviationSystem', 0):.2f}°, Real={self.flight_data2.get('magDeviationReal', 0):.2f}°")

                print(f"ECM: Oper={self.flight_data2.get('ecmOper', 0)}")

                if 'BMSVersionMajor' in self.flight_data2:
                    print(f"BMS Version: {self.flight_data2.get('BMSVersionMajor', 0)}.{self.flight_data2.get('BMSVersionMinor', 0)}.{self.flight_data2.get('BMSVersionMicro', 0)} Build {self.flight_data2.get('BMSBuildNumber', 0)}")

                if 'desired_RTT_FPS' in self.flight_data2:
                    print(f"RTT: Desired FPS={self.flight_data2.get('desired_RTT_FPS', 0)}")

                print(f"Version: {self.flight_data2.get('versionNum', 0)}")

        except Exception as e:
            import traceback
            print(f"Error parsing FlightData2: {e}")
            if self.debug:
                print(f"Debug: Data length={len(data)}, Error details:")
                traceback.print_exc()

    def _parse_intellivibe_data(self, data: bytes):
        """Parse complete IntellivibeData structure based on IVibeData.h (all fields)"""
        try:
            # Validate minimum expected size - IntellivibeData should be around 56 bytes
            if len(data) < 56:
                if self.debug:
                    print(f"IntellivibeData too small: {len(data)} bytes")
                return

            offset = 0
            ivibe_data = {}

            # Parse complete IntellivibeData structure according to IVibeData.h

            # Weapon counters (6 unsigned char = 6 bytes)
            if len(data) >= offset + 6:
                counters = struct.unpack('<6B', data[offset:offset+6])
                ivibe_data.update({
                    'AAMissileFired': counters[0],    # how many AA missiles fired
                    'AGMissileFired': counters[1],    # how many maveric/rockets fired
                    'BombDropped': counters[2],       # how many bombs dropped
                    'FlareDropped': counters[3],      # how many flares dropped
                    'ChaffDropped': counters[4],      # how many chaff dropped
                    'BulletsFired': counters[5]       # how many bullets shot
                })
                offset += 6

            # Collision counter (1 int32 = 4 bytes)
            if len(data) >= offset + 4:
                collision_counter = struct.unpack('<i', data[offset:offset+4])[0]
                ivibe_data['CollisionCounter'] = collision_counter  # collisions
                offset += 4

            # Boolean flags (8 bool = 8 bytes, treated as unsigned char)
            if len(data) >= offset + 8:
                flags = struct.unpack('<8B', data[offset:offset+8])
                ivibe_data.update({
                    'IsFiringGun': bool(flags[0]),    # gun is firing
                    'IsEndFlight': bool(flags[1]),    # ending the flight from 3d
                    'IsEjecting': bool(flags[2]),     # we've ejected
                    'In3D': bool(flags[3]),           # in 3D?
                    'IsPaused': bool(flags[4]),       # sim paused?
                    'IsFrozen': bool(flags[5]),       # sim frozen?
                    'IsOverG': bool(flags[6]),        # are G limits being exceeded?
                    'IsOnGround': bool(flags[7])      # are we on the ground?
                })
                offset += 8

            # Additional boolean flags (2 bool = 2 bytes)
            if len(data) >= offset + 2:
                more_flags = struct.unpack('<2B', data[offset:offset+2])
                ivibe_data.update({
                    'IsExitGame': bool(more_flags[0]), # did we exit Falcon?
                    # Padding byte for alignment
                })
                offset += 2

            # G-force (1 float = 4 bytes)
            if len(data) >= offset + 4:
                gforce = struct.unpack('<f', data[offset:offset+4])[0]
                ivibe_data['Gforce'] = gforce  # what gforce we are feeling
                offset += 4

            # Eye position (3 floats = 12 bytes)
            if len(data) >= offset + 12:
                eye_pos = struct.unpack('<3f', data[offset:offset+12])
                ivibe_data.update({
                    'eyex': eye_pos[0],  # where the eye is in relationship to the plane
                    'eyey': eye_pos[1],  # where the eye is in relationship to the plane
                    'eyez': eye_pos[2]   # where the eye is in relationship to the plane
                })
                offset += 12

            # Damage info (1 int32 + 1 float + 1 uint32 = 12 bytes)
            if len(data) >= offset + 12:
                damage_data = struct.unpack('<ifI', data[offset:offset+12])
                ivibe_data.update({
                    'lastdamage': damage_data[0],    # 1 to 8 depending on quadrant
                    'damageforce': damage_data[1],   # how big the hit was
                    'whendamage': damage_data[2]     # when the hit occurred, game time in ms
                })
                offset += 12

            # Store the parsed data
            self.intellivibe_data = ivibe_data

            # Print comprehensive IntellivibeData if requested
            if self.print_data:
                print("\n=== INTELLIVIBEDATA ===")
                print(f"Weapons Fired: AA Missiles={ivibe_data.get('AAMissileFired', 0)}, AG Missiles={ivibe_data.get('AGMissileFired', 0)}, Bombs={ivibe_data.get('BombDropped', 0)}, Bullets={ivibe_data.get('BulletsFired', 0)}")
                print(f"Countermeasures: Flare={ivibe_data.get('FlareDropped', 0)}, Chaff={ivibe_data.get('ChaffDropped', 0)}")
                print(f"Collisions: {ivibe_data.get('CollisionCounter', 0)}")

                print(f"Gun Status: Firing={ivibe_data.get('IsFiringGun', False)}")
                print(f"Flight Status: In3D={ivibe_data.get('In3D', False)}, EndFlight={ivibe_data.get('IsEndFlight', False)}, Ejecting={ivibe_data.get('IsEjecting', False)}, ExitGame={ivibe_data.get('IsExitGame', False)}")
                print(f"Sim Status: Paused={ivibe_data.get('IsPaused', False)}, Frozen={ivibe_data.get('IsFrozen', False)}")
                print(f"Aircraft Status: OnGround={ivibe_data.get('IsOnGround', False)}, OverG={ivibe_data.get('IsOverG', False)}")

                print(f"G-Force: {ivibe_data.get('Gforce', 0):.2f}G")
                print(f"Eye Position: X={ivibe_data.get('eyex', 0):.3f}, Y={ivibe_data.get('eyey', 0):.3f}, Z={ivibe_data.get('eyez', 0):.3f}")

                if ivibe_data.get('lastdamage', 0) > 0:
                    print(f"Last Damage: Quadrant={ivibe_data.get('lastdamage', 0)}, Force={ivibe_data.get('damageforce', 0):.2f}, When={ivibe_data.get('whendamage', 0)}ms")

        except Exception as e:
            import traceback
            print(f"Error parsing IntellivibeData: {e}")
            if self.debug:
                print(f"Debug: Data length={len(data)}, Error details:")
                traceback.print_exc()

    def _parse_osb_data(self, data: bytes):
        """Parse complete OSBData structure based on FlightData.h (all fields)"""
        try:
            # OSBData structure: leftMFD[20] + rightMFD[20]
            # Each OsbLabel: line1[8] + line2[8] + inverted(bool=1) = 17 bytes per label
            # Total: 40 labels * 17 bytes = 680 bytes
            expected_size = 40 * 17  # 680 bytes

            if len(data) < expected_size:
                if self.debug:
                    print(f"OSBData too small: {len(data)} bytes, expected {expected_size}")
                return

            offset = 0
            osb_data = {}

            # Parse left MFD OSB labels (20 labels)
            left_mfd = []
            for i in range(20):
                if len(data) >= offset + 17:
                    # line1[8] + line2[8] + inverted(1 byte) = 17 bytes
                    label_data = struct.unpack('<8s8s?', data[offset:offset+17])

                    # Decode strings and remove null terminators
                    line1 = label_data[0].decode('ascii', errors='ignore').rstrip('\x00')
                    line2 = label_data[1].decode('ascii', errors='ignore').rstrip('\x00')
                    inverted = label_data[2]

                    osb_label = {
                        'line1': line1,
                        'line2': line2,
                        'inverted': inverted
                    }
                    left_mfd.append(osb_label)
                    offset += 17

            osb_data['leftMFD'] = left_mfd

            # Parse right MFD OSB labels (20 labels)
            right_mfd = []
            for i in range(20):
                if len(data) >= offset + 17:
                    # line1[8] + line2[8] + inverted(1 byte) = 17 bytes
                    label_data = struct.unpack('<8s8s?', data[offset:offset+17])

                    # Decode strings and remove null terminators
                    line1 = label_data[0].decode('ascii', errors='ignore').rstrip('\x00')
                    line2 = label_data[1].decode('ascii', errors='ignore').rstrip('\x00')
                    inverted = label_data[2]

                    osb_label = {
                        'line1': line1,
                        'line2': line2,
                        'inverted': inverted
                    }
                    right_mfd.append(osb_label)
                    offset += 17

            osb_data['rightMFD'] = right_mfd

            # Store the parsed data
            self.osb_data = osb_data

            # Print comprehensive OSBData if requested
            if self.print_data:
                print("\n=== OSBDATA ===")

                # Count non-empty OSB labels
                left_active = sum(1 for label in left_mfd if label['line1'] or label['line2'])
                right_active = sum(1 for label in right_mfd if label['line1'] or label['line2'])
                print(f"Active Labels: Left MFD {left_active}/20, Right MFD {right_active}/20")

                # Show Left MFD OSB labels that have content
                if left_active > 0:
                    print("Left MFD OSB Labels:")
                    for i, label in enumerate(left_mfd):
                        if label['line1'] or label['line2']:
                            inv_str = " [INV]" if label['inverted'] else ""
                            print(f"  OSB{i+1:2d}: '{label['line1']}' / '{label['line2']}'{inv_str}")

                # Show Right MFD OSB labels that have content
                if right_active > 0:
                    print("Right MFD OSB Labels:")
                    for i, label in enumerate(right_mfd):
                        if label['line1'] or label['line2']:
                            inv_str = " [INV]" if label['inverted'] else ""
                            print(f"  OSB{i+1:2d}: '{label['line1']}' / '{label['line2']}'{inv_str}")

        except Exception as e:
            import traceback
            print(f"Error parsing OSBData: {e}")
            if self.debug:
                print(f"Debug: Data length={len(data)}, Error details:")
                traceback.print_exc()

    def _arduino_loop(self):
        """Main loop for Arduino communication"""
        print("Arduino communication loop started")

        # Add flood protection
        last_command = None
        flood_count = 0
        max_flood_count = 50  # Allow up to 50 consecutive identical commands

        while self.running and self.arduino_connected:
            try:
                if self.arduino_serial.in_waiting > 0:
                    # Read command from Arduino
                    command = self.arduino_serial.read(1)[0]

                    # Flood detection and recovery
                    if command == last_command:
                        flood_count += 1
                        if flood_count > max_flood_count:
                            print(f"WARNING: Flood detected - command 0x{command:02X} repeated {flood_count} times")
                            print("Attempting Arduino reset sequence...")

                            # Clear serial buffers
                            self.arduino_serial.reset_input_buffer()
                            self.arduino_serial.reset_output_buffer()

                            # Send reset/handshake sequence
                            try:
                                # Send handshake to try to reset Arduino state
                                self._send_arduino_response(0x5A, bytes([0x5A]))
                                time.sleep(0.1)
                                self._send_arduino_response(0xA5, bytes([0x5A]))
                                time.sleep(0.1)
                            except:
                                pass

                            flood_count = 0
                            print("Reset sequence sent, continuing...")
                            continue
                    else:
                        flood_count = 0

                    last_command = command

                    if self.debug:
                        print(f"Received Arduino command: 0x{command:02X}")

                    # Process command and send response
                    self._process_arduino_command(command)

                time.sleep(0.01)  # Small delay to prevent busy waiting

            except Exception as e:
                print(f"Arduino communication error: {e}")
                self.arduino_connected = False
                break

    def _process_arduino_command(self, command: int):
        """Process command from Arduino and send response (matches FalconBMSArduinoConnector.cpp exactly)

        Supported Arduino Commands (matching FalconBMSArduinoConnector.cpp):
        0x01: lightBits (uint32)           0x1A: ftit2 (float)              0x28: uhfPreset (int32)
        0x02: lightBits2 (uint32)          0x20: cabinAlt (float)           0x29: uhfFreq (long)
        0x03: lightBits3 (uint32)          0x21: kias (float)               0x30: speedBrake (float)
        0x04: blinkBits (uint32)           0x22: internalFuel (float)       0x31: IFFMode1Digit1 (byte)
        0x05: DED Lines (120 bytes)        0x23: externalFuel (float)       0x32: IFFMode1Digit2 (byte)
        0x06: fuelFlow (float)             0x24: epuFuel (float)            0x33: IFFMode3Digit1 (byte)
        0x07: instrLight (byte)            0x25: hydPressureA (float)       0x34: IFFMode3Digit2 (byte)
        0x08: PFL Lines (120 bytes)        0x26: hydPressureB (float)       0x35: fwd fuel (float)
        0x09: chaffCount (float)           0x27: cmdsMode (int32)           0x36: aft fuel (float)
        0x10: flareCount (float)           0x14: oilPressure (float)        0x37: total fuel (float)
        0x11: floodConsole (byte)          0x15: oilPressure2 (float)       0x38: desiredCourse (float)
        0x12: rpm (float)                  0x16: nozzlePos (float)          0x39: courseDeviation (float)
        0x13: ecmBits (5 uint32)           0x17: nozzlePos2 (float)         0x40: distanceToBeacon (float)
        0x18: ftit (float)                 0x19: ftit2 (float)              0x41: bearingToBeacon (float)

        Special Commands:
        0x0F: Ping response (0xAB)
        0x5A: Handshake response (0x5A)
        0x99: Packet Failed (no response)
        0xA5: Handshake response (0x5A)
        """
        try:
            response_data = bytes([0x00])  # Default response

            # Check if any data is available (flight_data is most common, so check it first)
            if not self.flight_data and not self.flight_data2 and not self.intellivibe_data and not self.osb_data:
                self._send_arduino_response(command, bytes([0x00]))
                return

            if command == 0x01:  # lightBits
                response_data = struct.pack('<I', self.flight_data.get('lightBits', 0))

            elif command == 0x02:  # lightBits2
                response_data = struct.pack('<I', self.flight_data.get('lightBits2', 0))

            elif command == 0x03:  # lightBits3
                response_data = struct.pack('<I', self.flight_data.get('lightBits3', 0))

            elif command == 0x04:  # blinkBits (from FlightData2)
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

            elif command == 0x07:  # instrLight (from FlightData2) - Arduino expects byte
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

            elif command == 0x11:  # floodConsole - Arduino expects byte (hardcoded 0x00 in C++)
                response_data = bytes([0x00])  # C# sends hardcoded 0x00

            elif command == 0x12:  # rpm
                response_data = struct.pack('<f', self.flight_data.get('rpm', 0.0))

            elif command == 0x13:  # ecmBits (array of 5 uint32, not 4)
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

            elif command == 0x28:  # BupUhfPreset
                response_data = struct.pack('<i', self.flight_data2.get('uhf_panel_preset', 0))

            elif command == 0x29:  # BupUhfFreq - Arduino expects long
                response_data = struct.pack('<l', self.flight_data2.get('uhf_panel_frequency', 0))

            elif command == 0x30:  # speedBrake
                response_data = struct.pack('<f', self.flight_data.get('speedBrake', 0.0))

            elif command == 0x31:  # iffBackupMode1Digit1 - Arduino expects byte
                response_data = struct.pack('<b', self.flight_data2.get('iffBackupMode1Digit1', 0))

            elif command == 0x32:  # iffBackupMode1Digit2 - Arduino expects byte
                response_data = struct.pack('<b', self.flight_data2.get('iffBackupMode1Digit2', 0))

            elif command == 0x33:  # iffBackupMode3ADigit1 - Arduino expects byte
                response_data = struct.pack('<b', self.flight_data2.get('iffBackupMode3ADigit1', 0))

            elif command == 0x34:  # iffBackupMode3ADigit2 - Arduino expects byte
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

            elif command == 0xA5:  # Handshake byte (from Arduino C++ code)
                response_data = bytes([0x5A])

            else:
                # Only print unknown command occasionally to avoid spam
                if not hasattr(self, '_last_unknown_command') or self._last_unknown_command != command:
                    print(f"Unknown command: 0x{command:02X}")
                    self._last_unknown_command = command
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
            if self.debug:
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
    parser.add_argument('--debug', action='store_true',
                       help='Enable debug output for Arduino communication')
    parser.add_argument('--print-data', action='store_true',
                       help='Print received multicast data values')

    args = parser.parse_args()

    if args.list_ports:
        list_serial_ports()
        return

    # Create and start client
    client = FalconBMSMulticastClient(
        multicast_group=args.multicast_group,
        port=args.port,
        arduino_port=args.arduino_port,
        debug=args.debug,
        print_data=args.print_data
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