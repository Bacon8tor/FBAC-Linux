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
                print(f"Unknown command: 0x{command:02X}")
                response_data = bytes([0x00])

            # Send response using same protocol as ArduinoConnector.cs
            self._send_arduino_response(command, response_data)

        except Exception as e:
            print(f"Error processing Arduino command 0x{command:02X}: {e}")