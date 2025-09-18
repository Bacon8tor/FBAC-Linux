# Unmapped BMS Data Fields for Arduino Implementation

This document lists all available BMS telemetry data that is NOT currently mapped to Arduino commands (0x01-0x41). You can use this to implement additional Arduino commands for data you need.

## Available Command Range
- **Available for new commands**: 0x42-0xFE (189 available command slots)
- **Reserved commands**: 0x0F (ping), 0x5A/0xA5 (handshake), 0x99 (packet failed)

---

## FlightData Fields (Currently Available, Not Mapped)

### Flight Dynamics & Control
```cpp
// Suggested Arduino Commands: 0x42-0x5F
float xDot;                    // 0x42 - X velocity (ft/s)
float yDot;                    // 0x43 - Y velocity (ft/s)
float zDot;                    // 0x44 - Z velocity (ft/s)
float xAccel;                  // 0x45 - X acceleration (ft/s²)
float yAccel;                  // 0x46 - Y acceleration (ft/s²)
float zAccel;                  // 0x47 - Z acceleration (ft/s²)
float alpha;                   // 0x48 - Angle of attack (degrees)
float beta;                    // 0x49 - Sideslip angle (degrees)
float gamma;                   // 0x4A - Flight path angle (degrees)
float mach;                    // 0x4B - Mach number
float vt;                      // 0x4C - True airspeed (ft/s)
float gs;                      // 0x4D - Ground speed (ft/s)
float windOffset;              // 0x4E - Wind offset (degrees)
float noseGearPos;             // 0x4F - Nose gear position (0-1)
float leftGearPos;             // 0x50 - Left gear position (0-1)
float rightGearPos;            // 0x51 - Right gear position (0-1)
float gearPos;                 // 0x52 - Main gear position (0-1)
```

### Engine & Propulsion
```cpp
// Suggested Arduino Commands: 0x60-0x6F
float rpm2;                    // 0x60 - Engine 2 RPM (%)
float ftit;                    // 0x61 - Fan Turbine Inlet Temp (°C)
float oilPressure;             // 0x62 - Oil pressure (PSI)
float fuelFlow2;               // 0x63 - Engine 2 fuel flow (lbs/hr)
float egt;                     // 0x64 - Exhaust gas temperature (°C)
float egt2;                    // 0x65 - Engine 2 EGT (°C)
float nozzlePos;               // 0x66 - Nozzle position (0-1)
```

### Flight Controls & Trim
```cpp
// Suggested Arduino Commands: 0x70-0x7F
float trimPitch;               // 0x70 - Pitch trim position
float trimRoll;                // 0x71 - Roll trim position
float trimYaw;                 // 0x72 - Yaw trim position
float flcsBit[32];            // 0x73-0x92 - FLCS BIT status (32 floats)
```

### Navigation & Position
```cpp
// Suggested Arduino Commands: 0x93-0x9F
float currentHeading;          // 0x93 - Current heading (degrees)
float desiredHeading;          // 0x94 - Desired heading (degrees)
float magVar;                  // 0x95 - Magnetic variation (degrees)
float iffTransponderActiveCode[5]; // 0x96-0x9A - IFF codes (5 floats)
```

### Misc Systems
```cpp
// Suggested Arduino Commands: 0xA0-0xAF
int32_t versionNum;            // 0xA0 - Data version number
int32_t mainPower;             // 0xA1 - Main power status
uint32_t hsiBits;              // 0xA2 - HSI status bits
uint32_t TWA[17];              // 0xA3-0xB3 - Threat Warning Array (17 uint32)
```

---

## FlightData2 Fields (Currently Available, Not Mapped)

### Engine Systems (Extended)
```cpp
// Suggested Arduino Commands: 0xB4-0xBF
float AAUZ;                    // 0xB4 - Barometric altitude (ft)
float RALT;                    // 0xB5 - Radar altitude (ft)
float bingoFuel;               // 0xB6 - Bingo fuel quantity (lbs)
float caraAlow;                // 0xB7 - CARA low altitude (ft)
float bullseyeX;               // 0xB8 - Bullseye X coordinate
float bullseyeY;               // 0xB9 - Bullseye Y coordinate
float BMSVersionMajor;         // 0xBA - BMS major version
float BMSVersionMinor;         // 0xBB - BMS minor version
float BMSVersionMicro;         // 0xBC - BMS micro version
float BMSBuildNumber;          // 0xBD - BMS build number
float StringAreaSize;          // 0xBE - String area size
float StringAreaTime;          // 0xBF - String area time
```

### Flight Performance
```cpp
// Suggested Arduino Commands: 0xC0-0xCF
float turnRate;                // 0xC0 - Turn rate (deg/s)
float lefPos;                  // 0xC1 - Leading edge flaps position
float tefPos;                  // 0xC2 - Trailing edge flaps position
float vtolPos;                 // 0xC3 - VTOL position
float pilotsOnline;            // 0xC4 - Number of pilots online
float pilotsCallsign[12];      // 0xC5-0xD0 - Pilot callsigns (12 floats)
```

### Navigation Extended
```cpp
// Suggested Arduino Commands: 0xD1-0xDF
float latitude;                // 0xD1 - Current latitude (degrees)
float longitude;               // 0xD2 - Current longitude (degrees)
float magDeviationSystem;      // 0xD3 - Magnetic deviation system
float magDeviationReal;        // 0xD4 - Magnetic deviation real
float bumpIntensity;           // 0xD5 - Turbulence intensity
float windU;                   // 0xD6 - Wind U component
float windV;                   // 0xD7 - Wind V component
float windW;                   // 0xD8 - Wind W component
```

### Systems Status
```cpp
// Suggested Arduino Commands: 0xE0-0xEF
uint32_t altBits;              // 0xE0 - Altitude warning bits
uint32_t powerBits;            // 0xE1 - Power system bits
uint32_t miscBits;             // 0xE2 - Miscellaneous bits
uint32_t bettyBits;            // 0xE3 - Betty voice warning bits
byte navMode;                  // 0xE4 - Navigation mode
byte ecmOper;                  // 0xE5 - ECM operation mode
int32_t tacan_ils_frequency;   // 0xE6 - TACAN/ILS frequency
int32_t desired_RTT_FPS;       // 0xE7 - Desired frame rate
float sideSlipdeg;             // 0xE8 - Sideslip in degrees
int32_t currentTime;           // 0xE9 - Current mission time
int16_t vehicleACD;            // 0xEA - Vehicle ACD
```

---

## IntellivibeData Fields (Currently Available, Not Mapped)

```cpp
// Suggested Arduino Commands: 0xF0-0xFE
unsigned char AAMissileFired;  // 0xF0 - AA missiles fired count
unsigned char AGMissileFired;  // 0xF1 - AG missiles fired count
unsigned char BombDropped;     // 0xF2 - Bombs dropped count
unsigned char FlareDropped;    // 0xF3 - Flares dropped count
unsigned char ChaffDropped;    // 0xF4 - Chaff dropped count
unsigned char BulletsFired;    // 0xF5 - Bullets fired count
int CollisionCounter;          // 0xF6 - Collision counter
bool IsFiringGun;              // 0xF7 - Gun firing status
bool IsEndFlight;              // 0xF8 - Flight ending status
bool IsEjecting;               // 0xF9 - Ejection status
bool In3D;                     // 0xFA - In 3D mode
bool IsPaused;                 // 0xFB - Simulation paused
bool IsFrozen;                 // 0xFC - Simulation frozen
bool IsOverG;                  // 0xFD - Over G-limit
bool IsOnGround;               // 0xFE - On ground status
bool IsExitGame;               // (No command - use existing data)
float Gforce;                  // (No command - use existing data)
float eyex, eyey, eyez;        // (No command - use existing data)
int lastdamage;                // (No command - use existing data)
float damageforce;             // (No command - use existing data)
unsigned int whendamage;       // (No command - use existing data)
```

---

## OSBData Fields (Currently Available, Not Mapped)

All OSB (Option Select Button) data is available through the existing parsers but not mapped to Arduino commands.

```cpp
// Suggested Arduino Commands: 0x100+ (if Arduino supports extended commands)
// Left MFD OSB Labels (20 buttons)
char leftMFD[20].line1[8];     // Left MFD OSB line 1 text
char leftMFD[20].line2[8];     // Left MFD OSB line 2 text
bool leftMFD[20].inverted;     // Left MFD OSB inverted status

// Right MFD OSB Labels (20 buttons)
char rightMFD[20].line1[8];    // Right MFD OSB line 1 text
char rightMFD[20].line2[8];    // Right MFD OSB line 2 text
bool rightMFD[20].inverted;    // Right MFD OSB inverted status
```

---

## Implementation Instructions

### 1. Choose Command Numbers
Pick unused command numbers (0x42-0xFE) for the data you need.

### 2. Add to Arduino handlePacket()
```cpp
case 0x42:  // Example: xDot
  memcpy(&xDot, data, sizeof(float));
  break;
```

### 3. Add to Python _process_arduino_command()
```python
elif command == 0x42:  # xDot
    response_data = struct.pack('<f', self.flight_data.get('xDot', 0.0))
```

### 4. Update Arduino Header
Add variables to your Arduino class:
```cpp
float xDot;  // Add to FalconBMSArduinoConnector.h
```

### 5. Add Getter Functions (Optional)
```cpp
float getXDot() { return xDot; }
```

## Data Type Reference
- `float` → `struct.pack('<f', value)` → `memcpy(&var, data, sizeof(float))`
- `int32_t` → `struct.pack('<i', value)` → `memcpy(&var, data, sizeof(int))`
- `uint32_t` → `struct.pack('<I', value)` → `memcpy(&var, data, sizeof(uint32_t))`
- `byte/uint8_t` → `struct.pack('<B', value)` → `memcpy(&var, data, sizeof(byte))`
- `bool` → `struct.pack('<B', int(value))` → `memcpy(&var, data, sizeof(byte))`

This gives you 189 additional data fields you can map to Arduino commands as needed for your specific project requirements.