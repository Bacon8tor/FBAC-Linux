# Arduino Compatibility Issues Found

## Issues in FalconBMSArduinoConnector.cpp

### 1. ECM Array Size Mismatch (Line 274)
**Problem**: The ECM handling loop only processes 4 uint32 values, but the Python script sends 5 values to match BMS specifications.

**Current Code**:
```cpp
case 0x13:
  for (uint8_t i = 0; i < 4; i++) {  // Should be i < 5
    ecm[i] = ...
  }
```

**Fix Required**:
- Change loop to `for (uint8_t i = 0; i < 5; i++)`
- Update header file `FalconBMSArduinoConnector.h` line 94: `uint32_t ecm[5];` (currently `ecm[4]`)

### 2. Incorrect memccpy Usage (Lines 351-369)
**Problem**: Cases 0x35-0x41 use `memccpy` instead of `memcpy` for simple data copying.

**Current Code**:
```cpp
case 0x35:
  memccpy(&fwd,data,sizeof(float),sizeof(float));  // Wrong function
case 0x36:
  memccpy(&aft,data,sizeof(float),sizeof(float));  // Wrong function
// ... etc for cases 0x37-0x41
```

**Fix Required**: Replace all `memccpy` with `memcpy`:
```cpp
case 0x35:
  memcpy(&fwd,data,sizeof(float));
case 0x36:
  memcpy(&aft,data,sizeof(float));
case 0x37:
  memcpy(&totalFuel,data,sizeof(float));
case 0x38:
  memcpy(&desiredCourse,data,sizeof(float));
case 0x39:
  memcpy(&courseDeviation,data,sizeof(float));
case 0x40:
  memcpy(&distanceToBearing,data,sizeof(float));
case 0x41:
  memcpy(&bearingToBearing,data,sizeof(float));
```

## Python Script Status
✅ **FIXED**: Python script now matches Arduino supported commands exactly (0x01-0x41 only)
✅ **FIXED**: Python script now sends correct data types (byte vs uint32, long vs int32)
✅ **FIXED**: Python script ECM command now sends 5 uint32 values
✅ **FIXED**: Python script instrLight now sends byte instead of uint32
✅ **FIXED**: Python script uhfFreq now sends long instead of int32

## Summary
The Python script has been corrected to match the Arduino expectations, but the Arduino C++ code has two bugs that prevent proper operation:
1. ECM array too small (needs to be size 5, not 4)
2. Wrong memory copy function used in 7 cases (0x35-0x41)

These Arduino fixes are required for full compatibility.