# Chinese Slave Master Fix - Display Value Correction

## Problem Statement
The master controller was not properly displaying values from Chinese slave flow meters:
- Slave value 17991.628 was showing as 17990 instead of 17991
- Need to display integer part only (no decimals)
- Must keep full decimal values in backend for future cloud transmission

## Solution
Chinese flow meters send values scaled up by a factor (multiplied by a power of 10) over Modbus RTU. The master must divide by this scale factor to display the integer part.

### Changes Made
1. **Added FLOW_SCALE_FACTOR constant** (line 54)
   - Set to 1000 for 3 decimal places (e.g., 17991.628 → 17991628)
   - Can be adjusted to 10, 100, or other values if needed

2. **Added raw value storage** (lines 93-94)
   - `total_flow_raw`: Full scaled value from slave (for cloud transmission)
   - `flow_rate_raw`: Full scaled value from slave (for cloud transmission)

3. **Modified value parsing** (lines 462-470)
   - Store raw scaled values
   - Calculate display values by dividing by FLOW_SCALE_FACTOR
   - Display values show integer part only (decimals truncated)

4. **Updated initialization** (lines 550-551)
   - Initialize new raw value fields

## Configuration

### Adjusting Scale Factor
If the display still shows incorrect values, you may need to adjust FLOW_SCALE_FACTOR:

```c
#define FLOW_SCALE_FACTOR   10UL    // For 1 decimal place (17991.6 → 179916)
#define FLOW_SCALE_FACTOR   100UL   // For 2 decimal places (17991.63 → 1799163)
#define FLOW_SCALE_FACTOR   1000UL  // For 3 decimal places (17991.628 → 17991628)
```

To determine the correct scale factor:
1. Note the raw value received from the slave (use a debugger or add logging)
2. Divide by the expected display value
3. The result is your scale factor

**Example:**
- Display should show: 17991
- Raw value received: 17991628
- Scale factor: 17991628 / 17991 = 1000 ✓

## Display Limitations
- Total flow display: 5 digits (00000-99999)
- Flow rate display: 5 digits (00000-99999)
- Values > 99999 show rightmost 5 digits only

For example:
- 139519 displays as 39519 (rightmost 5 digits)
- This is a hardware limitation of the 7-segment display

## Testing
Without physical hardware, the logic has been verified with test cases:
- 17991628 (17991.628 × 1000) → displays as 17991 ✓
- 139519000 (139519.000 × 1000) → displays as 139519 (shown as 39519) ✓
- Zero and edge cases handled correctly ✓

## Future Use
The raw values (`total_flow_raw` and `flow_rate_raw`) are preserved for:
- Cloud transmission with full precision
- Data logging with decimal places
- Any analysis requiring the complete value

Simply use the `_raw` fields instead of the regular fields when transmitting to cloud.
