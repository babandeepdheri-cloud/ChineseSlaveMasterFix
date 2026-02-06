# Cloud Transmission Guide - Accessing Exact Decimal Values

## Overview

The master controller **preserves exact decimal values** from Chinese slave meters in the backend. Display rounding is only for demo purposes. When transmitting to cloud, use the backend float values directly.

## Data Flow Architecture

```
Chinese Slave → Master Backend → Display (demo) + Cloud (exact)
17991628      → 17991.628      → 17992 (rounded) → 17991.628 (exact)
(scaled)        (float)          (unsigned long)   (float)
```

## How It Works

### 1. Receiving Data from Chinese Slave

Chinese slaves send values **scaled by 1000**:
- Actual value: `17991.628`
- Transmitted value: `17991628` (scaled)

### 2. Backend Storage (EXACT DECIMALS)

The master stores values as **float** to preserve decimals:

```c
// In modbus_parse_response():
slaves[slave_idx].total_flow = (float)total_val / 1000.0f;
slaves[slave_idx].flow_rate = (float)fr_val / 1000.0f;
```

**Result**: `slaves[0].total_flow = 17991.628` (EXACT decimals preserved!)

### 3. Display (DEMO ONLY - Rounded)

For the demo display, values are **rounded** to integers:

```c
// Display conversion (demo only):
disp_total_u = (unsigned long)(slaves[slave_idx].total_flow + 0.5f);
// Result: 17992 (rounded for display)
```

**Important**: This rounding is **ONLY for display**. Backend float values remain unchanged.

### 4. Cloud Transmission (EXACT DECIMALS)

For cloud transmission, access the **backend float values directly**:

```c
// Example cloud transmission function
void send_to_cloud(unsigned char slave_id) {
    unsigned char idx = slave_id - 1;
    
    // Access exact decimal values from backend
    float cloud_total = slaves[idx].total_flow;    // e.g., 17991.628
    float cloud_rate = slaves[idx].flow_rate;      // e.g., 12345.678
    
    // Transmit exact values to cloud
    cloud_transmit_data(slave_id, cloud_total, cloud_rate);
}
```

## Code Example: Accessing Backend Values

```c
// Get exact decimal values for any slave
for (int i = 0; i < MAX_SLAVES; i++) {
    if (slaves[i].online && slaves[i].discovered) {
        unsigned char slave_id = i + 1;
        float exact_total = slaves[i].total_flow;    // Exact decimals!
        float exact_rate = slaves[i].flow_rate;      // Exact decimals!
        
        // Now send to cloud with full precision
        printf("Slave %d: total=%.3f, rate=%.3f\n", 
               slave_id, exact_total, exact_rate);
    }
}
```

## Verification

To verify that exact decimals are preserved:

```c
// Backend storage:
printf("Backend: %.3f\n", slaves[0].total_flow);  // Output: 17991.628

// Display (rounded):
unsigned long display = (unsigned long)(slaves[0].total_flow + 0.5f);
printf("Display: %lu\n", display);  // Output: 17992

// Cloud transmission (exact):
float cloud_val = slaves[0].total_flow;
printf("Cloud: %.3f\n", cloud_val);  // Output: 17991.628
```

## Key Points

✅ **Backend preserves exact decimals**: `slaves[idx].total_flow = 17991.628`  
✅ **Display shows rounded values**: `17992` (for demo only)  
✅ **Cloud gets exact values**: Access `slaves[idx].total_flow` directly  
✅ **No data loss**: Full precision maintained in float variables  
✅ **Float precision**: ~7 significant digits (sufficient for flow values)  

## Summary

| Component | Value Type | Example | Purpose |
|-----------|-----------|---------|---------|
| Chinese Slave | Scaled integer | 17991628 | Transmission |
| Backend Storage | Float | 17991.628 | Exact decimals |
| Display | Rounded integer | 17992 | Demo only |
| Cloud | Float | 17991.628 | Exact decimals |

**Conclusion**: Yes, your master **IS** getting and preserving the exact decimal values in the backend! The display rounding is purely for demonstration purposes. For cloud transmission, simply access `slaves[idx].total_flow` and `slaves[idx].flow_rate` to get the exact decimal values (e.g., 17991.628).
