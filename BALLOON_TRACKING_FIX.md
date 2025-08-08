# Fix for Drone Hover Issue in Real-World Balloon Tracking

## Problem Description
When using a d435i camera in real-world environments, the drone hovers motionlessly during the balloon tracking phase instead of pursuing and hitting balloons. This issue does not occur in simulation.

## Root Cause
The detection script was hardcoded for simulation parameters. Real-world d435i cameras have different color channel characteristics than simulated cameras, requiring different red pixel detection thresholds.

## Solution

### 1. Use Real-World Launch Configuration
Instead of the original `func_hitballon.sh`, use the new real-world specific script:
```bash
./sh/func_hitballon_realworld.sh
```

### 2. Configure Detection Parameters
The new system supports configurable parameters for real-world use:

- **Red threshold**: Default 120 for real-world (vs 60 for simulation)
- **Camera topic**: Configurable camera input topic
- **Environment mode**: Automatically detects real vs simulation

### 3. Fine-tune Red Detection Threshold (if needed)
If the default threshold (120) doesn't work well with your specific lighting conditions:

1. Launch the tuning tool:
```bash
roslaunch object_det tune_red_threshold.launch
```

2. Follow the on-screen instructions:
   - Click on red balloon areas to test detection
   - Use +/- keys to adjust threshold
   - Press 's' to save the recommended threshold

3. Update your launch file with the recommended threshold:
```xml
<param name="red_threshold" value="YOUR_OPTIMAL_VALUE"/>
```

### 4. Launch Files

#### For Real-World Use:
- `object_det/launch/det_realworld.launch` - Real-world detection
- `mission_pkg/launch/single_hitballoon_realworld.launch` - Real-world balloon hitting

#### For Simulation:
- `object_det/launch/det_simulation.launch` - Simulation detection
- `mission_pkg/launch/single_hitballoon.launch` - Original simulation configuration

### 5. Key Changes Made

1. **Detection Script (`det.py`)**:
   - Added configurable environment mode (simulation vs real-world)
   - Separate red thresholds for different environments
   - Adaptive threshold fallback for difficult lighting
   - Enhanced logging for debugging

2. **Launch Files**:
   - Real-world specific configurations
   - Proper parameter passing
   - d435i camera optimized settings

3. **Shell Scripts**:
   - Real-world specific startup script
   - Proper launch file selection

### 6. Debugging Tips

If the drone still hovers:

1. Check detection logs:
```bash
rostopic echo /rosout | grep "red pixels"
```

2. Monitor object detection:
```bash
rostopic echo /objects
```

3. Use the tuning tool to find optimal threshold
4. Check camera topics are publishing:
```bash
rostopic list | grep camera
```

### 7. Parameters for d435i Camera

The real-world configuration includes optimized parameters for d435i:
- Higher red detection threshold (120 vs 60)
- Proper camera intrinsics
- Correct depth topic mapping
- Adaptive threshold fallback

## Verification

After applying this fix:
1. The drone should detect balloons in real-world conditions
2. Tracking behavior should resume normally
3. The HitDirect action should receive proper detection data
4. Logs should show successful red pixel detection

## Troubleshooting

If issues persist:
1. Use the red threshold tuning tool
2. Check lighting conditions
3. Verify camera calibration
4. Adjust detection parameters in launch files