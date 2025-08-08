# d435i Camera Balloon Detection Fix

## Problem Summary
The drone with d435i camera was hovering without moving during balloon tracking in real-world environments, while working fine in simulation and on other drones.

## Root Cause Analysis
The issue was caused by hardcoded simulation parameters being used in real-world environments:

1. **Hardcoded simulation mode**: `is_sim = True` was hardcoded in `det.py`
2. **Wrong red pixel threshold**: Using simulation threshold (60) instead of real-world threshold
3. **Fixed camera topics**: Not automatically detecting between simulation and real camera topics

## Implemented Fixes

### 1. Automatic Environment Detection (`det.py`)
```python
# Before: Hardcoded
is_sim = True
topic1 = "/camera/color/image_raw"

# After: Automatic detection
if "/rflysim/sensor1/img_rgb" in published_topics:
    topic1 = "/rflysim/sensor1/img_rgb"
    is_sim = True
    rospy.loginfo("检测到仿真环境")
elif "/camera/color/image_raw" in published_topics:
    topic1 = "/camera/color/image_raw"  
    is_sim = False
    rospy.loginfo("检测到真实环境")
```

### 2. Adaptive Red Pixel Thresholds
```python
# Different thresholds for different environments
red_channel_th_sim = 60    # Simulation environment
red_channel_th_real = 120  # Real environment (stronger lighting)

# Dynamic threshold adjustment for real world
if len(red_pixel_coords) == 0 and not self.is_sim:
    adaptive_threshold = min(red_mean + red_std, red_max * 0.8)
```

### 3. Enhanced Debugging and Logging
- Added detailed logging in both `det.py` and `hit.cpp`
- Shows detection mode, thresholds, and balloon tracking status
- Better error messages when balloon detection fails

## Usage

### For Real d435i Camera Environment:
1. Run the test script: `./test_balloon_detection.sh`
2. Start sensors: `./sh/sensor.sh`
3. Start planning: `./sh/func_planner.sh`
4. Monitor logs for: `"Detection mode: Real, Red threshold: 120"`

### For Simulation Environment:
1. Start simulation
2. The system automatically detects and uses simulation parameters
3. Monitor logs for: `"Detection mode: Simulation, Red threshold: 60"`

## Debugging

### Check Detection Status:
```bash
# Monitor object detection
rostopic echo /objects

# Check detection logs
rosnode info /det_node
```

### Common Issues and Solutions:

1. **No red pixels found**: 
   - Check lighting conditions
   - Ensure balloon is clearly red
   - System will automatically try lower thresholds

2. **Wrong environment detected**:
   - Check available topics: `rostopic list`
   - Ensure camera is properly started

3. **Balloon detected but drone doesn't move**:
   - Check `/planning/pos_cmd` topic
   - Monitor hit node logs for tracking status

## Key Files Modified:
- `src/object_det/scripts/det.py` - Main detection logic
- `src/mission_pkg/src/plugins/action/hit.cpp` - Balloon tracking logic
- `test_balloon_detection.sh` - Validation script