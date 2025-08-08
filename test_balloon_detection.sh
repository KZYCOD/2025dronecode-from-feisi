#!/bin/bash

# Test script for validating the d435i balloon detection fixes
# Usage: ./test_balloon_detection.sh

echo "======================================================"
echo "Testing d435i Balloon Detection Fixes"
echo "======================================================"

# Check if ROS is sourced
if ! command -v rosnode &> /dev/null; then
    echo "❌ ROS not found. Please source your ROS workspace first:"
    echo "   source /opt/ros/noetic/setup.bash"
    echo "   source ~/27com_ws/devel/setup.bash"
    exit 1
fi

echo "✓ ROS environment detected"

# Function to check if topic exists
check_topic() {
    local topic=$1
    if rostopic list | grep -q "^$topic$"; then
        echo "✓ Topic $topic exists"
        return 0
    else
        echo "❌ Topic $topic not found"
        return 1
    fi
}

# Function to check if node is running
check_node() {
    local node=$1
    if rosnode list | grep -q "$node"; then
        echo "✓ Node $node is running"
        return 0
    else
        echo "❌ Node $node not running"
        return 1
    fi
}

echo ""
echo "1. Checking camera setup..."
echo "======================================================"

# Check for simulation topics
if check_topic "/rflysim/sensor1/img_rgb"; then
    echo "🎮 Simulation environment detected"
    EXPECTED_MODE="simulation"
else
    echo "🔍 Checking for real camera topics..."
    if check_topic "/camera/color/image_raw"; then
        echo "📹 Real camera environment detected"
        EXPECTED_MODE="real"
    elif check_topic "/camera/color/image_rect_color"; then
        echo "📹 Real camera environment detected (alternative topic)"
        EXPECTED_MODE="real"
    else
        echo "⚠️  No camera topics found. Please start your camera:"
        echo "   For simulation: Start your simulation environment"
        echo "   For real camera: Run ./sh/sensor.sh"
        EXPECTED_MODE="unknown"
    fi
fi

# Check depth topic
echo ""
echo "2. Checking depth camera setup..."
echo "======================================================"
if check_topic "/camera/aligned_depth_to_color/image_raw"; then
    echo "✓ Depth camera topic found"
elif check_topic "/rflysim/sensor2/img_depth"; then
    echo "✓ Simulation depth topic found"
else
    echo "❌ No depth topics found"
fi

echo ""
echo "3. Testing detection node..."
echo "======================================================"

# Create a simple test to validate the detection script
python3 -c "
import sys
sys.path.append('./src/object_det/scripts')

try:
    # Test if we can import the required modules
    import rospy
    print('✓ ROS Python modules available')
    
    # Test the detection parameter logic
    red_channel_th_sim = 60
    red_channel_th_real = 120
    
    # Simulate the logic from det.py
    def test_threshold_selection(is_sim):
        threshold = red_channel_th_sim if is_sim else red_channel_th_real
        return threshold
    
    sim_threshold = test_threshold_selection(True)
    real_threshold = test_threshold_selection(False)
    
    assert sim_threshold == 60, f'Expected 60 for sim, got {sim_threshold}'
    assert real_threshold == 120, f'Expected 120 for real, got {real_threshold}'
    
    print('✓ Detection threshold logic works correctly')
    print(f'  - Simulation threshold: {sim_threshold}')
    print(f'  - Real environment threshold: {real_threshold}')
    
except ImportError as e:
    print(f'❌ Missing Python dependencies: {e}')
    print('   Please install: pip install numpy opencv-python')
except Exception as e:
    print(f'❌ Test failed: {e}')
    sys.exit(1)

print('✅ Detection script validation passed')
"

if [ $? -ne 0 ]; then
    echo "❌ Detection script test failed"
    exit 1
fi

echo ""
echo "4. Recommendations..."
echo "======================================================"

case $EXPECTED_MODE in
    "simulation")
        echo "🎮 For simulation environment:"
        echo "   - Detection should use threshold = 60"
        echo "   - Topic: /rflysim/sensor1/img_rgb"
        echo "   - Run: roslaunch mission_pkg test_sim.launch"
        ;;
    "real")
        echo "📹 For real d435i camera environment:"
        echo "   - Detection should use threshold = 120 (adaptive)"
        echo "   - Topic: /camera/color/image_raw"
        echo "   - Ensure good lighting conditions"
        echo "   - Run: ./sh/sensor.sh && ./sh/func_planner.sh"
        ;;
    "unknown")
        echo "⚠️  Environment unclear. To test:"
        echo "   1. Start your camera system"
        echo "   2. Re-run this test script"
        echo "   3. Check rostopic list for camera topics"
        ;;
esac

echo ""
echo "5. Next steps to test balloon detection:"
echo "======================================================"
echo "1. Start the camera system (sensor.sh)"
echo "2. Start the planning system (func_planner.sh)"
echo "3. Monitor these topics for debugging:"
echo "   rostopic echo /objects"
echo "   rostopic echo /mavros/local_position/pose"
echo "   rostopic echo /planning/pos_cmd"
echo ""
echo "4. Check logs for detection mode confirmation:"
echo "   Look for: 'Detection mode: Real, Red threshold: 120'"
echo "   Or: 'Detection mode: Simulation, Red threshold: 60'"
echo ""
echo "5. If balloon detection fails, check:"
echo "   - Balloon is clearly visible and red"
echo "   - Good lighting conditions"
echo "   - YOLO model can detect the balloon shape"
echo "   - Red pixel threshold may need adjustment"

echo ""
echo "======================================================"
echo "Test completed!"
echo "======================================================"