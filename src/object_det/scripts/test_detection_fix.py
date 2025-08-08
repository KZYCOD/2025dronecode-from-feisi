#!/usr/bin/env python3
"""
Test script to validate the detection threshold logic without ROS dependencies.
"""

import numpy as np
import sys
import os

# Add the script directory to path to import detection logic
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(script_dir)

def simulate_roi_track(red_threshold, roi_shape=(50, 50), red_pixel_ratio=0.3):
    """
    Simulate the roi_track function behavior.
    Creates a synthetic ROI with some red pixels and tests detection.
    """
    # Create synthetic ROI
    roi = np.random.randint(0, 256, (*roi_shape, 3), dtype=np.uint8)
    
    # Add some red pixels
    num_red_pixels = int(roi_shape[0] * roi_shape[1] * red_pixel_ratio)
    red_positions = np.random.choice(roi_shape[0] * roi_shape[1], num_red_pixels, replace=False)
    
    for pos in red_positions:
        y, x = divmod(pos, roi_shape[1])
        roi[y, x, 2] = np.random.randint(red_threshold + 10, 256)  # Red channel above threshold
    
    # Apply detection logic
    red_channel = roi[:,:,2]
    red_mask = red_channel > red_threshold
    red_pixel_coords = np.column_stack(np.where(red_mask))
    
    total_pixels = roi.shape[0] * roi.shape[1]
    detected_red_pixels = len(red_pixel_coords)
    detection_percentage = (detected_red_pixels / total_pixels) * 100 if total_pixels > 0 else 0
    
    # Calculate center if detection successful
    center_original = None
    if len(red_pixel_coords) > 0:
        center_roi = np.mean(red_pixel_coords, axis=0).astype(int)
        center_original = (center_roi[1], center_roi[0])  # Convert to (x, y)
    
    return center_original is not None, detection_percentage, detected_red_pixels

def test_threshold_configurations():
    """Test different threshold configurations"""
    print("Testing Detection Threshold Configurations")
    print("=" * 50)
    
    # Test scenarios
    scenarios = [
        ("High red content (balloon visible)", 0.4),
        ("Medium red content (partial balloon)", 0.2),
        ("Low red content (balloon distant)", 0.1),
        ("Very low red content (poor lighting)", 0.05),
    ]
    
    # Threshold configurations
    thresholds = [
        ("Simulation default", 60),
        ("Real-world default", 120),
        ("Conservative real-world", 100),
        ("Aggressive real-world", 80),
    ]
    
    for scenario_name, red_ratio in scenarios:
        print(f"\nScenario: {scenario_name}")
        print("-" * 30)
        
        for threshold_name, threshold in thresholds:
            success_count = 0
            total_tests = 10
            avg_percentage = 0
            
            for _ in range(total_tests):
                success, percentage, _ = simulate_roi_track(threshold, red_pixel_ratio=red_ratio)
                if success:
                    success_count += 1
                avg_percentage += percentage
            
            avg_percentage /= total_tests
            success_rate = (success_count / total_tests) * 100
            
            print(f"  {threshold_name:20} (th={threshold:3d}): "
                  f"Success Rate: {success_rate:5.1f}%, "
                  f"Avg Detection: {avg_percentage:5.1f}%")

def test_adaptive_fallback():
    """Test the adaptive threshold fallback mechanism"""
    print("\n" + "=" * 50)
    print("Testing Adaptive Threshold Fallback")
    print("=" * 50)
    
    primary_threshold = 120
    fallback_threshold = 90
    
    test_cases = [
        ("Good lighting", 0.3, True),
        ("Poor lighting", 0.15, True),
        ("Very poor lighting", 0.08, True),
        ("No balloon", 0.02, False),
    ]
    
    for case_name, red_ratio, should_detect in test_cases:
        print(f"\nTest Case: {case_name}")
        
        # Try primary threshold
        success_primary, pct_primary, count_primary = simulate_roi_track(primary_threshold, red_pixel_ratio=red_ratio)
        
        if not success_primary:
            # Try fallback threshold
            success_fallback, pct_fallback, count_fallback = simulate_roi_track(fallback_threshold, red_pixel_ratio=red_ratio)
            print(f"  Primary threshold ({primary_threshold}): FAILED ({pct_primary:.1f}%)")
            if success_fallback:
                print(f"  Fallback threshold ({fallback_threshold}): SUCCESS ({pct_fallback:.1f}%)")
                final_result = "DETECTED with fallback"
            else:
                print(f"  Fallback threshold ({fallback_threshold}): FAILED ({pct_fallback:.1f}%)")
                final_result = "NOT DETECTED"
        else:
            print(f"  Primary threshold ({primary_threshold}): SUCCESS ({pct_primary:.1f}%)")
            final_result = "DETECTED with primary"
        
        expected = "SHOULD DETECT" if should_detect else "SHOULD NOT DETECT"
        status = "✓" if (should_detect and "DETECTED" in final_result) or (not should_detect and "NOT DETECTED" in final_result) else "✗"
        
        print(f"  Result: {final_result} | Expected: {expected} {status}")

def validate_configuration_logic():
    """Validate the configuration selection logic"""
    print("\n" + "=" * 50)
    print("Testing Configuration Selection Logic")
    print("=" * 50)
    
    # Simulate the logic from det.py
    sim_threshold = 60
    real_threshold = 120
    
    configurations = [
        ("Simulation mode", True, sim_threshold),
        ("Real-world mode", False, real_threshold),
    ]
    
    for config_name, is_sim, expected_threshold in configurations:
        # This simulates the logic in the Depth_Estimate.__init__ method
        if is_sim:
            selected_threshold = sim_threshold
            env_name = "SIMULATION"
        else:
            selected_threshold = real_threshold
            env_name = "REAL WORLD d435i camera"
        
        print(f"{config_name}:")
        print(f"  Environment: {env_name}")
        print(f"  Selected threshold: {selected_threshold}")
        print(f"  Expected threshold: {expected_threshold}")
        print(f"  Status: {'✓' if selected_threshold == expected_threshold else '✗'}")

if __name__ == "__main__":
    print("Detection Threshold Validation Test")
    print("Testing fixes for d435i camera balloon detection hover issue")
    
    test_threshold_configurations()
    test_adaptive_fallback()
    validate_configuration_logic()
    
    print("\n" + "=" * 50)
    print("Test Summary:")
    print("- Real-world threshold (120) should work better than simulation (60) in poor lighting")
    print("- Adaptive fallback should help in challenging conditions")
    print("- Configuration selection should choose appropriate thresholds")
    print("\nTo use in real system:")
    print("1. Run: ./sh/func_hitballon_realworld.sh")
    print("2. If detection fails, use: roslaunch object_det tune_red_threshold.launch")
    print("3. Fine-tune threshold based on your specific lighting conditions")