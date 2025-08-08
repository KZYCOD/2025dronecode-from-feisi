#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
Red threshold tuning tool for d435i camera balloon detection.
This script helps users find the optimal red pixel detection threshold for their specific setup.
"""

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import sys

class RedThresholdTuner:
    def __init__(self):
        rospy.init_node('red_threshold_tuner')
        self.bridge = CvBridge()
        self.current_threshold = 120  # Start with real-world default
        self.image = None
        
        # Subscribe to camera
        camera_topic = rospy.get_param('~camera_topic', '/camera/color/image_raw')
        self.image_sub = rospy.subscribe(camera_topic, Image, self.image_callback)
        
        rospy.loginfo("Red threshold tuner started. Subscribed to %s", camera_topic)
        rospy.loginfo("Instructions:")
        rospy.loginfo("- Press 'q' to quit")
        rospy.loginfo("- Press '+'/'-' to increase/decrease threshold")
        rospy.loginfo("- Press 's' to save current threshold")
        rospy.loginfo("- Click on red balloon area to test detection")
        
    def image_callback(self, msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("Failed to convert image: %s", e)
    
    def test_red_detection(self, x, y, size=50):
        """Test red detection in a region around the clicked point"""
        if self.image is None:
            return False, 0
            
        h, w = self.image.shape[:2]
        left = max(0, x - size//2)
        right = min(w, x + size//2)
        top = max(0, y - size//2)
        bottom = min(h, y + size//2)
        
        roi = self.image[top:bottom, left:right]
        red_channel = roi[:,:,2]
        red_mask = red_channel > self.current_threshold
        red_pixels = np.sum(red_mask)
        total_pixels = roi.shape[0] * roi.shape[1]
        percentage = (red_pixels / total_pixels) * 100 if total_pixels > 0 else 0
        
        return red_pixels > 0, percentage
    
    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            success, percentage = self.test_red_detection(x, y)
            rospy.loginfo("Clicked at (%d, %d) - Threshold: %d, Red pixels: %.1f%%, Detection: %s", 
                         x, y, self.current_threshold, percentage, "SUCCESS" if success else "FAILED")
    
    def run(self):
        cv2.namedWindow('Red Threshold Tuning')
        cv2.setMouseCallback('Red Threshold Tuning', self.mouse_callback)
        
        while not rospy.is_shutdown():
            if self.image is not None:
                # Create a copy for visualization
                vis_image = self.image.copy()
                
                # Apply red threshold to entire image for visualization
                red_channel = vis_image[:,:,2]
                red_mask = red_channel > self.current_threshold
                
                # Highlight detected red pixels
                vis_image[red_mask] = [0, 0, 255]
                
                # Add text overlay
                text = f"Threshold: {self.current_threshold} | Press +/- to adjust, 's' to save, 'q' to quit"
                cv2.putText(vis_image, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                cv2.imshow('Red Threshold Tuning', vis_image)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('+') or key == ord('='):
                self.current_threshold = min(255, self.current_threshold + 5)
                rospy.loginfo("Increased threshold to %d", self.current_threshold)
            elif key == ord('-'):
                self.current_threshold = max(0, self.current_threshold - 5)
                rospy.loginfo("Decreased threshold to %d", self.current_threshold)
            elif key == ord('s'):
                rospy.loginfo("RECOMMENDED THRESHOLD: %d", self.current_threshold)
                rospy.loginfo("Add this line to your launch file:")
                rospy.loginfo('<param name="red_threshold" value="%d"/>', self.current_threshold)
        
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        tuner = RedThresholdTuner()
        tuner.run()
    except rospy.ROSInterruptException:
        pass