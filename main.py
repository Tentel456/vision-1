import cv2
import torch
from ultralytics import YOLO
import numpy as np
from pathlib import Path
import time
import logging
from weapon_system import WeaponSystem, WeaponType
from motor_control import MotorController
from target_tracking import TargetTracker
from weapon_calibration import WeaponCalibrator
from safety_system import SafetySystem, SafetyLevel
import threading
import queue
import json
import os

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class AutonomousVehicle:
    def __init__(self, weapon_type=WeaponType.SELF_DESTRUCT):
        # Initialize YOLO model
        self.model = YOLO('yolov8n.pt')
        
        # Initialize camera
        self.camera = cv2.VideoCapture(0)
        if not self.camera.isOpened():
            raise RuntimeError("Failed to open camera")
            
        # Movement detection parameters
        self.prev_frame = None
        self.movement_threshold = 1000  # Adjust based on testing
        
        # Initialize systems
        self.motor_controller = MotorController()
        self.weapon_system = WeaponSystem(weapon_type=weapon_type)
        self.target_tracker = TargetTracker()
        self.weapon_calibrator = WeaponCalibrator()
        self.safety_system = SafetySystem()
        
        # Engagement parameters
        self.engagement_distance = 5.0  # meters
        self.target_locked = False
        self.engagement_countdown = 0
        
        # Environmental parameters
        self.wind_speed = 0.0
        self.wind_direction = 0.0
        self.temperature = 20.0
        self.humidity = 50.0
        self.elevation = 0.0
        
        # Command queue for thread-safe communication
        self.command_queue = queue.Queue()
        
        # Initialize system parameters
        self._update_system_parameters()
        
    def _update_system_parameters(self):
        """Обновляет параметры систем в системе безопасности"""
        # Обновляем параметры камеры
        camera_params = {
            "fps": self.camera.get(cv2.CAP_PROP_FPS),
            "resolution": (int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH)),
                         int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))),
            "exposure": self.camera.get(cv2.CAP_PROP_EXPOSURE),
            "gain": self.camera.get(cv2.CAP_PROP_GAIN)
        }
        self.safety_system.update_system_parameters("camera", camera_params)
        
        # Обновляем параметры оружия
        weapon_params = {
            "safety_engaged": self.weapon_system.safety_engaged,
            "armed": self.weapon_system.armed,
            "ammo_count": self.weapon_system.ammo_count if hasattr(self.weapon_system, 'ammo_count') else 0
        }
        self.safety_system.update_system_parameters("weapon", weapon_params)
        
        # Обновляем параметры окружающей среды
        env_params = {
            "temperature": self.temperature,
            "humidity": self.humidity,
            "wind_speed": self.wind_speed,
            "visibility": 100.0  # Пример значения, должно быть измерено
        }
        self.safety_system.update_system_parameters("environment", env_params)
        
    def detect_movement(self, frame):
        if self.prev_frame is None:
            self.prev_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            return False
            
        # Convert current frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Calculate frame difference
        frame_diff = cv2.absdiff(gray, self.prev_frame)
        
        # Apply threshold
        _, thresh = cv2.threshold(frame_diff, 30, 255, cv2.THRESH_BINARY)
        
        # Calculate movement
        movement = np.sum(thresh)
        
        # Update previous frame
        self.prev_frame = gray
        
        return movement > self.movement_threshold
        
    def detect_targets(self, frame):
        results = self.model(frame)
        return results[0]
        
    def calculate_target_distance(self, target_position):
        """Calculate approximate distance to target based on bounding box size"""
        x1, y1, x2, y2 = target_position
        box_height = y2 - y1
        # This is a simplified calculation - you'll need to calibrate based on your camera
        distance = 1.0 / (box_height / frame.shape[0])
        return distance
        
    def engage_target(self, target):
        """Engage the detected target"""
        if not target or not target.bbox:
            return
            
        # Проверяем безопасность перед атакой
        safety_status = self.safety_system.get_safety_status()
        if safety_status["safety_level"] != SafetyLevel.NORMAL.value:
            logger.warning("Атака отменена из-за проблем с безопасностью")
            return
            
        # Calculate distance to target
        distance = self.calculate_target_distance(target.bbox)
        
        if distance <= self.engagement_distance:
            # Calculate ballistic correction
            offset_x, offset_y = self.weapon_calibrator.calculate_ballistic_correction(
                distance=distance,
                wind_speed=self.wind_speed,
                wind_direction=self.wind_direction,
                temperature=self.temperature,
                humidity=self.humidity,
                elevation=self.elevation
            )
            
            # Prepare weapon system
            if not self.weapon_system.armed:
                self.weapon_system.disengage_safety()
                self.weapon_system.arm()
                
            if self.weapon_system.prepare_to_fire():
                # Start engagement countdown
                if self.engagement_countdown == 0:
                    self.engagement_countdown = 3
                    logger.info("Starting engagement countdown...")
                    
                self.engagement_countdown -= 1
                
                if self.engagement_countdown == 0:
                    logger.warning("Engaging target!")
                    self.weapon_system.fire()
                    self.target_locked = False
                    self.engagement_countdown = 0
                    
    def process_frame(self):
        ret, frame = self.camera.read()
        if not ret:
            logger.error("Failed to capture frame")
            return
            
        # Detect movement
        movement_detected = self.detect_movement(frame)
        
        # Detect targets using YOLO
        results = self.detect_targets(frame)
        
        # Process detections for tracking
        detections = []
        for result in results.boxes.data:
            x1, y1, x2, y2, conf, cls = result
            if conf > 0.5:  # Confidence threshold
                detections.append(((x1, y1, x2, y2), conf, cls))
                
        # Update target tracker
        tracked_targets = self.target_tracker.update(detections, time.time())
        
        return frame, movement_detected, tracked_targets
        
    def run(self):
        try:
            while True:
                # Проверяем безопасность
                safety_status = self.safety_system.get_safety_status()
                if safety_status["emergency_stop"]:
                    logger.critical("Emergency stop triggered!")
                    self.emergency_stop()
                    break
                    
                frame, movement_detected, tracked_targets = self.process_frame()
                
                if movement_detected:
                    logger.info("Movement detected!")
                    # Move towards movement
                    self.motor_controller.move_forward()
                    
                # Process tracked targets
                for target in tracked_targets:
                    if target.confidence > 0.7:  # High confidence threshold for engagement
                        logger.info(f"High confidence target detected: {target.confidence:.2f}")
                        self.engage_target(target)
                        
                # Обновляем параметры систем
                self._update_system_parameters()
                        
                # Display frame (for testing)
                cv2.imshow('Autonomous Vehicle View', frame)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                    
        finally:
            self.cleanup()
            
    def emergency_stop(self):
        """Execute emergency stop sequence"""
        logger.critical("Executing emergency stop sequence")
        self.weapon_system.disarm()
        self.weapon_system.engage_safety()
        self.motor_controller.stop()
        self.cleanup()
        
    def cleanup(self):
        """Clean up resources"""
        self.camera.release()
        cv2.destroyAllWindows()
        self.motor_controller.close()
        self.weapon_system.close()
        self.safety_system.cleanup()

if __name__ == "__main__":
    # Initialize vehicle with self-destruct weapon
    vehicle = AutonomousVehicle(weapon_type=WeaponType.SELF_DESTRUCT)
    vehicle.run() 