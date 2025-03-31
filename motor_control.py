import serial
import time
import logging

logger = logging.getLogger(__name__)

class MotorController:
    def __init__(self, port='/dev/ttyUSB0', baud_rate=115200):
        try:
            self.serial = serial.Serial(port, baud_rate, timeout=1)
            logger.info(f"Connected to motor controller on {port}")
        except serial.SerialException as e:
            logger.error(f"Failed to connect to motor controller: {e}")
            raise
            
    def move_forward(self, speed=100):
        """Move the vehicle forward at specified speed (0-255)"""
        command = f"F{speed}\n"
        self._send_command(command)
        
    def move_backward(self, speed=100):
        """Move the vehicle backward at specified speed (0-255)"""
        command = f"B{speed}\n"
        self._send_command(command)
        
    def turn_left(self, speed=100):
        """Turn left at specified speed (0-255)"""
        command = f"L{speed}\n"
        self._send_command(command)
        
    def turn_right(self, speed=100):
        """Turn right at specified speed (0-255)"""
        command = f"R{speed}\n"
        self._send_command(command)
        
    def stop(self):
        """Stop the vehicle"""
        command = "S\n"
        self._send_command(command)
        
    def _send_command(self, command):
        try:
            self.serial.write(command.encode())
            time.sleep(0.1)  # Small delay to ensure command is processed
        except serial.SerialException as e:
            logger.error(f"Failed to send command: {e}")
            
    def close(self):
        """Close the serial connection"""
        if hasattr(self, 'serial'):
            self.serial.close()
            logger.info("Motor controller connection closed") 