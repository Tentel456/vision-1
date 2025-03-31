import logging
import time
from enum import Enum
import serial
import threading

logger = logging.getLogger(__name__)

class WeaponType(Enum):
    SELF_DESTRUCT = "self_destruct"
    BALLISTIC = "ballistic"

class WeaponSystem:
    def __init__(self, weapon_type=WeaponType.SELF_DESTRUCT, port='/dev/ttyUSB1', baud_rate=115200):
        self.weapon_type = weapon_type
        self.armed = False
        self.safety_engaged = True
        self.ready_to_fire = False
        
        # Initialize weapon control interface
        try:
            self.serial = serial.Serial(port, baud_rate, timeout=1)
            logger.info(f"Connected to weapon system on {port}")
        except serial.SerialException as e:
            logger.error(f"Failed to connect to weapon system: {e}")
            raise
            
        # Initialize weapon-specific parameters
        if self.weapon_type == WeaponType.BALLISTIC:
            self.ammo_count = 100  # Default ammo count
            self.fire_rate = 600  # Rounds per minute
            self.last_fire_time = 0
            self.min_fire_interval = 60.0 / self.fire_rate  # Minimum time between shots
            
    def arm(self):
        """Arm the weapon system"""
        if not self.safety_engaged:
            self.armed = True
            logger.info("Weapon system armed")
            return True
        return False
        
    def disarm(self):
        """Disarm the weapon system"""
        self.armed = False
        self.ready_to_fire = False
        logger.info("Weapon system disarmed")
        
    def engage_safety(self):
        """Engage the safety mechanism"""
        self.safety_engaged = True
        self.armed = False
        self.ready_to_fire = False
        logger.info("Safety engaged")
        
    def disengage_safety(self):
        """Disengage the safety mechanism"""
        self.safety_engaged = False
        logger.info("Safety disengaged")
        
    def prepare_to_fire(self):
        """Prepare the weapon system for firing"""
        if self.armed and not self.safety_engaged:
            self.ready_to_fire = True
            logger.info("Weapon system ready to fire")
            return True
        return False
        
    def fire(self):
        """Fire the weapon"""
        if not self.ready_to_fire:
            logger.warning("Weapon system not ready to fire")
            return False
            
        if self.weapon_type == WeaponType.SELF_DESTRUCT:
            return self._execute_self_destruct()
        elif self.weapon_type == WeaponType.BALLISTIC:
            return self._fire_ballistic()
            
    def _execute_self_destruct(self):
        """Execute self-destruct sequence"""
        try:
            # Send self-destruct command
            command = "SD\n"
            self.serial.write(command.encode())
            logger.warning("Self-destruct sequence initiated")
            
            # Start countdown thread
            countdown_thread = threading.Thread(target=self._self_destruct_countdown)
            countdown_thread.start()
            
            return True
        except Exception as e:
            logger.error(f"Failed to execute self-destruct: {e}")
            return False
            
    def _self_destruct_countdown(self):
        """Countdown sequence for self-destruct"""
        countdown = 5
        while countdown > 0:
            logger.warning(f"Self-destruct in {countdown} seconds...")
            time.sleep(1)
            countdown -= 1
        logger.critical("SELF-DESTRUCT EXECUTED")
        
    def _fire_ballistic(self):
        """Fire ballistic weapon"""
        current_time = time.time()
        if current_time - self.last_fire_time < self.min_fire_interval:
            logger.warning("Fire rate limit reached")
            return False
            
        try:
            if self.ammo_count <= 0:
                logger.warning("Out of ammunition")
                return False
                
            # Send fire command
            command = "FIRE\n"
            self.serial.write(command.encode())
            
            self.ammo_count -= 1
            self.last_fire_time = current_time
            logger.info(f"Fired ballistic weapon. Ammo remaining: {self.ammo_count}")
            
            return True
        except Exception as e:
            logger.error(f"Failed to fire ballistic weapon: {e}")
            return False
            
    def get_status(self):
        """Get current weapon system status"""
        return {
            "weapon_type": self.weapon_type.value,
            "armed": self.armed,
            "safety_engaged": self.safety_engaged,
            "ready_to_fire": self.ready_to_fire,
            "ammo_count": self.ammo_count if self.weapon_type == WeaponType.BALLISTIC else None
        }
        
    def close(self):
        """Close the weapon system interface"""
        if hasattr(self, 'serial'):
            self.serial.close()
            logger.info("Weapon system interface closed") 