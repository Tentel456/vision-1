from signal_detector import SignalDetector
from drone_controller import DroneController
import time
import threading
from queue import Queue
import logging

# Configure logging
logging.basicConfig(level=logging.INFO,
                   format='%(asctime)s - %(levelname)s - %(message)s')

class DroneDefenseSystem:
    def __init__(self, drone_connection_string='udpin:localhost:14550'):
        self.signal_detector = SignalDetector()
        self.drone_controller = DroneController(drone_connection_string)
        self.signal_queue = Queue()
        self.is_running = False
        self.drone_busy = False

    def start(self):
        """Start the defense system"""
        try:
            # Initialize components
            self.signal_detector.initialize()
            self.drone_controller.connect()
            
            # Start signal monitoring
            self.is_running = True
            self.signal_detector.start_monitoring()
            
            # Start processing thread
            self.processing_thread = threading.Thread(target=self._process_signals)
            self.processing_thread.start()
            
            logging.info("Drone defense system started")
            
        except Exception as e:
            logging.error(f"Error starting system: {e}")
            self.stop()
            raise

    def _process_signals(self):
        """Process detected signals and coordinate drone response"""
        while self.is_running:
            signals = self.signal_detector.get_detected_signals()
            
            for signal_data in signals:
                if not self.drone_busy:
                    self._handle_threat(signal_data)
            
            time.sleep(1)

    def _handle_threat(self, signal_data):
        """Handle detected threat"""
        try:
            self.drone_busy = True
            logging.info(f"Threat detected at frequency: {signal_data['frequency']/1e6:.2f} MHz")
            
            # Take off
            self.drone_controller.arm_and_takeoff(target_altitude=20)
            
            # Here you would implement the logic to determine the target's location
            # This could involve triangulation, direction finding, etc.
            # For now, we'll use a placeholder location
            target_location = {
                'lat': 0.0,  # Replace with actual target location
                'lon': 0.0,  # Replace with actual target location
                'alt': 15.0  # Replace with actual target altitude
            }
            
            # Move to target and neutralize
            self.drone_controller.goto_position(
                target_location['lat'],
                target_location['lon'],
                target_location['alt']
            )
            
            # Perform neutralization
            self.drone_controller.neutralize_target(target_location)
            
            # Return to base
            self.drone_controller.return_to_launch()
            self.drone_controller.land()
            
            logging.info("Threat neutralization completed")
            
        except Exception as e:
            logging.error(f"Error handling threat: {e}")
            self.drone_controller.return_to_launch()
            self.drone_controller.land()
        
        finally:
            self.drone_busy = False

    def stop(self):
        """Stop the defense system"""
        self.is_running = False
        self.signal_detector.stop_monitoring()
        
        if hasattr(self, 'processing_thread'):
            self.processing_thread.join()
        
        self.drone_controller.close()
        logging.info("Drone defense system stopped")

if __name__ == "__main__":
    # Create and start the defense system
    system = DroneDefenseSystem()
    try:
        system.start()
        
        # Keep the main thread running
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nShutting down...")
        system.stop() 