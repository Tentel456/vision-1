from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math

class DroneController:
    def __init__(self, connection_string='udpin:localhost:14550'):
        self.connection_string = connection_string
        self.vehicle = None
        self.armed = False
        self.mode = None

    def connect(self):
        """Connect to the drone"""
        try:
            print(f"Connecting to drone on {self.connection_string}")
            self.vehicle = connect(self.connection_string, wait_ready=True)
            print("Connected to drone")
        except Exception as e:
            print(f"Error connecting to drone: {e}")
            raise

    def arm_and_takeoff(self, target_altitude=10):
        """Arm the drone and take off to specified altitude"""
        print("Basic pre-arm checks")
        while not self.vehicle.is_armable:
            print("Waiting for vehicle to initialize...")
            time.sleep(1)

        print("Arming motors")
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed:
            print("Waiting for arming...")
            time.sleep(1)

        print("Taking off!")
        self.vehicle.simple_takeoff(target_altitude)

        while True:
            print(f"Altitude: {self.vehicle.location.global_relative_frame.alt}")
            if self.vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
                print("Reached target altitude")
                break
            time.sleep(1)

    def goto_position(self, lat, lon, alt):
        """Go to specified position"""
        target_location = LocationGlobalRelative(lat, lon, alt)
        self.vehicle.simple_goto(target_location)
        
        while True:
            current_location = self.vehicle.location.global_relative_frame
            distance = self._get_distance_metres(current_location, target_location)
            if distance < 1.0:
                print("Reached target position")
                break
            time.sleep(1)

    def neutralize_target(self, target_location):
        """Move to target location and perform neutralization"""
        print(f"Moving to neutralize target at {target_location}")
        self.goto_position(target_location.lat, target_location.lon, target_location.alt)
        
        # Here you would implement the actual neutralization mechanism
        # This could be deploying a net, using directed energy, etc.
        print("Target neutralized")

    def return_to_launch(self):
        """Return to launch position"""
        print("Returning to launch")
        self.vehicle.mode = VehicleMode("RTL")
        while True:
            if self.vehicle.mode.name == "RTL":
                print("Returning to launch")
                break
            time.sleep(1)

    def land(self):
        """Land the drone"""
        print("Landing")
        self.vehicle.mode = VehicleMode("LAND")
        while True:
            if self.vehicle.mode.name == "LAND":
                print("Landing")
                break
            time.sleep(1)

    def close(self):
        """Close the connection to the drone"""
        if self.vehicle:
            self.vehicle.close()

    def _get_distance_metres(self, aLocation1, aLocation2):
        """Calculate distance between two points in meters"""
        dlat = aLocation2.lat - aLocation1.lat
        dlong = aLocation2.lon - aLocation1.lon
        return math.sqrt((dlat*dlat + dlong*dlong) * 1.113195e5**2) 