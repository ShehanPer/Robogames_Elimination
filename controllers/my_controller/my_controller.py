"""my_controller controller."""

from controller import Robot

# Create the Robot instance
robot = Robot()

# Get the time step of the current world
timestep = int(robot.getBasicTimeStep())

# Function to get device instances
def get_device(device_name):
    device = robot.getDevice(device_name)
    if device:
        device.enable(timestep)
    return device

# Get distance sensors (proximity)
ps_names = ['ps5', 'ps2', 'ps7', 'ps0']
proximity_sensors = [get_device(name) for name in ps_names]

# Get ultrasonic sensors
us_names = ['us0', 'us1', 'us2']  # Front, Left, Right
ultrasonic_sensors = [get_device(name) for name in us_names]

# Function to convert ultrasonic readings (depends on the LUT)
def ultrasonic_to_distance(value):
    return value * 0.01  # Adjust scaling based on your lookup table

# Main loop
while robot.step(timestep) != -1:
    # Read proximity sensors
    for ps in proximity_sensors:
        if ps:
            print(f"{ps.getName()} reading: {ps.getValue()}")

    # Read ultrasonic sensors
    for us in ultrasonic_sensors:
        if us:
            raw_value = us.getValue()
            distance = ultrasonic_to_distance(raw_value)  # Convert to meters
            print(f"{us.getName()} raw: {raw_value}, distance: {distance:.3f}m")

    pass  # Keep simulation running
