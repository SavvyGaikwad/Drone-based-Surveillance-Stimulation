import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
from matplotlib.patches import Circle
import random
import time
import os
import pickle
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
# Add MySQL imports here
import mysql.connector
from mysql.connector import Error
import datetime

def create_database_connection():
    """Create a connection to MySQL database"""
    try:
        connection = mysql.connector.connect(
            host='localhost',          # Change if your MySQL server is hosted elsewhere
            user='root',      # Replace with your MySQL username
            password='12210812',  # Replace with your MySQL password
            database='MLDL'            # Database name
        )
        
        if connection.is_connected():
            return connection
        else:
            print("Failed to connect to MySQL database")
            return None
            
    except Error as e:
        print(f"Error connecting to MySQL: {e}")
        return None

def setup_database():
    """Set up the database and table if they don't exist"""
    try:
        # First connect without specifying the database
        connection = mysql.connector.connect(
            host='localhost',
            user='root',
            password='12210812'
        )
        
        if connection.is_connected():
            cursor = connection.cursor()
            
            # Create database if it doesn't exist
            cursor.execute("CREATE DATABASE IF NOT EXISTS MLDL")
            
            # Switch to the MLDL database
            cursor.execute("USE MLDL")
            
            # Create Drones table if it doesn't exist
            create_table_query = """
            CREATE TABLE IF NOT EXISTS Drones (
                id INT AUTO_INCREMENT PRIMARY KEY,
                drone_id INT NOT NULL,
                timestamp DATETIME NOT NULL,
                role VARCHAR(20) NOT NULL,
                target_x FLOAT NOT NULL,
                target_y FLOAT NOT NULL
            )
            """
            cursor.execute(create_table_query)
            
            print("Database and table setup completed successfully")
            
            # Close the connection
            if connection.is_connected():
                cursor.close()
                connection.close()
            
    except Error as e:
        print(f"Error setting up database: {e}")

def log_drone_data(drone_id, role, target_x, target_y):
    """Log drone data to the MySQL database"""
    try:
        connection = create_database_connection()
        if connection is not None:
            cursor = connection.cursor()
            
            # Insert data into the Drones table
            insert_query = """
            INSERT INTO Drones (drone_id, timestamp, role, target_x, target_y)
            VALUES (%s, %s, %s, %s, %s)
            """
            
            # Get current timestamp
            current_time = datetime.datetime.now()
            
            # Execute the query
            cursor.execute(insert_query, (drone_id, current_time, role, target_x, target_y))
            
            # Commit the changes
            connection.commit()
            
            # Close the connection
            if connection.is_connected():
                cursor.close()
                connection.close()
                
    except Error as e:
        print(f"Error logging drone data: {e}")

class TerrainEnvironment:
    """Manages the terrain and environment for the drone simulation"""
    def __init__(self, map_size):
        self.map_size = map_size
        self.terrain_type = "hill"  # Always hill for this simulation
        self.height_map = None
        self.height_map_resolution = 1  # Resolution of height map
        self.generate_terrain()
        
    def generate_terrain(self):
        """Generate a hilly terrain similar to the MATLAB implementation"""
        # Size of the heightmap - higher resolution 
        map_size = self.map_size * 2  # Double resolution for smoother terrain
        
        # Initialize height map
        self.height_map = np.zeros((map_size, map_size))
        
        # Create a mesh grid for generating terrain
        X, Y = np.meshgrid(np.arange(map_size), np.arange(map_size))
        
        # Create a central hill
        center_x = map_size / 2
        center_y = map_size / 2
        
        # Distance from center
        distance = np.sqrt((X - center_x)**2 + (Y - center_y)**2)
        
        # Gaussian hill
        sigma = map_size / 4
        self.height_map = 4 * np.exp(-(distance**2) / (2 * sigma**2))
        
        # Add some smaller hills
        for i in range(3):
            center_x = map_size * random.random()
            center_y = map_size * random.random()
            sigma = map_size / 8
            distance = np.sqrt((X - center_x)**2 + (Y - center_y)**2)
            self.height_map = self.height_map + 2 * np.exp(-(distance**2) / (2 * sigma**2))
        
        # Apply some smoothing (using Gaussian filter)
        from scipy.ndimage import gaussian_filter
        self.height_map = gaussian_filter(self.height_map, sigma=1.5)
        
    def get_terrain_height(self, x, y):
        """Get terrain height at a specific x,y position"""
        # Convert world coordinates to heightmap indices
        map_x = min(int(x * 2), self.height_map.shape[0] - 1)  # Account for double resolution
        map_y = min(int(y * 2), self.height_map.shape[1] - 1)
        
        # Ensure within bounds
        map_x = max(0, min(map_x, self.height_map.shape[0] - 1))
        map_y = max(0, min(map_y, self.height_map.shape[1] - 1))
        
        return self.height_map[map_y, map_x]  # Note: numpy arrays are [y,x]

class DroneFormation:
    """Controls the entire formation of drones with structured movement"""
    def __init__(self, map_size, environment):
        self.map_size = map_size
        self.environment = environment
        
        # Start at the base station
        self.base_station = (1, 1)
        self.x = self.base_station[0]
        self.y = self.base_station[1]
        self.z = self.environment.get_terrain_height(self.x, self.y) + 3  # Height above terrain
        
        # Movement parameters
        self.speed = 0.15  # Speed multiplier
        self.min_altitude = 3  # Minimum altitude above terrain
        
        # Path planning
        self.movement_phase = 0  # 0: outward journey, 1: inward journey
        self.current_waypoint_index = 0
        self.waypoints = []
        self.generate_path()
        
        # Status
        self.status = "waiting"  # waiting, moving
        self.takeoff_delay = 30  # Wait time before takeoff
        
    def generate_path(self):
        """Generate a structured path with multiple concentric squares"""
        # Clear existing waypoints
        self.waypoints = []
        
        # Start from base station
        self.waypoints.append(self.base_station)
        
        # Outer border - square pattern
        margin = 3.0  # Margin from map edge
        outer_size = self.map_size - 2 * margin
        
        # First square - outer border
        # From base to top-left corner
        self.waypoints.append((margin, margin))
        # To top-right corner
        self.waypoints.append((margin + outer_size, margin))
        # To bottom-right corner
        self.waypoints.append((margin + outer_size, margin + outer_size))
        # To bottom-left corner
        self.waypoints.append((margin, margin + outer_size))
        # Back to top-left corner
        self.waypoints.append((margin, margin))
        
        # Create 3 more concentric squares, each getting smaller
        for i in range(1, 4):
            # Each square is smaller than the previous
            inner_margin = margin + i * (outer_size / 8)
            inner_size = outer_size - 2 * i * (outer_size / 8)
            
            # Add the four corners of each inner square
            self.waypoints.append((inner_margin, inner_margin))
            self.waypoints.append((inner_margin + inner_size, inner_margin))
            self.waypoints.append((inner_margin + inner_size, inner_margin + inner_size))
            self.waypoints.append((inner_margin, inner_margin + inner_size))
            self.waypoints.append((inner_margin, inner_margin))
        
        # Add peak/center point
        self.waypoints.append((self.map_size / 2, self.map_size / 2))
        
        # Store the number of outward waypoints to reverse for return journey
        self.outward_waypoints_count = len(self.waypoints)
    
    def update(self):
        # Handle takeoff delay
        if self.status == "waiting":
            if self.takeoff_delay > 0:
                self.takeoff_delay -= 1
                return
            else:
                self.status = "moving"
        
        # For moving formation
        if self.status == "moving":
            # Get current target waypoint
            if self.movement_phase == 0:
                # Outward journey
                target = self.waypoints[self.current_waypoint_index]
            else:
                # Inward journey (reverse path)
                reverse_index = self.outward_waypoints_count - 1 - self.current_waypoint_index
                target = self.waypoints[reverse_index]
            
            # Calculate distance to target
            dx = target[0] - self.x
            dy = target[1] - self.y
            distance = np.sqrt(dx**2 + dy**2)
            
            # If we're close to the current waypoint, move to the next one
            if distance < 0.2:
                self.current_waypoint_index += 1
                
                # Check if we need to switch direction
                if self.movement_phase == 0 and self.current_waypoint_index >= self.outward_waypoints_count:
                    # Switch to inward journey
                    self.movement_phase = 1
                    self.current_waypoint_index = 0
                elif self.movement_phase == 1 and self.current_waypoint_index >= self.outward_waypoints_count:
                    # Reset to outward journey when we get back to base
                    self.movement_phase = 0
                    self.current_waypoint_index = 0
            
            else:
                # Move toward the current target
                direction = np.arctan2(dy, dx)
                
                # Calculate new position
                move_distance = min(self.speed, distance)  # Don't overshoot
                self.x += move_distance * np.cos(direction)
                self.y += move_distance * np.sin(direction)
            
            # Update height based on terrain with minimum altitude
            terrain_height = self.environment.get_terrain_height(self.x, self.y)
            self.z = terrain_height + self.min_altitude
        
    def get_position(self):
        return (self.x, self.y, self.z)
    
    def get_movement_info(self):
        """Return information about the current movement phase"""
        phase_name = "Outward" if self.movement_phase == 0 else "Return"
        if self.current_waypoint_index < len(self.waypoints):
            if self.movement_phase == 0:
                target = self.waypoints[self.current_waypoint_index]
            else:
                reverse_index = self.outward_waypoints_count - 1 - self.current_waypoint_index
                target = self.waypoints[reverse_index]
            return f"{phase_name} - Target: ({target[0]:.1f}, {target[1]:.1f})"
        return f"{phase_name} - Completing journey"

class Drone:
    def __init__(self, id, grid_x, grid_y, color, map_size, formation, environment):
        self.id = id
        self.grid_x = grid_x  # Position in grid (0, 1, 2, 3)
        self.grid_y = grid_y  # Position in grid (0, 1, 2, 3)
        self.color = color
        self.map_size = map_size
        self.formation = formation
        self.environment = environment
        
        # Height adjustment parameters
        self.height_adjustment_rate = 0.2
        self.terrain_following_distance = 2.5  # Minimum distance from terrain
        
        # Base station coordinates
        self.base_station = (1, 1)
        
        # Start all drones at the base station
        self.x = self.base_station[0]
        self.y = self.base_station[1]
        self.z = self.environment.get_terrain_height(self.x, self.y) + 1 + (id * 0.2)  # Stacked at base
        
        # Previous position for smoother height transitions
        self.prev_x = self.x
        self.prev_y = self.y
        self.prev_z = self.z
        
        # Grid spacing for formation
        self.grid_spacing = 2  # Space between drones in the grid
        
        # Battery properties
        self.max_battery = 100
        
        # Persistent battery state
        battery_file = f"drone_{id}_battery.pkl"
        if os.path.exists(battery_file):
            try:
                with open(battery_file, 'rb') as f:
                    saved_data = pickle.load(f)
                    self.battery = saved_data['battery']
                    self.battery_drain_rate = saved_data['drain_rate']
                    # Ensure battery is not less than 50%
                    if self.battery < 50:
                        self.battery = random.uniform(50, 100)
            except:
                # If there's any error loading, create new values
                self.battery = random.uniform(50, 100)
                self.battery_drain_rate = random.uniform(0.005, 0.01)  # Much slower battery drain
        else:
            # First run, create new values
            self.battery = random.uniform(50, 100)
            self.battery_drain_rate = random.uniform(0.005, 0.01)  # Much slower battery drain
        
        # Save battery state to file
        self.save_battery_state()
            
        # Status
        # Status
        self.status = "waiting"  # waiting, patrolling, investigating, returning, charging, rejoining
        self.speed = 0.3  # Speed when returning to base or rejoining formation
        
        # POI distance tracking
        self.distance_to_poi = 0

        # Leader status
        self.is_leader = False
        self.score = 0  # Priority score
    
    def calculate_position(self):
        """Calculate actual position based on formation center and grid position"""
        formation_x, formation_y, formation_z = self.formation.get_position()
        
        # Calculate offset from formation center
        # Adjusting to make a proper square grid
        grid_center = 1.5  # Center of a 4x4 grid (0,1,2,3)
        offset_x = (self.grid_x - grid_center) * self.grid_spacing
        offset_y = (self.grid_y - grid_center) * self.grid_spacing
        
        # Calculate actual position
        actual_x = formation_x + offset_x
        actual_y = formation_y + offset_y
        
        # Get terrain height at this specific location
        terrain_height = self.environment.get_terrain_height(actual_x, actual_y)
        
        # Calculate height based on terrain with individual variations
        # Use the formation's height as a reference point
        formation_terrain_height = self.environment.get_terrain_height(formation_x, formation_y)
        terrain_difference = terrain_height - formation_terrain_height
        
        # Calculate the z position with terrain following
        # Base height + terrain difference + small random variation for natural look
        actual_z = formation_z + terrain_difference * 0.8 + random.uniform(-0.3, 0.3)
        
        # Ensure minimum safe height above terrain
        min_safe_height = terrain_height + self.terrain_following_distance
        if actual_z < min_safe_height:
            actual_z = min_safe_height
        
        return (actual_x, actual_y, actual_z)
    
    def save_battery_state(self):
        """Save battery state to a file"""
        battery_file = f"drone_{self.id}_battery.pkl"
        with open(battery_file, 'wb') as f:
            pickle.dump({
                'battery': self.battery, 
                'drain_rate': self.battery_drain_rate
            }, f)
    
    def update(self):
    # Save previous position for smooth transitions
        self.prev_x = self.x
        self.prev_y = self.y
        self.prev_z = self.z
    
    # Wait for formation to take off
        if self.status == "waiting" and self.formation.status == "moving":
            self.status = "patrolling"
    
    # Battery management
        if self.status == "patrolling":
            self.battery -= self.battery_drain_rate
        
        # Calculate position based on formation and terrain
            target_x, target_y, target_z = self.calculate_position()
        
        # Gradually move towards target position for smoother animation
            self.x = target_x
            self.y = target_y
        
        # Smooth height transition based on terrain
            height_diff = target_z - self.z
        # Apply gradual height adjustment
            if abs(height_diff) > 0.05:  # Only adjust if difference is significant
                self.z += np.sign(height_diff) * min(self.height_adjustment_rate, abs(height_diff))
            else:
                self.z = target_z
        
        # If battery low, return to base
            if self.battery < 20:
                self.status = "returning"

        elif self.status == "investigating":
            self.battery -= self.battery_drain_rate * 1.2  # Higher battery drain when investigating
    
    # Get POI position
            poi = points_of_interest[0]
    
    # Circle parameters
            circle_radius = 5.0  # Larger radius for a clearer circle
    
    # Arrange drones in a circle based on their ID
            total_drones_investigating = sum(1 for d in drones if d.status == "investigating")
    
    # Find this drone's position in the circle (based on investigating drones only)
            investigating_drones = [d.id for d in drones if d.status == "investigating"]
            if self.id in investigating_drones:
                position_index = investigating_drones.index(self.id)
                angle = (position_index / total_drones_investigating) * 2 * np.pi
        
        # Add time-based rotation to make the circle dynamic
                rotation_speed = 0.02  # Slower for more stable circle
                angle += rotation_speed * time.time()
        
        # Calculate target position on circle
                target_x = poi[0] + circle_radius * np.cos(angle)
                target_y = poi[1] + circle_radius * np.sin(angle)
        
        # Create altitude variation based on position in circle
        # This creates a wave pattern in height as drones circle
                height_variation = np.sin(angle) * 1.0
                base_height = poi[2] + 3.0  # Base hover height above POI
                target_z = base_height + height_variation
        
        # Ensure minimum safe height above terrain
                terrain_height = self.environment.get_terrain_height(target_x, target_y)
                safe_z = max(target_z, terrain_height + 2.5)
        
        # Move towards target position with smooth interpolation
                dx = target_x - self.x
                dy = target_y - self.y
                dz = safe_z - self.z
        
        # Use smoother movement with a fixed rate
                move_rate = 0.1  # Adjust for desired speed
                self.x += dx * move_rate
                self.y += dy * move_rate
                self.z += dz * move_rate
        else:
        # If somehow this drone is in investigating mode but not in the list
        # (shouldn't happen but just in case)
        # Calculate position based on formation and terrain as fallback
            target_x, target_y, target_z = self.calculate_position()
            self.x = target_x
            self.y = target_y
            self.z = target_z
    
    # If battery low, return to base
        if self.battery < 20:
            self.status = "returning"

        elif self.status == "returning":
        # Move towards base station
            dx = self.base_station[0] - self.x
            dy = self.base_station[1] - self.y
            distance_2d = np.sqrt(dx**2 + dy**2)
        
            if distance_2d < 0.5:  # At base station
                self.status = "charging"
                self.z = self.environment.get_terrain_height(self.base_station[0], self.base_station[1]) + 0.5
            else:
            # Move towards base station
                direction = np.arctan2(dy, dx)
                self.x += self.speed * np.cos(direction)
                self.y += self.speed * np.sin(direction)
            
            # Adjust height to follow terrain with minimum clearance
                terrain_height = self.environment.get_terrain_height(self.x, self.y)
                target_z = terrain_height + 2
            
            # Smooth height change with dynamic adjustment rate based on slope
                height_diff = target_z - self.z
                adjustment = min(0.2, abs(height_diff) * 0.3)  # Faster adjustment on steep slopes
            
                if abs(height_diff) > 0.05:  # Only adjust if difference is significant
                    self.z += np.sign(height_diff) * adjustment
                else:
                    self.z = target_z
            
                self.battery -= self.battery_drain_rate * 0.25  # Even less drain when returning
            
        elif self.status == "charging":
            self.battery += 0.3  # Charge rate
            if self.battery >= self.max_battery:
                self.battery = self.max_battery
                self.status = "rejoining"  # Change to rejoining instead of patrolling
            
        elif self.status == "rejoining":
        # Calculate target position in formation
            target_x, target_y, target_z = self.calculate_position()
        
        # Calculate distance to target position
            dx = target_x - self.x
            dy = target_y - self.y
            distance_2d = np.sqrt(dx**2 + dy**2)
        
            if distance_2d < 0.5:  # Close enough to position
                self.status = "patrolling"
            # Snap to correct position with small variation for natural look
                self.x = target_x + random.uniform(-0.1, 0.1)
                self.y = target_y + random.uniform(-0.1, 0.1)
                self.z = target_z
            else:
            # Move towards target position
                direction = np.arctan2(dy, dx)
                self.x += self.speed * np.cos(direction)
                self.y += self.speed * np.sin(direction)
            
            # Adjust height gradually towards target height
                height_diff = target_z - self.z
                adjustment = min(0.15, abs(height_diff) * 0.2)  # Smooth height transition
            
                if abs(height_diff) > 0.05:
                    self.z += np.sign(height_diff) * adjustment
                else:
                    self.z = target_z
            
            # Small battery drain during rejoining
            self.battery -= self.battery_drain_rate * 0.3
    
    # Save battery state after update
        self.save_battery_state()

    def calculate_distance_to_poi(self, poi):
        """Calculate 3D distance from drone to point of interest"""
        return np.sqrt((self.x - poi[0])**2 + (self.y - poi[1])**2 + (self.z - poi[2])**2)
                
    def get_battery_color(self):
        # Color gradient based on battery level
        if self.battery > 60:
            return "green"
        elif self.battery > 30:
            return "orange"
        else:
            return "red"
            
    def get_position(self):
        return (self.x, self.y, self.z)
    
    def get_status_display(self):
        status_text = f"Drone {self.id}: "
        if self.status == "waiting":
            status_text += "Waiting for takeoff"
        elif self.status == "patrolling":
            status_text += "Patrolling"
        elif self.status == "investigating":
            status_text += "Investigating POI"
        elif self.status == "returning":
            status_text += "Returning to base"
        elif self.status == "rejoining":
            status_text += "Rejoining formation"
        else:
            status_text += "Charging"
        status_text += f" - Battery: {self.battery:.1f}%"
        return status_text
    
    def calculate_priority_score(self):
        """Calculate priority score based on distance to POI and battery level
        Score = (100 - distance) + battery percentage
        """
        if self.status == "patrolling" or self.status == "investigating":
            distance_score = max(0, 100 - self.distance_to_poi)  # Ensure not negative
            self.score = distance_score + self.battery
        else:
        # Lower score for drones not actively patrolling or investigating
            self.score = self.battery
        return self.score

# Simulation parameters
MAP_SIZE = 20
NUM_POINTS_OF_INTEREST = 1  # Just one POI

# Create environment
environment = TerrainEnvironment(MAP_SIZE)

# Set up the database
setup_database()

# Create formation controller with environment
formation = DroneFormation(MAP_SIZE, environment)

# Create drones in a square grid formation
drones = []
colors = ['blue', 'green', 'red', 'purple', 'orange', 'cyan', 'magenta', 'yellow', 'teal', 'lime', 'pink', 'brown', 
          'navy', 'maroon', 'olive', 'darkgreen']  # Added more colors for 16 drones

# Create a 4x4 grid of drones (16 drones total)
grid_size = 4
for y in range(grid_size):
    for x in range(grid_size):
        drone_id = y * grid_size + x + 1
        drones.append(Drone(drone_id, x, y, colors[(drone_id-1) % len(colors)], MAP_SIZE, formation, environment))

# Create one point of interest
points_of_interest = []
x = random.uniform(MAP_SIZE/4, 3*MAP_SIZE/4)
y = random.uniform(MAP_SIZE/4, 3*MAP_SIZE/4)
points_of_interest.append((x, y, environment.get_terrain_height(x, y)))

# Set up the visualization - Create 2 subplots: 2D and 3D views
fig = plt.figure(figsize=(15, 8))

# 2D top view
ax1 = fig.add_subplot(1, 2, 1)
ax1.set_xlim(0, MAP_SIZE)
ax1.set_ylim(0, MAP_SIZE)
ax1.set_title('Mountain Patrol - Top View (16 Drones)')

# 3D view
ax2 = fig.add_subplot(1, 2, 2, projection='3d')
ax2.set_xlim(0, MAP_SIZE)
ax2.set_ylim(0, MAP_SIZE)
ax2.set_zlim(0, 8)  # Adjusted based on height map values
ax2.set_title('Mountain Patrol - 3D View (16 Drones)')

# Visualize terrain in 3D view
terrain_x = np.linspace(0, MAP_SIZE, environment.height_map.shape[0])
terrain_y = np.linspace(0, MAP_SIZE, environment.height_map.shape[1])
terrain_X, terrain_Y = np.meshgrid(terrain_x, terrain_y)
terrain_surface = ax2.plot_surface(
    terrain_X, terrain_Y, environment.height_map, 
    cmap=cm.terrain, alpha=0.6, linewidth=0, antialiased=True
)

# Add contour plot to 2D view to see terrain
contour = ax1.contour(
    terrain_X, terrain_Y, environment.height_map, 
    levels=10, cmap='terrain', alpha=0.6
)
ax1.clabel(contour, inline=1, fontsize=8)

# Add base station in corner
base_x, base_y = 1, 1
base_z = environment.get_terrain_height(base_x, base_y)
base_station_2d = plt.Circle((base_x, base_y), 1, color='gray', alpha=0.7)
ax1.add_patch(base_station_2d)
ax1.text(base_x, base_y, "BASE", ha='center', va='center', color='white')

# Add base station to 3D view
ax2.scatter([base_x], [base_y], [base_z], color='gray', s=100, depthshade=True)
ax2.text(base_x, base_y, base_z, "BASE", color='white')

# Plot the single point of interest in both views
poi = points_of_interest[0]
ax1.scatter(poi[0], poi[1], s=150, color='gold', marker='*', alpha=0.9)
ax1.text(poi[0], poi[1] + 0.8, "POI", ha='center', fontsize=9)
ax2.scatter([poi[0]], [poi[1]], [poi[2]], s=150, color='gold', marker='*', alpha=0.9)
ax2.text(poi[0], poi[1], poi[2] + 0.5, "POI", ha='center', fontsize=9)

# Set up drone plotting
drone_markers_2d = []
drone_markers_3d = []
battery_indicators = []
status_texts = []
grid_lines = []
trail_lines_3d = []  # Add trails to show vertical movement

# Add formation center marker 
formation_pos = formation.get_position()
formation_marker_2d = plt.Circle((formation_pos[0], formation_pos[1]), 0.2, color='black', alpha=0.2)
ax1.add_patch(formation_marker_2d)
formation_marker_3d = ax2.scatter([formation_pos[0]], [formation_pos[1]], [formation_pos[2]], 
                                color='black', s=20, alpha=0.2)

# Creating grid lines to show formation
for i in range(grid_size):
    h_line, = ax1.plot([0, 0], [0, 0], color='gray', linestyle='-', alpha=0.2)
    v_line, = ax1.plot([0, 0], [0, 0], color='gray', linestyle='-', alpha=0.2)
    grid_lines.append(h_line)
    grid_lines.append(v_line)

# Initialize drone markers
for drone in drones:
    pos = drone.get_position()
    # Create 2D drone marker
    marker_2d = plt.Circle((pos[0], pos[1]), 0.3, color=drone.color)
    ax1.add_patch(marker_2d)
    drone_markers_2d.append(marker_2d)
    
    # Create 3D drone marker
    marker_3d = ax2.scatter([pos[0]], [pos[1]], [pos[2]], color=drone.color, s=50)
    drone_markers_3d.append(marker_3d)
    
    # Create battery indicator (2D only)
    battery = plt.Circle((pos[0], pos[1] + 0.5), 0.15, color=drone.get_battery_color())
    ax1.add_patch(battery)
    battery_indicators.append(battery)
    
    # Add status text (2D only)
    status = ax1.text(pos[0], pos[1] - 0.5, f"{drone.id}", fontsize=7, ha='center')
    status_texts.append(status)
    
    # Add trail line for each drone to show height changes
    trail, = ax2.plot([pos[0], pos[0]], [pos[1], pos[1]], [0, pos[2]], 
                     color=drone.color, alpha=0.3, linestyle='--')
    trail_lines_3d.append(trail)

# Create a variable to track the formation's path
formation_path_x = [formation.x]
formation_path_y = [formation.y]
formation_path_line, = ax1.plot(formation_path_x, formation_path_y, 'k-', alpha=0.3)

# Add battery percentage information to sidebar
plt.figtext(0.02, 0.95, "Battery Indicators:", fontsize=9, 
            bbox=dict(facecolor='white', alpha=0.7))
plt.figtext(0.02, 0.93, "Green: >60%  |  Orange: 30-60%  |  Red: <30%", fontsize=8, 
            bbox=dict(facecolor='white', alpha=0.7))
plt.figtext(0.02, 0.91, "Drones return to base at <20% battery", fontsize=8, 
            bbox=dict(facecolor='white', alpha=0.7))

# Add terrain following indicator
plt.figtext(0.02, 0.88, "Grid formation follows terrain contours during patrol", fontsize=8,
            bbox=dict(facecolor='white', alpha=0.7))

# Update movement info text
plt.figtext(0.02, 0.85, "Structured pattern: Border → Inner squares → Peak → Return", fontsize=8,
            bbox=dict(facecolor='white', alpha=0.7))
plt.figtext(0.02, 0.82, "Formation follows terrain contours during patrol", fontsize=8,
            bbox=dict(facecolor='white', alpha=0.7))

# Add POI distance information
plt.figtext(0.02, 0.79, "POI Distance shown with battery levels", fontsize=8,
            bbox=dict(facecolor='white', alpha=0.7))

# Add drone status information
plt.figtext(0.02, 0.76, "After charging, drones rejoin formation", fontsize=8,
            bbox=dict(facecolor='white', alpha=0.7))

# Legend
ax1.scatter([], [], s=150, color='gold', marker='*', label='Point of Interest')
ax1.plot([], [], 'o', color='gray', markersize=10, label='Base Station')
ax1.plot([], [], 'o', color='black', markersize=6, alpha=0.2, label='Formation Center')

# Group drones by color in legend - adjusted for more drones
drones_per_row = 4  # Show fewer drones per row in legend to avoid overcrowding
for i in range(0, min(8, len(drones)), drones_per_row):  # Only show first 8 drones in legend to avoid clutter
    legend_drones = [ax1.plot([], [], 'o', color=drones[j].color, markersize=6, 
                    label=f'Drone {drones[j].id}')[0] for j in range(i, min(i+drones_per_row, min(8, len(drones))))]

ax1.legend(loc='upper right', fontsize=8, ncol=1)

# Function to update the simulation
def update(frame):
    # Get point of interest (POI) position
    poi = points_of_interest[0]  # We have only one POI in this simulation
    
    # Update formation position first
    formation.update()
    formation_pos = formation.get_position()
    
    # Track formation path - only keep last 100 positions to avoid clutter
    formation_path_x.append(formation_pos[0])
    formation_path_y.append(formation_pos[1])
    if len(formation_path_x) > 100:
        formation_path_x.pop(0)
        formation_path_y.pop(0)
    formation_path_line.set_data(formation_path_x, formation_path_y)
    
    # Update formation markers
    formation_marker_2d.center = (formation_pos[0], formation_pos[1])
    formation_marker_3d._offsets3d = ([formation_pos[0]], [formation_pos[1]], [formation_pos[2]])
    
    # Update grid lines
    grid_spacing = drones[0].grid_spacing
    grid_size = 4  # Keep at 4 for 4x4 grid
    center_offset = grid_spacing * 1.5  # Center of grid
    
    # Update horizontal grid lines
    for i in range(grid_size):
        y_pos = formation_pos[1] + (i - 1.5) * grid_spacing
        grid_lines[i*2].set_data(
            [formation_pos[0] - center_offset, formation_pos[0] + center_offset],
            [y_pos, y_pos]
        )
    
    # Update vertical grid lines
    for i in range(grid_size):
        x_pos = formation_pos[0] + (i - 1.5) * grid_spacing
        grid_lines[i*2+1].set_data(
            [x_pos, x_pos],
            [formation_pos[1] - center_offset, formation_pos[1] + center_offset]
        )
    
    # Clear previous status text
    plt.figtext(0.02, 0.02, "", fontsize=9)
    
    # Count drones by status
    status_counts = {"waiting": 0, "patrolling": 0, "investigating": 0, "returning": 0, "charging": 0, "rejoining": 0}
    
    # Initialize leader_index to -1 (no leader)
    leader_index = -1
    
    # Update each drone and calculate distance to POI
    for i, drone in enumerate(drones):
        # Ensure we have drone objects with proper IDs
        drone.update()
        pos = drone.get_position()
        
        # Update status count
        status_counts[drone.status] = status_counts.get(drone.status, 0) + 1
        
        # Calculate and update distance to POI
        drone.distance_to_poi = drone.calculate_distance_to_poi(poi)

        # Calculate priority score
        drone.calculate_priority_score()

        # Check if any drone is close to POI
        if drone.distance_to_poi < 4 and drone.status == "patrolling":
            # Change all patrolling drones to investigating
            for other_drone in drones:
                if other_drone.status == "patrolling":
                    other_drone.status = "investigating"
    
    # Only select a leader if at least one drone is investigating
    if any(d.status == "investigating" for d in drones):
        max_score = -1
        leader_index = -1
        
        # Only consider investigating drones for leadership
        for i, d in enumerate(drones):
            if d.status == "investigating" and d.score > max_score:
                max_score = d.score
                leader_index = i
        
        # Reset leader status for all drones
        for d in drones:
            d.is_leader = False
        
        # Set new leader if we found one among investigating drones
        if leader_index >= 0:
            drones[leader_index].is_leader = True
    else:
        # If no drones are investigating, ensure no drone is marked as leader
        for d in drones:
            d.is_leader = False

    # Update visual elements for all drones and log data to database
    for i, drone in enumerate(drones):
        pos = drone.get_position()
    
        # Update 2D drone position and size based on leader status
        drone_markers_2d[i].center = (pos[0], pos[1])
        if drone.is_leader:
            drone_markers_2d[i].radius = 0.5  # Make leader larger
        else:
            drone_markers_2d[i].radius = 0.3  # Normal size for other drones
    
        # Update 3D drone position and size
        drone_markers_3d[i]._offsets3d = ([pos[0]], [pos[1]], [pos[2]])
        if drone.is_leader:
            drone_markers_3d[i].set_sizes([100])  # Larger marker for leader
        else:
            drone_markers_3d[i].set_sizes([50])   # Normal size

        # Update trail line to show height
        terrain_height = environment.get_terrain_height(pos[0], pos[1])
        trail_lines_3d[i].set_data_3d(
            [pos[0], pos[0]], 
            [pos[1], pos[1]], 
            [terrain_height, pos[2]]
        )
    
        # Update battery indicator (2D only)
        battery_indicators[i].center = (pos[0], pos[1] + 0.5)
        battery_indicators[i].set_color(drone.get_battery_color())

        # Update status text (2D only)
        status_texts[i].set_position((pos[0], pos[1] - 0.5))
        if drone.is_leader:
            status_texts[i].set_text(f"{drone.id}★")  # Add star for leader
            status_texts[i].set_fontweight('bold')
        else:
            status_texts[i].set_text(f"{drone.id}")
            status_texts[i].set_fontweight('normal')
            
        # DATABASE LOGGING SECTION - Log each drone individually
        # Get target position based on drone status
        if drone.status == "investigating":
            target_x, target_y = poi[0], poi[1]
        elif drone.status == "returning":
            target_x, target_y = drone.base_station[0], drone.base_station[1]
        elif drone.status == "patrolling" or drone.status == "rejoining":
            target_pos = drone.calculate_position()
            target_x, target_y = target_pos[0], target_pos[1]
        else:  # "waiting" or "charging"
            target_x, target_y = drone.base_station[0], drone.base_station[1]
            
        # Determine role
        role = "leader" if drone.is_leader else "coordinator"
        
        # Log to database (only every 20 frames to avoid database overload)
        if frame % 20 == 0:
            # Log with explicit drone ID to ensure all drones are logged
            log_drone_data(drone.id, role, target_x, target_y)
            
    # Rest of your code...
    
    # Add status display with battery levels and POI distances
    status_lines = []
    # First half (drones 1-8)
    for i in range(0, 8):
        d = drones[i]
        status_line = f"D{d.id}{' ★' if d.is_leader else ''}: {d.battery:.0f}% - {d.status[:4]} - Dist: {d.distance_to_poi:.1f}"
        status_lines.append(status_line)
    
    # Second half (drones 9-16)
    second_half_lines = []
    for i in range(8, 16):
        d = drones[i]
        status_line = f"D{d.id}{' ★' if d.is_leader else ''}: {d.battery:.0f}% - {d.status[:4]} - Dist: {d.distance_to_poi:.1f}"
        second_half_lines.append(status_line)
    
    # Create two columns of status information
    combined_lines = []
    for i in range(8):
        combined_lines.append(f"{status_lines[i]:<35} | {second_half_lines[i]}")
    
    status_text = "\n".join(combined_lines)
    plt.figtext(0.55, 0.02, status_text, fontsize=7,
                bbox=dict(facecolor='white', alpha=0.7))
    
    # Show current formation position and movement info
    movement_info = formation.get_movement_info()
    plt.figtext(0.02, 0.73, f"Formation: ({formation_pos[0]:.1f}, {formation_pos[1]:.1f}) - {movement_info}", 
                fontsize=8, bbox=dict(facecolor='white', alpha=0.5))
    
    # Show POI position
    plt.figtext(0.02, 0.70, f"POI Location: ({poi[0]:.1f}, {poi[1]:.1f}, {poi[2]:.1f})",
                fontsize=8, bbox=dict(facecolor='white', alpha=0.5))
    
    # Show drone status summary
    status_summary = f"Patrolling: {status_counts['patrolling']} | Investigating: {status_counts['investigating']} | Charging: {status_counts['charging']} | " \
                 f"Returning: {status_counts['returning']} | Rejoining: {status_counts['rejoining']}"
    plt.figtext(0.02, 0.67, status_summary, 
                fontsize=8, bbox=dict(facecolor='white', alpha=0.5))
    # Show leader info
    if leader_index >= 0:
        leader = drones[leader_index]
        plt.figtext(0.02, 0.64, f"Leader: Drone {leader.id} - Score: {leader.score:.1f} - Status: {leader.status}",
                    fontsize=8, bbox=dict(facecolor='yellow', alpha=0.3))
    
    return [formation_marker_2d, formation_marker_3d, formation_path_line] + drone_markers_2d + drone_markers_3d + battery_indicators + status_texts + grid_lines + trail_lines_3d

# Create animation
ani = animation.FuncAnimation(fig, update, frames=100, interval=100, blit=False)

plt.tight_layout()
plt.show() # type: ignore