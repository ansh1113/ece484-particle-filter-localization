import numpy as np
from maze import Maze, Particle, Robot
import bisect
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetEntityState, SetEntityState
import shutil
from std_msgs.msg import Float32MultiArray
from scipy.integrate import ode
import math
import random
import matplotlib.pyplot as plt

def vehicle_dynamics(t, vars, vr, delta):
    curr_x = vars[0]
    curr_y = vars[1] 
    curr_theta = vars[2]
    
    dx = vr * np.cos(curr_theta)
    dy = vr * np.sin(curr_theta)
    dtheta = delta
    return [dx,dy,dtheta]

class ParticleFilter:
    def __init__(self, bob, world, num_particles, sensor_limit, x_start, y_start, node):
        self.num_particles = num_particles  # The number of particles for the particle filter
        self.sensor_limit = sensor_limit    # The sensor limit of the sensor
        self.node = node                    # ROS 2 node for communication
        particles = []
        self.gps_reading = None

        for _ in range(num_particles):
            x = np.random.uniform(0, world.width)
            y = np.random.uniform(0, world.height)
            particles.append(Particle(x=x, y=y, maze=world, sensor_limit=sensor_limit))

        self.particles = particles          # Randomly assign particles at the begining
        self.bob = bob                      # The estimated robot state
        self.world = world                  # The map of the maze
        self.x_start = x_start              # The starting position of the map in the gazebo simulator
        self.y_start = y_start              # The starting position of the map in the gazebo simulator
        self.set_entity_state_client = self.node.create_client(SetEntityState, '/set_entity_state')
        self.controlSub = self.node.create_subscription(Float32MultiArray, "/gem/control", self.__controlHandler, 10)
        self.get_model_state_client = self.node.create_client(GetEntityState, '/get_entity_state')
        self.control = []                   # A list of control signal from the vehicle
        self.step_history = []              # History of steps for plotting
        self.error_history = []              # History of errors for plotting

    def __controlHandler(self,data):
        """
        Description:
            Subscriber callback for /gem/control. Store control input from gem controller to be used in particleMotionModel.
        """
        tmp = list(data.data)
        self.control.append(tmp)

    def getModelState(self):
        """
        Description:
            Requests the current state of the polaris model when called
        Returns:
            modelState: contains the current model state of the polaris vehicle in gazebo
        """

        while not self.get_model_state_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Service not available, waiting again...')
            
        request = GetEntityState.Request()
        request.name = 'gem'
        
        try:
            future = self.get_model_state_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future)
            response = future.result()
            return response
        except Exception as e:
            self.node.get_logger().error(f"Service call failed: {e}")
            return None

    def updateWeight(self, lidar_readings):
        if lidar_readings is None:
            return
        
        #### TODO ####
        # Update the weight of each particle according to some function
        # (perhaps a gaussian kernel) that computes the score for each
        # particles' lidar measurement vs the lidar measurement from the robot.
        #
        # Make sure that the sum of all particle weights adds up to 1
        # after updating the weights.

        sigma = 1.5
        for particle in self.particles:
            particle_lidar = particle.read_sensor()
            squared_diff = 0
            for i in range(len(lidar_readings)):
                if particle_lidar[i] is not None and lidar_readings[i] is not None:
                    squared_diff += (particle_lidar[i] - lidar_readings[i]) ** 2
            particle.weight = np.exp(-squared_diff / (2 * sigma ** 2))
        total_weight = sum([p.weight for p in self.particles])

        if total_weight > 0:
            for particle in self.particles:
                particle.weight /= total_weight
        else:
            # If all weights are zero, assign uniform weights
            uniform_weight = 1.0 / len(self.particles)
            for particle in self.particles:
                particle.weight = uniform_weight

        #raise NotImplementedError("implement this!!!")


        #### END ####

    def resampleParticle(self):
        new_particles = []

        #### TODO ####
        # Resample current particles to generate a new set of particles.
        #
        # Things to consider:
        #   -   Resample particles based on the weight of each particle
        #
        #   -   If all the particles bunch up, then we will be stuck in a
        #       non-optimal solution. We can't be too certain! How can we
        #       mitigate this?
        #
        #   -   For problem [10] bonus points use gps measurements to get 
        #       a "rough" estimate of where you are. How can we use this
        #       to make sure the particle filter does not converge / stay
        #       in a non-optimal solution??
        #
        #       gps_x       = self.gps_reading[0]
        #       gps_y       = self.gps_reading[1]
        #       gps_heading = self.gps_reading[2]
        weights = [p.weight for p in self.particles]
        cumulative_weights = []
        cumsum = 0
        for w in weights:
            cumsum += w
            cumulative_weights.append(cumsum)

        num_resampled = int(0.7 * self.num_particles)  # 70% resampled
        num_random = self.num_particles - num_resampled  # 30% random/GPS-based

        for _ in range(num_resampled):
            rand = random.uniform(0, cumulative_weights[-1])
            index = bisect.bisect_left(cumulative_weights, rand)
            if index >= len(self.particles):
                index = len(self.particles) - 1
            selected_particle = self.particles[index]
            new_particle = Particle(
                x=selected_particle.x, 
                y=selected_particle.y,
                maze=self.world,
                sensor_limit=self.sensor_limit,
                heading=selected_particle.heading
            )
            new_particle.x += np.random.normal(0, 0.03)
            new_particle.y += np.random.normal(0, 0.03)
            new_particle.heading += np.random.normal(0, 0.0005)
            new_particles.append(new_particle)
            
            
        for i in range(num_random):
            
            if self.gps_reading is not None and np.random.random() < 0.8:
                # 80% of random particles use GPS guidance
                gps_x = self.gps_reading[0]
                gps_y = self.gps_reading[1]
                gps_heading = self.gps_reading[2]

                gps_noise_x = np.random.normal(0, 0.02)
                gps_noise_y = np.random.normal(0, 0.02)
                gps_noise_heading = np.random.normal(0, 0.0005)
                
                new_particle = Particle(
                    x=gps_x + gps_noise_x,
                    y=gps_y + gps_noise_y,
                    maze=self.world,
                    sensor_limit=self.sensor_limit,
                    heading=gps_heading + gps_noise_heading
                )
            
            else:
                new_particle = Particle(
                    x=np.random.uniform(0, self.world.width),
                    y=np.random.uniform(0, self.world.height),
                    maze=self.world,
                    sensor_limit=self.sensor_limit
                )
            
            new_particles.append(new_particle)
            
        
        
        
        # raise NotImplementedError("implement this!!!")


        #### END ####

        self.particles = new_particles

    def particleMotionModel(self):
        dt = 0.05   # might need adjusting depending on compute performance

        #### TODO ####
        # Estimate next state for each particle according to the control
        # input from the actual robot.
        # 
        # You can use an ODE function or the vehicle_dynamics function
        # provided at the top of this file.
        for vr, delta in self.control:
            for particle in self.particles:
                # Current state of the particle
                curr_state = [particle.x, particle.y, particle.heading]
                solver = ode(vehicle_dynamics)
                solver.set_integrator('dopri5')
                solver.set_initial_value(curr_state, 0)
                solver.set_f_params(vr, delta)
                new_state = solver.integrate(dt)
                particle.x = new_state[0]
                particle.y = new_state[1]
                particle.heading = new_state[2]
            
                noise_x = np.random.normal(0, 0.02)      # noise in x
                noise_y = np.random.normal(0, 0.02)      # noise in y
                noise_theta = np.random.normal(0, 0.005)   # noise in heading
                
                particle.x += noise_x
                particle.y += noise_y
                particle.heading += noise_theta
                
                # Normalize heading to [-pi, pi]
                particle.heading = np.arctan2(np.sin(particle.heading), np.cos(particle.heading))
        
        # Retrieve the control inputs from self.control (velocity vr and steering angle delta)
        # For each particle, use the vehicle_dynamics function or ODE solver to update its position
        # Add noise to account for motion uncertainty (particles shouldn't move exactly the same way)
        
        # raise NotImplementedError("implement this!!!")

        #### END ####

        self.control = []

    def runFilter(self, show_frequency):
        """
        Description:
            Run PF localization
        """
        self.world.clear_objects()
        self.world.show_particles(self.particles, show_frequency=show_frequency)
        self.world.show_robot(self.bob)
        count = 0 
    
        try:
            while rclpy.ok():
                lidar_reading, gps_reading = self.bob.read_sensor()

                # ensure at least one positive gps reading before running the filter
                if gps_reading is not None:
                    self.gps_reading = gps_reading
                if self.gps_reading is None:
                    continue

                # if no control inputs have arrived, do nothing
                if len(self.control) == 0:
                    continue

                #### TODO ####
                # 1. perform a particle motion step
                # 2. update weights based on measurements
                # 3. resample particles
                #
                # Hint: use class helper functions
                
                self.particleMotionModel()
                self.updateWeight(lidar_reading)
                self.resampleParticle()

                #### END ####

                if count % 2 == 0:
                    #### TODO ####
                    # Re-render world, make sure to clear previous objects first!
                    self.world.clear_objects()
                    self.world.show_particles(self.particles, show_frequency=show_frequency)
                    self.world.show_robot(self.bob)

                    #### END ####

                    estimated_location = self.world.show_estimated_location(self.particles)
                    err = math.sqrt((estimated_location[0] - self.bob.x) ** 2 + (estimated_location[1] - self.bob.y) ** 2)
                    print(f":: step {count} :: err {err:.3f}")
                    
                    # Store data for plotting
                    self.step_history.append(count)
                    self.error_history.append(err)
                    
                count += 1
        
        except KeyboardInterrupt:
            print("\n\nStopping particle filter and generating plot...")
        finally:
            # Plot at the end - this will ALWAYS run even if Ctrl+C is pressed
            self.plot_results()

    def plot_results(self):
        """Plot the error history"""
        if len(self.step_history) == 0:
            print("No data to plot!")
            return
            
        plt.figure(figsize=(10, 6))
        plt.plot(self.step_history, self.error_history, 'b-', linewidth=2, label='Localization Error')
        plt.xlabel('Step', fontsize=12)
        plt.ylabel('Localization Error (units)', fontsize=12)
        plt.title('Particle Filter Localization Error over Time', fontsize=14)
        plt.grid(True, alpha=0.3)
        plt.legend()
        
        # Save the plot
        plt.savefig('particle_filter_error.png', dpi=300, bbox_inches='tight')
        print(f"\n✓ Plot saved to particle_filter_error.png")
        print(f"✓ Total steps: {len(self.step_history)}")
        print(f"✓ Final error: {self.error_history[-1]:.3f}")
        print(f"✓ Steps with error < 10: {np.where(np.array(self.error_history) < 10.0)[0].shape[0]}")
        print(f"percentage = ",f"{(np.where(np.array(self.error_history) < 10.0)[0].shape[0] / len(self.error_history)) * 100:.2f}%")
        print(f"✓ Average error: {np.mean(self.error_history):.3f}")
        
        # Show the plot
        plt.show()