GNC System Architecture

Mission: From any given starting orbit, and a target space station, plan and maneuver a docking.

Core Components Hierarchy:
1. Mission State Machine (1 Hz)
   - Top level decision making
   - Phase management
   - Interruption handling
   
2. Planners (10 Hz)
   - Orbit Transfer Planner
   - Proximity Operations Planner
   
3. Translation Controllers (50 Hz)
   - Burn Translation Controller
   - Proximity Translation Controller
   - Near Field Translation Controller
   
4. Attitude Controller (100+ Hz)
   - Singleton
   - Always running
   - Final actuator authority

External Systems (considered separate):
1. State Estimation System
   - Maintains all state estimates
   - Handles sensor fusion
   - Manages transform tree between frames
   
2. Health Monitoring (Mission Executive)
   - Basic system health
   - Abort conditions
   - Sensor/actuator status

Hardware Configuration:
1. Actuators:
   - Main Engine: Large burns, fixed direction
   - RCS Thrusters: Small translation/attitude
   - Reaction Wheels: Primary attitude control
   
2. Sensors (handled by state estimation):
   - IMU
   - Star trackers
   - GPS (when available)
   - Relative navigation sensors
   
Data Flow:
Mission SM -> Planner -> Translation Controller -> Attitude Controller -> Actuators

Specific Flows for mission sequence:
>10km    : Mission SM -> Orbital Transfer Planner -> Burn Translation Controller -> Attitude Controller
30m-10km :            -> Proximity Planner -> Proximity Translation Controller -> Attitude Controller
<30m     :            -> Proximity Planner -> Near Field Translation Controller -> Attitude Controller

Each level runs independently at specified rate using latest commands from above

Reference Frames:
1. ECI (Earth-Centered Inertial)
2. LVLH (Local Vertical Local Horizontal)
3. Body Frame
4. Various sensor/actuator frames


Mission State Machine (1 Hz)
States:
1. ORBIT_TRANSFER
   Entry: Target orbit provided
   Exit: Within 10km of target
   Interrupts: 
   - Collision risk detected
   - Critical system status
   - Momentum dump needed (when safe)

2. PROXIMITY_OPERATIONS
   Entry: ~10km from target
   Exit: Within 30m of target
   Interrupts:
   - Safety envelope violation
   - Approach corridor deviation
   - Momentum dump needed (when safe)

3. DOCKING
   Entry: ~30m from target
   Exit: Docking complete
   Interrupts:
   - Approach corridor violation
   - Closing rate violation
   - Final approach abort

4. ABORT
   Entry: Any critical violation
   Actions: Execute abort procedure based on phase
   Exit: Safe state achieved

Orbit Transfer Planner (10 Hz)
States and Algorithms:
1. PLANNING
   - Convert states to orbital elements
   - Calculate required changes:
     * Δa (semi-major axis)
     * Δe (eccentricity)
     * Δi (inclination)
     * ΔΩ (RAAN)
     * Δω (argument of perigee)
   
2. BURN_SEQUENCE
   - For plane changes:
     * Calculate optimal burn points
     * dv = 2v*sin(Δi/2) at nodes
   
   - For Hohmann transfers:
     * dv1 = sqrt(μ/r1) * (sqrt(2r2/(r1+r2)) - 1)
     * dv2 = sqrt(μ/r2) * (1 - sqrt(2r1/(r1+r2)))
     * Transfer time = π*sqrt((r1+r2)³/8μ)
   
   - For phasing:
     * Calculate phase angle difference
     * Adjust orbit period for catch-up/fall-back
     * dv calculations similar to Hohmann

3. EXECUTION
   - Monitor burn execution
   - Update remaining burns
   - Check achievement of targets


Proximity Operations Planner (10 Hz)
Primary Function:
- Takes target position
- Generates appropriate trajectories
- Selects appropriate controller

States and Algorithms:
1. TRAJECTORY_PLANNING
   Input: Target position (from Mission Planner)
   Process:
   - Determine if target is in proximity or near-field regime
   - Generate appropriate trajectory:
     * Proximity regime (>30m):
       - Uses CW equations for prediction
       - Generates trajectory accounting for orbital mechanics
       - Ensures trajectory ends where Near Field controller can take over
     * Near field regime (<30m):
       - Direct path planning
       - Considers approach corridor
   
   Output: 
   - Trajectory segments
   - Controller selection (Proximity or Near Field)

2. EXECUTION_MONITORING
   - Tracks progress along trajectory
   - Verifies conditions for controller switching
   - Monitors safety constraints
   - Can add waypoints for:
     * Obstacle avoidance
     * Safety concerns
     * Hold points if needed

Safety Considerations:
- Verify entire trajectory before execution
- Ensure clean transition between controllers
- Maintain abort paths
- Check approach corridors


Translation Controllers (50 Hz):

1. Burn Translation Controller
Purpose: Execute large orbital maneuvers using main engine
Inputs: 
- Burn sequence (delta-v vector and timing)
- Current state
Outputs:
- Thrust command (main engine)
- Desired attitude (for pointing main engine)

Algorithm:
- Align spacecraft for burn
- Monitor burn execution and timing
- No closed loop control on translation
- Just burn execution and timing
- Pure open loop commands based on burn plan

2. Proximity Translation Controller
Purpose: Execute relative motion trajectories using RCS
Inputs:
- Desired trajectory in LVLH frame
- Current state (position/velocity in LVLH)
Outputs:
- RCS thrust commands
- Desired attitude

Algorithm:
```python
class ProximityTranslationController:
    """
    Purpose: Execute relative motion trajectories using RCS
    Update Rate: 50 Hz
    
    Gain Scheduling:
    - Gains increase as range decreases
    - Smooth transition to prevent discontinuities
    """
    def compute_gains(self, range_to_target):
        """
        Smooth gain scheduling based on range
        """
        # Example scheduling function
        # Increases gains smoothly as we get closer
        gain_scale = 1.0 + 0.5*np.exp(-range_to_target/1000)
        
        # Base gains scaled by range
        self.Kp = self.base_Kp * gain_scale
        self.Kd = self.base_Kd * gain_scale
        
    def compute_control(self, state, reference):
        # Update gains first
        range_to_target = np.linalg.norm(state[0:3])
        self.compute_gains(range_to_target)
        
        # CW-based control with scheduled gains
        n = self.orbital_rate
        
        # State in LVLH
        pos = state[0:3]
        vel = state[3:6]
        
        # Errors
        pos_error = reference.position - pos
        vel_error = reference.velocity - vel
        
        # Feedback with scheduled gains
        feedback = (self.Kp @ pos_error + 
                   self.Kd @ vel_error)
        
        # Orbital mechanics compensation
        orbital_correction = np.array([
            -2 * n * vel[1],
            0,
            0
        ])
        
        return feedback + orbital_correction

    def verify_stability(self, range_to_target):
        """
        Verify stability criteria:
        1. Gains vary smoothly
        2. Rate of gain change bounded
        3. System remains stable across gain range
        """
        gains = self.compute_gains(range_to_target)
        gain_rate = self.compute_gain_rate(range_to_target)
        
        return (self.verify_gain_smoothness(gains) and
                self.verify_gain_rate_bounds(gain_rate))
```

3. Near Field Translation Controller
Purpose: Direct position control for docking using RCS
Inputs:
- Desired position/velocity
- Current state
Outputs:
- RCS thrust commands
- Desired attitude

Algorithm:
```python
def compute_control(self, state, reference):
    # Simple PD control
    pos_error = reference.position - state.position
    vel_error = reference.velocity - state.velocity
    
    return self.Kp @ pos_error + self.Kd @ vel_error
```

Attitude Controller (100+ Hz):
Purpose: Maintain attitude control throughout all phases
State: Singleton, always running
Inputs:
- Desired attitude (from Translation Controller)
- Current attitude state
Outputs:
- Reaction wheel commands
- RCS commands when needed

Algorithm:
```python
def compute_control(self, state, desired_attitude):
    # Convert to quaternion error
    q_error = quaternion_multiply(
        desired_attitude,
        quaternion_inverse(state.attitude)
    )
    
    # Angular velocity error
    w_error = desired_attitude.angular_velocity - state.angular_velocity
    
    if self.wheels_saturated():
        # Use RCS for attitude control
        return self.compute_rcs_control(q_error, w_error)
    else:
        # Normal wheel control
        # Convert errors to torque commands
        torque = (self.Kp @ quaternion_to_rotation(q_error) + 
                 self.Kd @ w_error)
        return self.wheels_to_torque(torque)

def wheels_saturated(self):
    return np.any(np.abs(self.wheel_speeds) > WHEEL_SPEED_LIMIT)
```

Key Controller Properties:
1. Translation Controllers
   - Each designed for specific regime
   - Clear handoff conditions
   - Appropriate control laws for physics

2. Attitude Controller
   - Always maintains control authority
   - Handles actuator management
   - Reports status up chain
"""
