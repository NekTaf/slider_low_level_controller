# slider_low_level_controller

# Description 
Converts continuous body frame thrust and torque commands to discrete PWM thrust pulses 

Contents: 
- thrust_to_pwm.py -> converts (3,) force vector to (8,) thrust vector for each on-board thruster
- write_to_arduino(for real robot deployment) -> subscribes to (8,) thrust vector and writes data to Arduino board with specified baud rate 


# Slider Platform 

SSH to the Slider platform and run:

```bash 
ros2 launch slider_low_level_controller write_to_arduino.launch.py
```

this reads the ```\eight_thrust_pulse``` topic and send the serial commands to the on-board Arduino board 


