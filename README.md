# dronerl

A trainning code to generate a neural network that helps the drone to successfully travel to a setpoint.

In this code the rewarding system is now completely based on distance from setpoint.

### src

[arm.py](https://github.com/DarkcrusherX/dronerl/blob/master/src/arm.py) : Function to disarm and arm and takeoff the drone.

[gazebo_connection.py](https://github.com/DarkcrusherX/dronerl/blob/master/src/gazebo_connection.py) : Functions to pause, unpause and reset the position of the model.

[myquadcopter_env.py](https://github.com/DarkcrusherX/dronerl/blob/master/src/myquadcopter_env.py) : Defining the environment.Final setpoint,number of steps in each episode,reward system,input to the Neural network(Observation = State), resetting the environment.

[rlgame.py](https://github.com/DarkcrusherX/dronerl/blob/master/src/rlgame.py) : Defining the neural network, processing observation/state (Q-value), Choosing action.

### Installations

[Install sitl with gazebo](https://github.com/Aeroclub-IITM/Installation-SITL-Gazebo-ROS)

pip install gym

pip install tensorflow

pip install tensorflow-gpu

pip install mitdeeplearning

pip install ipython

### Launch

Launch px4 sitl with gazebo( Can use ardupilot after changing OFFBOARD to GUIDED in arm.py), mavros and python rlgame.py.

Disable preflight checks before running rlgame.py.
