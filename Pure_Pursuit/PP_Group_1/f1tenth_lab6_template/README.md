# Pure Pursuit Controller (Group-1)

### Contributers
- Aryaan Sinha
- Harsh Vibhor Sharma
- Utsab Karan

## Simulation Environement Setup
- It is important to setup the simulation env. Use the following links.
- Link 1
- doc link 

## Available Controllers
- **Pure Pursuit V1**
    - run the node with the respective name
    - This controller has a fixed initial velocity of 5. I has a rectification algorithm which correctly reduces the speed during turns for smoother results. It uses a fixed Look-Ahead Distance.
    - It runs around the map for 1 loop after which is wobbles and crashes. Refer to V2 for the looped version.
    - It uses a 14k waypoint set and does not use any velocity profile data set for each of the waypoints.
    - The more you try to increase the speed, the lower you must set the loop delay(ros2 timer object). This is one of the problems we face in the code.
    - At very high speeds it becomes impossible for correct turns whichout crashes regardless of the timer delay.

