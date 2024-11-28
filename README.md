ROS2 SHEEP-WOLF-HUNTER SIMULATION

# ROS2 Package: ros2_sheep, ROS2 Workspace: ros2_hunter, 

## Description
This package implements a simulation of three nodes namely sheep_node, wolf-node and hunter_node in turtlesim environment. 
The specific behaviors of each nodes(sheep, wolf and hunter) are:
1. The sheep moves randomly with a low velocity.
2. The wolf avoids the hunter while trying to reach the sheep.
3. The hunter chases the wolf.

The simulation ensures that:
- From the random locations all the nodes are started.
- For identification purposes, different pen colors are used i.e., Red for Sheep_node, Green for wolf_node and Blue for hunter_node.
- Nodes are not allowed to hit walls or boundaries or go out of range or vibrate or overlap with each other.
- Both events (wolf reaching the sheep and hunter catching the wolf) occur.

## Implementation Details
### Sheep Node
- Moves randomly within the boundaries of the environment at a low velocity.
- Avoids clamping to the wall using boundary-checking logic.

### Wolf Node
- By calculating the relative position, the wolf node chases the sheep node and moves towards it.
- Wolf node avoids the hunter and calculates it relative position wiht respect to hunter and moves away. 

### Hunter Node
- By calculating the relative position of wolf, hunter node chases the wolf node.

### Statistics
The solution was tested 10 times

### Instructions to Run
1. Source your ROS2 environment:
   
   source /opt/ros/humble/setup.bash
   source ~/ros2_hunter/install/setup.bash
2. Launch the nodes:

   ros2 run ros2_wolf sheep_node
   ros2 run ros2_wolf wolf_node
   ros2 run ros2_wolf hunter_node

3. Observe the behavior in the turtlesim window.

4. Logs are generated in the terminal to track events i.e., Launch the nodes:

   ros2 run ros2_wolf sheep_node
   ros2 run ros2_wolf wolf_node
   ros2 run ros2_wolf hunter_node

6. Observe the behavior in the turtlesim window.

7. Logs are generated in the terminal to track events ie.,hunter catching the wolf, wolf reaching the sheep.
