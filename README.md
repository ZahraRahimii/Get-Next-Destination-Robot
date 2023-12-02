# Get Next Destination Robot

To achieve this purpose we need to define two nodes:

## Mission Node: 
This node should respond to the GetNextDestination service, so that for each request, two random numbers in the range of -10 to 10 will be sent as x and y for the next destination.

## Control Node:
This node is responsible for controlling the robot, which is done by a PID controller.
