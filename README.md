# My "Happy" Package

# Intended Behavior

Run the client and server separately.

    ros2 run my_happy_package my_happy_server
    ros2 run my_happy_package my_happy_client

This gets output on the server side

    [INFO] [1621451424.663179579] [trajectory_action_server]: Server Ready
    [INFO] [1621451428.565478566] [trajectory_action_server]: Received goal request with 1 joints
    [INFO] [1621451428.566111553] [trajectory_action_server]: Executing goal
    [INFO] [1621451428.566476222] [trajectory_action_server]: Goal succeeded

And on the client side

    [INFO] [1621451428.564211692] [trajectory_action_client]: Sending goal
    [INFO] [1621451428.566249347] [trajectory_action_client]: Goal accepted by server, waiting for result
    [INFO] [1621451428.567517278] [trajectory_action_client]:

# Actual Behavior
If you run the server and client on separate machines, you get the following error on the server side

    [INFO] [1621451548.255623249] [trajectory_action_server]: Server Ready
    terminate called after throwing an instance of 'eprosima::fastcdr::exception::NotEnoughMemoryException'
    what():  Not enough memory in the buffer stream


The client does not report the goal being accepted.
