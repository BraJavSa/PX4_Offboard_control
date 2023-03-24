% Create a publisher object for the /cmd_vel topic with Twist message type
pospub = rospublisher('/vehicle/cmd_pos', 'geometry_msgs/PoseStamped');

% Create a message object for the Twist message type
position = rosmessage('geometry_msgs/PoseStamped');

% Set the linear and angular velocities in the Twist message
position.pose.position.x = 100;
position.pose.position.y = 0;
position.pose.position.z = 0;
position.pose.orientation.w = -0.99;

% Publish the Twist message to the /cmd_vel topic
send(pospub, position);