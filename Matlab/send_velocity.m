clear all
% Create a publisher object for the /cmd_vel topic with Twist message type
twistpub = rospublisher('/vehicle/cmd_pos', 'geometry_msgs/PoseStamped');

% Create a message object for the Twist message type
twistmsg = rosmessage('geometry_msgs/PoseStamped');

% Set the linear and angular velocities in the Twist message
twistmsg.Pose.Position.X = 0;
twistmsg.Pose.Position.Y = 100;
twistmsg.Pose.Position.Z = 0;
twistmsg.Pose.Orientation.W = 0;

% Publish the Twist message to the /cmd_vel topic
send(twistpub, twistmsg);

