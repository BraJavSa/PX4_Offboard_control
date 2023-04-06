
while true
pose = rostopic("echo", '/mavros/local_position/pose'); 
mappos = [pose.Pose.Position.X, pose.Pose.Position.Y]
end
