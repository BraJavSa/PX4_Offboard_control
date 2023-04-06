%% create path planning
clear all
first_way=[0:5:100;zeros(1,21)];
second_way=[100:-5:0; 10*ones(1,21)];
block_way=[first_way,[100;5],second_way,[0;15]];
full_way=block_way;
for i=1:4
    full_way=[full_way,[block_way(1,:);block_way(2,:)+i*20]];
end
full_way=[full_way,[first_way(1,:);first_way(2,:)+100]];
plot(full_way(1,:),full_way(2,:),'-*')
hold on
count=1;
next_point=[full_way(1,count),full_way(2,count)];
twistpub = rospublisher('/vehicle/cmd_pos', 'geometry_msgs/PoseStamped');
twistmsg = rosmessage('geometry_msgs/PoseStamped');
twistmsg.Pose.Position.X = full_way(1,count);
twistmsg.Pose.Position.Y = full_way(2,count);
twistmsg.Pose.Position.Z = 0;
send(twistpub, twistmsg);
real_way=[0;0];
 while count<=100
     pose = rostopic("echo", '/mavros/local_position/pose'); 
     mappos = [pose.Pose.Position.X, pose.Pose.Position.Y];
     dif=norm(next_point-mappos);
     real_way=[real_way,mappos'];
     if dif<=2.5
         count=count+1;
         next_point=[full_way(1,count),full_way(2,count)]
         twistmsg.Pose.Position.X = full_way(1,count);
         twistmsg.Pose.Position.Y = full_way(2,count);
         send(twistpub, twistmsg);
     end
 end
 plot(real_way(1,:),real_way(2,:),'red')