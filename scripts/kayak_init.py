#!/usr/bin/env python
import rospy
import time 
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

class OffboardNode:
    def __init__(self):

        rospy.init_node("offb_node_py")
        
        self.state_sub = rospy.Subscriber("mavros/state", State, self.state_cb)
        self.local_velo_pub= rospy.Publisher("mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)
        
        self.vel_sub = rospy.Subscriber("vehicle/cmd_vel", Twist, self.update_velocity)

        rospy.wait_for_service("/mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        self.velo = Twist()
        self.offb_set_mode = SetModeRequest()
        self.offb_set_mode.custom_mode = 'OFFBOARD'
        self.arm_cmd = CommandBoolRequest()
        self.arm_cmd.value = True
        

    def state_cb(self, msg):
        self.current_state = msg
    
    def update_velocity(self, vel=Twist):
    	self.velo = vel

    def send_initial_velocity(self):
        
        self.velo.linear.x = 0
        self.velo.linear.y = 0
        self.velo.linear.z = 0

        for i in range(100):   
            self.local_velo_pub.publish(self.velo)
            time.sleep(0.05)

        rospy.loginfo("Initial velocity published")
        self.set_mode_client.call(self.offb_set_mode)
        self.arming_client.call(self.arm_cmd)
        rospy.loginfo("Arming and offboard mode completed")

    def process(self):
    
        last_req = rospy.Time.now()

        if(self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(self.set_mode_client.call(self.offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()
        else:
            if(not self.current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(self.arming_client.call(self.arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")

                last_req = rospy.Time.now()

        self.local_velo_pub.publish(self.velo)

def disarming_and_hold_mode():
    
    #offb_set_mode = SetModeRequest()
    #offb_set_mode.custom_mode = 'HOLD'
    #type_service = SetMode
    #service_name = '/mavros/set_mode'
    #ans = call_service(service=service_name, type=type_service, val = offb_set_mode)
    
    value = False
    type_service = CommandBool
    service_name = '/mavros/cmd/arming'
    ans = call_service(service=service_name, type=type_service, val = value)
    
    

def call_service(service,type,val):
    rospy.wait_for_service(service)
    try:
        srv =  rospy.ServiceProxy(service, type)
        ans = srv(val)
        return ans
    except rospy.ServiceException as e:
         rospy.loginfo("Service call failed: %s"%e)

def main():
    offboard_mode = OffboardNode()
    offboard_mode.send_initial_velocity()
    rospy.on_shutdown(disarming_and_hold_mode)
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        offboard_mode.process()
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except IOError as e:
        print(e)
    
