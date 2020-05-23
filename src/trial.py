from arm import armtakeoff
from gazebo_connection import GazeboConnection
import rospy

rospy.init_node('arm_and_takeoff_node', anonymous=True)
ga= GazeboConnection()

hi = armtakeoff()


hi.arm()
print("arm completed")
hi.disarm()
print("disarmed")
ga.pauseSim()
ga.resetSim()
ga.unpauseSim()
print("byes")