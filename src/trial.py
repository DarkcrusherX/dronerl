from arm import armtakeoff
from gazebo_connection import GazeboConnection

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