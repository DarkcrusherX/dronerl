from arm import armtakeoff
from gazebo_connection import GazeboConnection

ga= GazeboConnection()

hi = armtakeoff()
hi.arm()
ga.pauseSim()
ga.resetSim()
ga.unpauseSim()
hi.arm()