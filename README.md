# rtt-core-extensions
RTT Extensions for robot control (incl. kinematic chain concepts)


```bash
import("rtt-core-extensions")
loadComponent("fullArmJa","cogimon::RTTKinematicChainJa")
setActivity("fullArmJa",0.05,50,ORO_SCHED_OTHER)
fullArmJa.configureUserside(7, 7)
fullArmJa.addPortRobotside("wholeArm_cmd_out", 7)
fullArmJa.configure()
fullArmJa.start()


# connect ports
var ConnPolicy cp;

# connect proxy output to gazebo position command input
connect("fullArmJa.wholeArm_cmd_out", "lwr_gazebo.JointPositionCtrl", cp)
# connect complete feedback
# TODO

stream("fullArmJa.command_in", rsb.transport.socket.scope("/my/input"))
```
