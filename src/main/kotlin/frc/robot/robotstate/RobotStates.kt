package frc.robot.robotstate

import org.team5987.annotation.LoggedOutput

enum class RobotState() {
    IDLING,
    INTAKING,
    SHOOTING
}

@LoggedOutput(path = COMMAND_NAME_PREFIX)
var state = RobotState.IDLING
