package frc.robot

import frc.robot.lib.Mode
import org.littletonrobotics.junction.LoggedRobot
import org.team5987.annotation.LoggedOutput

const val LOOP_TIME = 0.02 // [s]

@LoggedOutput
val CURRENT_MODE: Mode
    get() =
        if (LoggedRobot.isReal()) {
            Mode.REAL
        } else {
            if (System.getenv("isReplay") == "true") {
                Mode.REPLAY
            } else {
                Mode.SIM
            }
        }
