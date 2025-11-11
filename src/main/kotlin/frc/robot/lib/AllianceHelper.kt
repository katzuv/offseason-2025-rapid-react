package frc.robot.lib

import edu.wpi.first.wpilibj.DriverStation
import org.team5987.annotation.LoggedOutput

@LoggedOutput
val IS_RED: Boolean
    get() =
        DriverStation.getAlliance().isPresent &&
            DriverStation.getAlliance().get() == DriverStation.Alliance.Red

@LoggedOutput
val IS_ENABLED: Boolean
    get() = DriverStation.isEnabled()
