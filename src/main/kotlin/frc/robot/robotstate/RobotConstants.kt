package frc.robot.robotstate

import edu.wpi.first.math.geometry.Ellipse2d
import edu.wpi.first.math.geometry.Translation2d
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.m
import frc.robot.lib.math.interpolation.InterpolatingDouble

val HUB_LOCATION = Translation2d(8.2.m, 4.1.m)
const val COMMAND_NAME_PREFIX = "RobotState"

val OUTER_SHOOTING_AREA =
    Ellipse2d(HUB_LOCATION, 3.5.m) // the outer part of the shooting area
val INNER_SHOOTING_AREA =
    Ellipse2d(HUB_LOCATION, 1.3.m) // the inner part of the shooting area

val FLYWHEEL_VELOCITY_KEY = InterpolatingDouble(robotDistanceFromHub[m])
