package frc.robot.robotstate

import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.applyLeds
import frc.robot.drive
import frc.robot.hopper
import frc.robot.lib.extensions.*
import frc.robot.lib.onTrue
import frc.robot.robotRelativeBallPoses
import frc.robot.roller
import frc.robot.turret
import org.team5987.annotation.LoggedOutput

@LoggedOutput(path = COMMAND_NAME_PREFIX)
val isInDeadZone = Trigger {
    val driveTranslation = drive.pose.translation
    !OUTER_SHOOTING_AREA.contains(driveTranslation) ||
            INNER_SHOOTING_AREA.contains(driveTranslation)
}

@LoggedOutput(path = COMMAND_NAME_PREFIX)
val atShootingRotation =
    turret.isAtSetpoint.and {
        drive.pose.rotation.measure.isNear(
            swerveCompensationAngle.measure,
            ROTATION_TOLERANCE
        )
    }

val isShooting = Trigger { state == RobotState.SHOOTING }
val isIntaking = Trigger { state == RobotState.INTAKING }
private val hasFrontBall = roller.hasBall
val hasBackBall = hopper.hasBall
private val ballsEmpty = hasFrontBall.or(hasBackBall).negate()

fun bindRobotCommands() {
    isShooting.apply {
        and(ballsEmpty).onTrue(setIntaking(), stopShooting())
        and(!isInDeadZone, atShootingRotation).onTrue(startShooting())
        and((isInDeadZone).or(!atShootingRotation))
            .onTrue(driveToShootingPoint())
    }
    isIntaking.apply {
        and(hasFrontBall, hasBackBall)
            .onTrue(roller.stop(), hopper.stop(), setShooting())
        and(hasBackBall, !hasFrontBall).apply {
            onTrue(stopIntaking())
            and(robotRelativeBallPoses::isNotEmpty).apply {
                onTrue(roller.intake())
                and(globalBallPoses::isNotEmpty).onTrue(alignToBall())
            }
        }
        and(ballsEmpty, robotRelativeBallPoses::isNotEmpty).apply {
            onTrue(roller.intake(), hopper.start())
            and(globalBallPoses::isNotEmpty).onTrue(alignToBall())
        }
    }
    applyLeds()
}

private fun setRobotState(newState: RobotState) =
    Commands.runOnce({ state = newState })

fun setShooting() = setRobotState(RobotState.SHOOTING)

fun setIntaking() = setRobotState(RobotState.INTAKING)
