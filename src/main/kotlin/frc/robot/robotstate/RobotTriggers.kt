package frc.robot.robotstate

import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.applyLeds
import frc.robot.drive
import frc.robot.lib.IS_ENABLED
import frc.robot.lib.extensions.*
import frc.robot.lib.shooting.disableCompensation
import frc.robot.robotRelativeBallPoses
import frc.robot.subsystems.roller.Roller
import frc.robot.subsystems.shooter.flywheel.Flywheel
import frc.robot.subsystems.shooter.flywheel.STATIC_SHOOT_VELOCITY
import frc.robot.subsystems.shooter.hood.Hood
import frc.robot.subsystems.shooter.hood.STATIC_SHOOT_SETPOINT
import frc.robot.subsystems.shooter.hopper.Hopper
import frc.robot.subsystems.shooter.turret.Turret
import org.team5987.annotation.LoggedOutput

@LoggedOutput(path = COMMAND_NAME_PREFIX)
val isInDeadZone = Trigger {
    val driveTranslation = drive.pose.translation
    !OUTER_SHOOTING_AREA.contains(driveTranslation) ||
        INNER_SHOOTING_AREA.contains(driveTranslation)
}

// TODO: Check this!!
@LoggedOutput(path = COMMAND_NAME_PREFIX)
val atShootingRotation =
    Turret.isAtSetpoint.and(
        Trigger {
                drive.pose.rotation.measure.isNear(
                    swerveCompensationAngle.measure,
                    ROTATION_TOLERANCE
                )
            }
            .or { disableAutoAlign.get() }
    )

val isShooting = Trigger { state == RobotState.SHOOTING }
val isStaticShooting = Trigger { state == RobotState.FIXED_SHOOTING }
val isIntaking = Trigger { state == RobotState.INTAKING }
val isIdling = Trigger { state == RobotState.IDLING }
private val hasFrontBall = Roller.hasBall
val hasBackBall = Hopper.hasBall
private val ballsEmpty = hasFrontBall.or(hasBackBall).negate()

fun bindRobotCommands() {
    isShooting.apply {
        and(ballsEmpty.and { !forceShoot }).onTrue(setIntaking())
        and(!isInDeadZone).apply {
            val shouldShootOnMove = Trigger { !disableCompensation.get() }
            and(atShootingRotation, shouldShootOnMove.negate())
                .onTrue(startShooting())
            and(shouldShootOnMove).onTrue(startShooting())
        }
        and((isInDeadZone).or(!atShootingRotation))
            .onTrue(driveToShootingPoint(disableAutoAlign::get))
    }
    isIntaking.apply {
        and(hasFrontBall, hasBackBall)
            .onTrue(Roller.stop(), Hopper.stop(), setShooting())
        and(hasBackBall, !hasFrontBall).apply {
            onTrue(Hopper.stop())
            and(robotRelativeBallPoses::isNotEmpty, { intakeByVision }).apply {
                and { !forceShoot }.onTrue(alignToBall(disableAutoAlign::get))
            }
            onTrue(Roller.intake())
        }
        and(!hasBackBall, hasFrontBall).apply {
            onTrue(Hopper.startIntake(), Roller.intake())
        }
        and(ballsEmpty).apply {
            and(robotRelativeBallPoses::isNotEmpty, { intakeByVision }).apply {
                onTrue(
                    Roller.intake(),
                    Hopper.startIntake(),
                    alignToBall(disableAutoAlign::get)
                )
            }
            onTrue(stopIntaking())
            and { !intakeByVision }
                .onTrue(Roller.intake(), Hopper.startIntake())
        }
        onTrue(stopShooting())
    }
    isStaticShooting.apply {
        onTrue(Roller.intake(), Hopper.startShoot())
        whileTrue(
            Hood.setAngle { STATIC_SHOOT_SETPOINT },
            Flywheel.setVelocity { STATIC_SHOOT_VELOCITY }
        )
    }
    isIdling.apply { onTrue(Roller.stop(), Hopper.stop(), Flywheel.stop()) }
    applyLeds()
}

val isDisabled = Trigger { !IS_ENABLED }.onTrue(setIdling())

private fun setRobotState(newState: RobotState) =
    Commands.runOnce({ state = newState })

fun setShooting() = setRobotState(RobotState.SHOOTING)

fun setIntaking() = setRobotState(RobotState.INTAKING)

fun setIdling() = setRobotState(RobotState.IDLING)

fun setStaticShooting() = setRobotState(RobotState.FIXED_SHOOTING)
