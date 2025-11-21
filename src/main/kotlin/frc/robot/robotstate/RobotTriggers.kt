package frc.robot.robotstate

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.RobotContainer.forceShoot
import frc.robot.RobotContainer.shouldShootOneBall
import frc.robot.applyLeds
import frc.robot.drive
import frc.robot.lib.extensions.and
import frc.robot.lib.extensions.onTrue
import frc.robot.lib.extensions.whileTrue
import frc.robot.lib.shooting.disableCompensation
import frc.robot.robotRelativeBallPoses
import frc.robot.subsystems.roller.Roller
import frc.robot.subsystems.shooter.flywheel.Flywheel
import frc.robot.subsystems.shooter.flywheel.STATIC_SHOOT_VELOCITY
import frc.robot.subsystems.shooter.hood.Hood
import frc.robot.subsystems.shooter.hood.STATIC_SHOOT_SETPOINT
import frc.robot.subsystems.shooter.hopper.Hopper
import org.team5987.annotation.LoggedOutput

@LoggedOutput(path = COMMAND_NAME_PREFIX)
val isInDeadZone = Trigger {
    val driveTranslation = drive.pose.translation
    !OUTER_SHOOTING_AREA.contains(driveTranslation) ||
        INNER_SHOOTING_AREA.contains(driveTranslation)
}

val isShooting = Trigger { state == RobotState.SHOOTING }
val isStaticShooting = Trigger { state == RobotState.FIXED_SHOOTING }
val isIntaking = Trigger { state == RobotState.INTAKING }
val isIdling = Trigger { state == RobotState.IDLING }

@LoggedOutput(path = COMMAND_NAME_PREFIX)
val isTurretInRange = Trigger { appliedTurretAngle == turretAngleToHub }

private val hasFrontBall = Roller.hasBall
val hasBackBall = Hopper.hasBall // TODO: CHANGE!
private val ballsEmpty = hasFrontBall.or(hasBackBall).negate()

val shouldShootOnMove = Trigger { !disableCompensation.get() }

private val isIntakeManual = Trigger { !intakeByVision }
private val isIntakeAuto = Trigger {
    intakeByVision && robotRelativeBallPoses.isNotEmpty()
}

fun bindRobotCommands() {
    isShooting.apply {
        and(ballsEmpty.and(forceShoot.negate())).onTrue(setIntaking())
        and(isInDeadZone.negate()).apply {
            and(shouldShootOnMove.negate().and(isTurretInRange))
                .onTrue(startShooting())
            and(shouldShootOnMove).onTrue(startShooting())
        }
        and(shouldShootOnMove.negate()).apply {
            and(isInDeadZone)
                .onTrue(alignToShootingPoint(deadZoneAlignmentSetpoint))
            and(isTurretInRange.negate()).onTrue(alignSwerveToHub())
        }
    }
    isIntaking.apply {
        and(hasFrontBall.or(shouldShootOneBall), hasBackBall)
            .onTrue(Roller.stop(), Hopper.stop())
            .apply {
                and(RobotModeTriggers.test())
                    .onTrue(setTestShooting())
                    .negate()
                    .onTrue(setShooting())
            }
        and(hasBackBall, hasFrontBall.negate()).apply {
            onTrue(Hopper.slowBack(), Roller.intake())
            and(robotRelativeBallPoses::isNotEmpty, { intakeByVision }).apply {
                and(forceShoot.negate())
                    .onTrue(alignToBall(disableAutoAlign::get))
            }
        }
        and(hasBackBall.negate(), hasFrontBall).apply {
            onTrue(Hopper.startIntake(), Roller.intake())
        }
        and(ballsEmpty).apply {
            onTrue(stopIntaking())

            and(isIntakeAuto).apply {
                onTrue(
                    Roller.intake(),
                    Hopper.startIntake(),
                    alignToBall(disableAutoAlign::get)
                )
            }
            and(isIntakeManual).onTrue(Roller.intake(), Hopper.startIntake())
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
    isIdling.apply { onTrue(stopAll().ignoringDisable(true)) }
    applyLeds()
}

val isDisabled =
    Trigger { DriverStation.isDisabled() }
        .onTrue(setIdling().ignoringDisable(true))

private fun setRobotState(newState: RobotState) =
    Commands.runOnce({ state = newState })

fun setShooting() = setRobotState(RobotState.SHOOTING)

fun setIntaking() = setRobotState(RobotState.INTAKING)

fun setIdling() = setRobotState(RobotState.IDLING)

fun setStaticShooting() = setRobotState(RobotState.FIXED_SHOOTING)
