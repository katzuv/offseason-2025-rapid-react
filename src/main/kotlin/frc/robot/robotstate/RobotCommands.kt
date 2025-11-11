package frc.robot.robotstate

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.parallel
import edu.wpi.first.wpilibj2.command.Commands.sequence
import edu.wpi.first.wpilibj2.command.Commands.waitUntil
import frc.robot.*
import frc.robot.lib.extensions.*
import frc.robot.lib.math.interpolation.InterpolatingDouble
import frc.robot.lib.named
import frc.robot.lib.shooting.ShotData
import frc.robot.lib.shooting.calculateShot
import frc.robot.lib.shooting.disableCompensation
import frc.robot.lib.wrapAround
import frc.robot.robotRelativeBallPoses
import frc.robot.subsystems.drive.alignToPose
import frc.robot.subsystems.roller.Roller
import frc.robot.subsystems.shooter.flywheel.*
import frc.robot.subsystems.shooter.hood.HOOD_ANGLE_BY_DISTANCE
import frc.robot.subsystems.shooter.hood.Hood
import frc.robot.subsystems.shooter.hopper.Hopper
import frc.robot.subsystems.shooter.turret.SOFTWARE_LIMIT_CONFIG
import kotlin.collections.map
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean
import org.team5987.annotation.LoggedOutput

var hoodAngle = InterpolatingDouble(robotDistanceFromHub[m])
@LoggedOutput var forceShoot = false
var disableAutoAlign = LoggedNetworkBoolean("/Tuning/disableAutoAlign", true)
var intakeByVision = false
val compensatedShot: ShotData
    get() {
        val robotSpeeds =
            ChassisSpeeds.fromRobotRelativeSpeeds(
                drive.chassisSpeeds,
                drive.rotation
            )
        val shooterExitVelocity =
            Flywheel.velocity.toLinear(FLYWHEEL_DIAMETER, 1.0)
        val shot = calculateShot(drive.pose, robotSpeeds, shooterExitVelocity)

        mapOf(
                "compensatedShot/compensatedTarget" to
                    Pose2d(shot.compensatedTarget, Rotation2d()),
                "regularShot/target" to Pose2d(HUB_LOCATION, Rotation2d()),
                "compensatedShot/compensatedDistance" to
                    shot.compensatedDistance,
                "regularShot/distance" to robotDistanceFromHub,
                "compensatedShot/turretAngle" to shot.turretAngle.measure,
                "regularShot/turretAngle" to angleFromRobotToHub,
                "shooterExitVelocity" to shooterExitVelocity
            )
            .log("$COMMAND_NAME_PREFIX/onMoveShoot")

        return shot
    }

@LoggedOutput(path = COMMAND_NAME_PREFIX)
val robotDistanceFromHub
    get() = drive.pose.distanceFromPoint(HUB_LOCATION)

@LoggedOutput(path = COMMAND_NAME_PREFIX)
val angleFromRobotToHub
    get() = (drive.pose.translation.rotationToPoint(HUB_LOCATION))

@LoggedOutput(path = COMMAND_NAME_PREFIX)
val turretToRobotHubAngle
    get() = -angleFromRobotToHub + Rotation2d.k180deg + drive.pose.rotation

@LoggedOutput
val turretAngleToHub: Angle
    get() =
        (if (!disableCompensation.get()) {
                compensatedShot.turretAngle.measure
            } else turretToRobotHubAngle.measure)
            .wrapAround(
                SOFTWARE_LIMIT_CONFIG.ReverseSoftLimitThreshold.rot,
                SOFTWARE_LIMIT_CONFIG.ForwardSoftLimitThreshold.rot
            )

@LoggedOutput(path = COMMAND_NAME_PREFIX)
val turretToHub: Pose2d
    get() = Pose2d(drive.pose.translation, Rotation2d(turretAngleToHub))

@LoggedOutput(path = COMMAND_NAME_PREFIX)
val robotToHub: Pose2d
    get() = Pose2d(drive.pose.translation, angleFromRobotToHub)

@LoggedOutput val hub = Pose2d(HUB_LOCATION, Rotation2d())

@LoggedOutput(path = COMMAND_NAME_PREFIX)
val swerveCompensationAngle
    get() = drive.rotation + angleFromRobotToHub - Rotation2d(turretAngleToHub)

@LoggedOutput(path = COMMAND_NAME_PREFIX)
val globalBallPoses
    get() =
        robotRelativeBallPoses
            .map { it + Pose3d(drive.pose).toTransform() }
            .toTypedArray()

@LoggedOutput(path = COMMAND_NAME_PREFIX)
val deadZoneAlignmentSetpoint
    get() =
        (if (
                INNER_SHOOTING_AREA.getDistance(drive.pose.translation) <
                    OUTER_SHOOTING_AREA.getDistance(drive.pose.translation)
            )
                INNER_SHOOTING_AREA_ALIGNMENT
            else OUTER_SHOOTING_AREA_ALIGNMENT) // Find area for shooting.
            .nearest(drive.pose.translation)

fun setForceShoot() = Commands.runOnce({ forceShoot = true })

fun stopForceShoot() = Commands.runOnce({ forceShoot = false })

fun disableAutoAlign() = Commands.runOnce({ disableAutoAlign.set(true) })

fun enableAutoAlign() = Commands.runOnce({ disableAutoAlign.set(false) })

fun stopIntakeByVision() = Commands.runOnce({ intakeByVision = false })

fun setIntakeByVision() = Commands.runOnce({ intakeByVision = true })

fun driveToShootingPoint(toRun: () -> Boolean = { false }): Command =
    drive
        .defer {
            alignToPose(
                Pose2d(deadZoneAlignmentSetpoint, swerveCompensationAngle)
            )
        }
        .until(toRun)
        .named("Drive")

fun startShooting() =
    sequence(
            drive.lock(),
            Flywheel.setVelocity {
                    FLYWHEEL_VELOCITY_KEY.value = robotDistanceFromHub[m]
                    SHOOTER_VELOCITY_BY_DISTANCE.getInterpolated(
                            FLYWHEEL_VELOCITY_KEY
                        )
                        .value
                        .rps
                }
                .alongWith(
                    sequence(
                        waitUntil(Flywheel.isAtSetVelocity),
                        parallel(Hopper.startShoot(), Roller.intake())
                    )
                ),
        )
        .named(COMMAND_NAME_PREFIX)

fun stopShooting() =
    parallel(Flywheel.stop(), Hopper.stop()).named(COMMAND_NAME_PREFIX)

fun stopIntaking() =
    parallel(Roller.stop(), Hopper.stop()).named(COMMAND_NAME_PREFIX)

fun alignToBall(toRun: () -> Boolean = { false }): Command =
    drive
        .defer {
            alignToPose(globalBallPoses.first().toPose2d())
                .onlyIf(globalBallPoses::isNotEmpty)
        }
        .until(toRun)
        .named(COMMAND_NAME_PREFIX)

fun hoodDefaultCommand() =
    Hood.setAngle {
        hoodAngle.value = compensatedShot.compensatedDistance[m]
        HOOD_ANGLE_BY_DISTANCE.getInterpolated(hoodAngle).value.deg
    }
