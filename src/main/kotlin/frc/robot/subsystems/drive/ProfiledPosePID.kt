package frc.robot.subsystems.drive

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.TunableGains
import frc.robot.lib.extensions.deg_ps
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.rad_ps
import frc.robot.lib.gains
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber
import org.team5987.annotation.LoggedOutput

private const val LOGGING_PREFIX = "AutoAlignment"

private val linearMaxVelocityTunable =
    LoggedNetworkNumber("$LOGGING_PREFIX/linearMaxVelocity", 4.69)
private val linearMaxAccelerationTunable =
    LoggedNetworkNumber("$LOGGING_PREFIX/linearMaxAcceleration", 2.8)

private val xGains =
    TunableGains(
        "x Gains",
        kP = 4.0,
        cruiseVelocity = linearMaxVelocityTunable,
        acceleration = linearMaxAccelerationTunable
    )
private val yGains =
    TunableGains(
        "y Gains",
        cruiseVelocity = linearMaxVelocityTunable,
        acceleration = linearMaxAccelerationTunable
    )
private val thetaGains =
    TunableGains(
        "ϴ Gains",
        kP = 6.0,
        cruiseVelocity = 7.deg_ps[rad_ps],
        acceleration = 360.deg_ps[rad_ps]
    )

@LoggedOutput("X controller", LOGGING_PREFIX)
var xController = xGains.profiledPIDController

@LoggedOutput("Y controller", LOGGING_PREFIX)
var yController = yGains.profiledPIDController

@LoggedOutput("ϴ controller", LOGGING_PREFIX)
var thetaController =
    thetaGains.profiledPIDController.apply {
        enableContinuousInput(-Math.PI, Math.PI)
    }

@LoggedOutput(path = LOGGING_PREFIX)
var atGoal: Trigger =
    Trigger(xController::atGoal)
        .and(yController::atGoal)
        .and(thetaController::atGoal)

fun updateProfiledPIDGains() {
    mapOf(
            xController to xGains,
            yController to yGains,
            thetaController to thetaGains
        )
        .forEach { (controller, gains) -> controller.gains = gains }
}

fun setGoal(desiredPose: Pose2d) {
    updateProfiledPIDGains()
    xController.setGoal(desiredPose.x)
    yController.setGoal(desiredPose.y)
    thetaController.setGoal(desiredPose.rotation.radians)
}

fun resetProfiledPID(botPose: Pose2d, botSpeeds: ChassisSpeeds) {
    xController.reset(botPose.x, botSpeeds.vxMetersPerSecond)
    yController.reset(botPose.y, botSpeeds.vyMetersPerSecond)
    thetaController.reset(
        botPose.rotation.radians,
        botSpeeds.omegaRadiansPerSecond
    )
}

fun setTolerance(pose2d: Pose2d) {
    xController.setTolerance(pose2d.x)
    yController.setTolerance(pose2d.y)
    thetaController.setTolerance(pose2d.rotation.radians)
}

/**
 * Returns field relative chassis speeds to the selected goal.
 * @botPose the current pose of the robot
 */
fun getSpeedSetpoint(botPose: Pose2d): () -> ChassisSpeeds = {
    ChassisSpeeds(
        xController.calculate(botPose.x),
        yController.calculate(botPose.y),
        thetaController.calculate(botPose.rotation.radians)
    )
}
