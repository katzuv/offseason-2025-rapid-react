package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.units.measure.Angle
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.toRotation2d
import frc.robot.lib.getPose3d
import frc.robot.lib.getRotation3d
import frc.robot.lib.getTranslation3d
import frc.robot.robotstate.COMMAND_NAME_PREFIX
import frc.robot.subsystems.drive.Drive
import frc.robot.subsystems.shooter.hood.Hood
import frc.robot.subsystems.shooter.turret.Turret
import frc.robot.subsystems.wrist.Wrist
import org.littletonrobotics.junction.Logger
import org.team5987.annotation.LoggedOutput

private val swerveModulePose: Array<Translation2d> =
    Drive.getModuleTranslations()

private val kWheelRadius = 0.0508.m

private fun getSwerveModulePoseTurn(
    moduleX: Double,
    moduleY: Double,
    moduleYaw: Angle
): Pose3d {
    return Pose3d(
        Translation3d(moduleX, moduleY, kWheelRadius[m]),
        getRotation3d(yaw = moduleYaw)
    )
}

private fun getSwerveModulePoseDrive(
    moduleX: Double,
    moduleY: Double,
    moduleYaw: Angle,
    modulePitch: Angle
): Pose3d {

    return Pose3d(
        Translation3d(moduleX, moduleY, kWheelRadius[m]),
        getRotation3d(yaw = moduleYaw, pitch = modulePitch)
    )
}

private fun getAllSwerveModulePoseTurn(): Array<Pose3d> {
    val swervePosesTurn: Array<Pose3d> =
        arrayOf(Pose3d(), Pose3d(), Pose3d(), Pose3d())
    for (i in 0..3) {
        swervePosesTurn[i] =
            getSwerveModulePoseTurn(
                swerveModulePose[i].x,
                swerveModulePose[i].y,
                drive.SwerveTurnAngle[i]
            )
    }
    return swervePosesTurn
}

private fun getAllSwerveModulePoseDrive(): Array<Pose3d> {
    val swervePosesDrive: Array<Pose3d> =
        arrayOf(Pose3d(), Pose3d(), Pose3d(), Pose3d())

    for (i in 0..3) {
        swervePosesDrive[i] =
            getSwerveModulePoseDrive(
                swerveModulePose[i].x,
                swerveModulePose[i].y,
                drive.SwerveTurnAngle[i],
                drive.SwerveDriveAngle[i]
            )
    }
    return swervePosesDrive
}

val wristTranslation
    get() = getTranslation3d(-0.3, 0.0, 0.28)
val wristRotation
    get() = getRotation3d(pitch = Wrist.inputs.position)
val wristPose
    get() = getPose3d(wristTranslation, wristRotation)

val rollerTranslation
    get() = getTranslation3d(-0.25, 0.0, 0.0)
val rollerRotation
    get() = getRotation3d(0.0)
val rollerPose
    get() = wristPose + Transform3d(rollerTranslation, rollerRotation)

val turretTranslation
    get() = getTranslation3d(z = 0.41)
val turretRotation
    get() = getRotation3d(yaw = Turret.inputs.position)
val turretPose
    get() =
        getPose3d(turretTranslation, turretRotation) +
            Transform3d(getTranslation3d(0.0), getRotation3d(yaw = -90.deg))

val hoodTranslation
    get() = getTranslation3d(z = 0.083, y = 0.151)
val hoodRotation
    get() = getRotation3d(roll = 50.deg + Hood.inputs.position)
val hoodPose
    get() = turretPose + Transform3d(hoodTranslation, hoodRotation)

val flywheelTranslation
    get() = getTranslation3d(0.0)
val flywheelRotation
    get() = getRotation3d(pitch = (-10).deg, yaw = 90.deg)
val flywheelPose
    get() = hoodPose + Transform3d(flywheelTranslation, flywheelRotation)

val subsystemPoseArray = Array(19) { getPose3d() }

fun getSubsystemPose(): Array<Pose3d> {
    val swerveModulesPoses = getAllSwerveModulePoseDrive()

    swerveModulesPoses.forEachIndexed { i, modulePose ->
        subsystemPoseArray[2 * i + 1] = modulePose
    }
    subsystemPoseArray[8] = wristPose
    subsystemPoseArray[9] = rollerPose
    subsystemPoseArray[15] = turretPose
    subsystemPoseArray[16] = flywheelPose
    subsystemPoseArray[17] = hoodPose

    return subsystemPoseArray
}

fun logSubsystemPose() {
    Logger.recordOutput("RobotPose3d", *getSubsystemPose())
}

@LoggedOutput(path = COMMAND_NAME_PREFIX)
val shootingDirection
    get() =
        Pose2d(
            drive.pose.translation,
            turretRotation.toRotation2d() + drive.pose.rotation
        )
