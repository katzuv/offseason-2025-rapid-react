package frc.robot.autonomous.paths.deploy.choreo

import choreo.Choreo
import choreo.auto.AutoFactory
import choreo.trajectory.SwerveSample
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.drive

private var startTime: Double? = null
private val field = Field2d()

private fun isRedAlliance(): Boolean {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
}

val autoFactory: AutoFactory =
    AutoFactory(
        { drive.pose },
        { pose -> drive.resetOdometry(pose) },
        { sample: SwerveSample -> drive.followTrajectory(sample) },
        true,
        drive
    )

fun mio(): Command {
    return autoFactory.trajectoryCmd("mio")
}

internal fun runChoreoPath(name: String): Command {
    val trajectory = Choreo.loadTrajectory(name)

    return Commands.sequence(
        Commands.runOnce({
            trajectory.ifPresent { t ->
                t.getInitialPose(isRedAlliance()).ifPresent { pose ->
                    drive.resetOdometry(pose)
                }
            }
            startTime = Timer.getFPGATimestamp()
        }),
        Commands.run(
            {
                trajectory.ifPresent { t ->
                    startTime?.let { st ->
                        val elapsed = Timer.getFPGATimestamp() - st
                        t.sampleAt(elapsed, isRedAlliance())
                            .ifPresentOrElse(
                                { s -> drive.runVelocity(s.chassisSpeeds) },
                                {
                                    drive.runVelocity(
                                        ChassisSpeeds(0.0, 0.0, 0.0)
                                    )
                                }
                            )
                    }
                }
            },
            drive
        )
    )
}

// fun mio(): Command = runChoreoPath("mio")

fun mio2(): Command = runChoreoPath("mio2")
