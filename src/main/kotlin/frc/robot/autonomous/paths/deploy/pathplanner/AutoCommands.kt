package frc.robot.autonomous.paths.deploy.pathplanner

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.drive
import frc.robot.lib.IS_RED

internal fun runPath(name: String, mirror: Boolean = false): Command {
    val originalPath = PathPlannerPath.fromPathFile(name)
    val path = if (IS_RED) originalPath.flipPath() else originalPath
    var startPose = path.pathPoses[0]
    return drive.defer {
        Commands.runOnce({
                drive.resetOdometry(startPose)
                AutoBuilder.resetOdom(startPose)
            })
            .andThen(AutoBuilder.followPath(path))
    }
}

fun AC1(): Command = runPath("AC1")

fun C1S(): Command = runPath("C1S")

fun CC2(): Command = runPath("CC2")

fun C2C3(): Command = runPath("C2C3")

fun BRP2(): Command = runPath("BRP2")

fun AC1SRP(): Command = Commands.sequence(AC1(), C1S())

fun CC2C3(): Command = Commands.sequence(CC2(), C2C3())
