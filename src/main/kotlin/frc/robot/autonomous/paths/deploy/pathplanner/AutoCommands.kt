package frc.robot.autonomous.paths.deploy.pathplanner

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.commands.PathPlannerAuto
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command

internal fun runPath(name: String): Command {
    val path = PathPlannerPath.fromPathFile(name)
    val startPose = path.pathPoses[0]
    return AutoBuilder.resetOdom(startPose)
        .andThen(AutoBuilder.followPath(path))
}

fun ExamplePath(): Command = PathPlannerAuto("ExamplePath")
fun AutoTest(): Command= PathPlannerAuto("AutoTest")
fun BackAutoTest(): Command= PathPlannerAuto("BackAutoTest")
