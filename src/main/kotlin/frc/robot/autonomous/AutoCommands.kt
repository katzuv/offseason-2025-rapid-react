package frc.robot.autonomous

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.commands.PathPlannerAuto
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.roller.Roller
import frc.robot.subsystems.wrist.Wrist

internal fun runPath(name: String): Command {
    val path = PathPlannerPath.fromPathFile(name)
    var startPose = path.pathPoses[0]
    return AutoBuilder.resetOdom(startPose)
        .andThen(AutoBuilder.followPath(path))
}

fun A(): Command {
    return Roller.intake().andThen(Wrist.default())
}

fun Test2(): Command = PathPlannerAuto("Test2")

fun StartAuto(): Command = PathPlannerAuto("StartAuto")

