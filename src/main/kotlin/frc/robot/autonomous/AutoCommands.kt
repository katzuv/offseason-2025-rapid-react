package frc.robot.autonomous

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.commands.PathPlannerAuto
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.drive
import frc.robot.subsystems.roller.Roller
import frc.robot.subsystems.wrist.Wrist
import kotlin.random.Random

internal fun runPath(name: String): Command {
    val path = PathPlannerPath.fromPathFile(name)
    var startPose = path.pathPoses[0]
    return AutoBuilder.resetOdom(startPose)
        .andThen(AutoBuilder.followPath(path))
}

fun A(): Command {
    return Roller.intake().andThen(Wrist.default())
}
fun Test2(): Command= PathPlannerAuto("Test2")

//fun AC1(): Command = runPath("AC1")
//
//fun C1S(): Command = runPath("C1S")
//
//fun CC2(): Command = runPath("CC2")
//
//fun C2C3(): Command = runPath("C2C3")
//
//fun BRP2(): Command = runPath("BRP2")
//
//fun AC1SRP(): Command = Commands.sequence(AC1(), C1S())
//
//fun CC2C3(): Command = Commands.sequence(CC2(), C2C3())
