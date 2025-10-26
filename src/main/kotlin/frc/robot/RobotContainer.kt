package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.autonomous.paths.deploy.pathplanner.AC1SRP
import frc.robot.autonomous.paths.deploy.pathplanner.BRP2
import frc.robot.autonomous.paths.deploy.pathplanner.CC2C3
import frc.robot.lib.extensions.enableAutoLogOutputFor
import frc.robot.lib.extensions.sec
import frc.robot.lib.extensions.volts
import frc.robot.lib.shooting.toggleCompensation
import frc.robot.lib.sysid.sysId
import frc.robot.robotstate.*
import frc.robot.subsystems.drive.DriveCommands
import frc.robot.subsystems.roller.Roller
import frc.robot.subsystems.shooter.hood.Hood
import frc.robot.subsystems.shooter.turret.Turret
import frc.robot.subsystems.wrist.Wrist
import org.ironmaple.simulation.SimulatedArena
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser

object RobotContainer {

    private val driverController = CommandPS5Controller(0)
    private val switchController = CommandGenericHID(1)
    private val autoChooser: LoggedDashboardChooser<Command>

    enum class SwitchInput(val buttonId: Int) {
        DisableAutoAlign(0),
        StaticSetpoint(1),
        IntakeByVision(2)
    }

    init {
        drive // Ensure Drive is initialized
        autoChooser =
            LoggedDashboardChooser(
                "Auto Choices",
                AutoBuilder.buildAutoChooser()
            )
        registerAutoCommands()
        configureButtonBindings()
        configureDefaultCommands()
        bindRobotCommands()

        if (CURRENT_MODE == Mode.SIM) {
            SimulatedArena.getInstance().resetFieldForAuto()
        }

        enableAutoLogOutputFor(this)
    }

    @AutoLogOutput(key = "MapleSimPose")
    private fun getMapleSimPose(): Pose2d? =
        driveSimulation?.simulatedDriveTrainPose

    private fun configureDefaultCommands() {
        drive.defaultCommand =
            DriveCommands.joystickDrive(
                { driverController.leftY },
                { driverController.leftX },
                { -driverController.rightX * 0.8 }
            )
//        Turret.defaultCommand = Turret.setAngle { turretAngleToHub }
        Hood.defaultCommand = hoodDefaultCommand()
        Wrist.defaultCommand = Wrist.open()
    }

    private fun configureButtonBindings() {
        // reset swerve
        driverController.apply {
            options()
                .onTrue(
                    drive.runOnce { drive.resetGyro() }.ignoringDisable(true),
                )

            circle().onTrue(setIntaking())
            L2().onTrue(Roller.intake()).onFalse(Roller.stop())
            R2().onTrue(Roller.outtake()).onFalse(Roller.stop())
            square().onTrue(setIntaking())
            cross().onTrue(setShooting())

            povDown().onTrue(setIdling())
            povUp().onTrue(toggleCompensation())
            triangle().onTrue(setForceShoot()).onFalse(stopForceShoot())

            create().whileTrue(Wrist.reset())
        }

        SwitchController.apply {
            button(SwitchInput.DisableAutoAlign.buttonId)
                .whileTrue(disableAutoAlign())
                .whileFalse(enableAutoAlign())

            button(SwitchInput.StaticSetpoint.buttonId)
                .whileTrue(setStaticShooting())
                .whileFalse(setShooting())

            button(SwitchInput.IntakeByVision.buttonId)
                .whileTrue(setIntakeByVision())
                .onFalse(stopIntakeByVision())
        }
        // Reset gyro / odometry
        val resetOdometry =
            if (CURRENT_MODE == Mode.SIM)
                Runnable {
                    drive.resetOdometry(
                        driveSimulation!!.simulatedDriveTrainPose
                    )
                }
            else
                Runnable {
                    drive.resetOdometry(
                        Pose2d(drive.pose.translation, Rotation2d())
                    )
                }
    }

    fun getAutonomousCommand(): Command = autoChooser.get()

    private fun registerAutoCommands() {
        val namedCommands: Map<String, Command> = mapOf()

        NamedCommands.registerCommands(namedCommands)

        // Set up SysId routines
        autoChooser.addOption(
            "Drive Wheel Radius Characterization",
            DriveCommands.wheelRadiusCharacterization()
        )
        autoChooser.addOption(
            "Drive Simple FF Characterization",
            DriveCommands.feedforwardCharacterization()
        )
        autoChooser.addOption(
            "Drive SysId (Quasistatic Forward)",
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        )
        autoChooser.addOption(
            "Drive SysId (Quasistatic Reverse)",
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        )
        autoChooser.addOption(
            "Drive SysId (Dynamic Forward)",
            drive.sysIdDynamic(SysIdRoutine.Direction.kForward)
        )
        autoChooser.addOption(
            "Drive SysId (Dynamic Reverse)",
            drive.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        )

        autoChooser.addOption(
            "swerveFFCharacterization",
            DriveCommands.feedforwardCharacterization()
        )

        autoChooser.addDefaultOption("BRP2", BRP2())
        autoChooser.addOption("AC1SRP", AC1SRP())
        autoChooser.addOption("CC2C3", CC2C3())
        autoChooser.addOption(
            "hoodSysId",
            Hood.sysId()
                .withForwardRoutineConfig(1.8.volts.per(sec), 1.volts, 0.75.sec)
                .withBackwardRoutineConfig(
                    1.volts.per(sec),
                    0.8.volts,
                    0.75.sec
                )
                .command()
        )
    }

    fun resetSimulationField() {
        if (CURRENT_MODE != Mode.SIM) return

        drive.resetOdometry(Pose2d(3.0, 3.0, Rotation2d()))
        SimulatedArena.getInstance().resetFieldForAuto()
    }
}
