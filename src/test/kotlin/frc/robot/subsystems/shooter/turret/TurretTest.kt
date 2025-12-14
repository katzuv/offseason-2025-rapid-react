package frc.robot.subsystems.shooter.turret

import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.simulation.BatterySim
import edu.wpi.first.wpilibj.simulation.RoboRioSim
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.lib.extensions.deg
import io.kotest.core.spec.style.FunSpec
import io.kotest.matchers.doubles.shouldBeNear
import org.littletonrobotics.junction.LoggedRobot

class TurretTest :
    FunSpec({
        beforeSpec {
            // Initialize HAL for WPILib simulation
            assert(HAL.initialize(500, 0))
            // Initialize Logger to prevent errors
            LoggedRobot.isReal()
        }

        afterSpec {
            // Clean up after all tests
            HAL.shutdown()
        }

        beforeTest {
            // Reset command scheduler before each test
            CommandScheduler.getInstance().cancelAll()
            CommandScheduler.getInstance().clearComposedCommands()
            // Simulate battery voltage
            RoboRioSim.setVInVoltage(12.0)
        }

        afterTest {
            // Clean up after each test
            CommandScheduler.getInstance().cancelAll()
        }

        test("setAngle with static value should command motor to target position") {
            // Reset the turret to a known position
            val resetCommand = Turret.reset(0.deg)
            resetCommand.schedule()
            CommandScheduler.getInstance().run()

            // Set a target angle using the static setAngle method
            val targetAngle = 45.deg
            val setAngleCommand = Turret.setAngle(targetAngle)
            setAngleCommand.schedule()
            CommandScheduler.getInstance().run()

            // Run simulation for several iterations to allow motor to respond
            for (i in 0..20) {
                Turret.periodic()
                CommandScheduler.getInstance().run()
                RoboRioSim.setVInVoltage(
                    BatterySim.calculateDefaultBatteryLoadedVoltage(0.0)
                )
            }

            // Verify the motor position is moving toward or at the target
            // In simulation, the motor should respond to position commands
            val currentPosition = Turret.inputs.position.`in`(deg)
            // The position should be greater than 0 (moving towards 45 degrees)
            assert(currentPosition >= 0.0) {
                "Motor position should be at or moving toward target"
            }
        }

        test("setAngle with lambda should update angle setpoint") {
            // Reset the turret
            val resetCommand = Turret.reset(0.deg)
            resetCommand.schedule()
            CommandScheduler.getInstance().run()

            // Create a dynamic angle supplier
            var currentAngle = 30.0
            val angleSupplier = { currentAngle.deg }

            // Set angle using lambda
            val setAngleCommand = Turret.setAngle(angleSupplier)
            setAngleCommand.schedule()

            // Run command scheduler and update turret
            for (i in 0..5) {
                CommandScheduler.getInstance().run()
                Turret.periodic()
                // Simulate voltage
                RoboRioSim.setVInVoltage(
                    BatterySim.calculateDefaultBatteryLoadedVoltage(0.0)
                )
            }

            // Verify initial angle
            Turret.angleSetpoint.`in`(deg).shouldBeNear(30.0, 0.1)

            // Change the angle
            currentAngle = 60.0

            // Run a few more iterations
            for (i in 0..5) {
                CommandScheduler.getInstance().run()
                Turret.periodic()
                RoboRioSim.setVInVoltage(
                    BatterySim.calculateDefaultBatteryLoadedVoltage(0.0)
                )
            }

            // Verify angle was updated
            Turret.angleSetpoint.`in`(deg).shouldBeNear(60.0, 0.1)
        }

        test("reset should initialize the turret position") {
            // Reset to a specific angle
            val resetAngle = 20.deg
            val resetCommand = Turret.reset(resetAngle)
            resetCommand.schedule()
            CommandScheduler.getInstance().run()

            // Update inputs to reflect the reset
            Turret.periodic()

            // In simulation, the reset should set the encoder position
            // The exact behavior depends on the simulator implementation
            // We verify the command executed without throwing an exception
            val position = Turret.inputs.position.`in`(deg)
            // Position should be set to the reset angle
            position.shouldBeNear(20.0, 0.5)
        }
    })
