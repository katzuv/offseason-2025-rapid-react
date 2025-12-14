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

        test("setAngle should set the motor position setpoint") {
            // Reset the turret to a known position
            val resetCommand = Turret.reset(0.deg)
            resetCommand.schedule()
            CommandScheduler.getInstance().run()

            // Set a target angle
            val targetAngle = 45.deg
            val setAngleCommand = Turret.setAngle(targetAngle)
            setAngleCommand.schedule()
            CommandScheduler.getInstance().run()

            // Simulate periodic update
            Turret.periodic()

            // Verify the angle setpoint is set correctly
            Turret.angleSetpoint.`in`(deg).shouldBeNear(45.0, 0.1)
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

        test("reset should set the turret position to specified angle") {
            // Reset to a specific angle
            val resetAngle = 20.deg
            val resetCommand = Turret.reset(resetAngle)
            resetCommand.schedule()
            CommandScheduler.getInstance().run()

            // The motor position should be reset
            // Since this is a simulation, we verify the command executed
            // without errors
            Turret.inputs.position.`in`(deg).shouldBeNear(20.0, 0.1)
        }
    })
