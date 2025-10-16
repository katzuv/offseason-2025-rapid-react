package frc.robot.subsystems.shooter.flywheel

import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC
import com.ctre.phoenix6.controls.VoltageOut
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.rps
import frc.robot.lib.extensions.sec
import frc.robot.lib.named
import frc.robot.lib.namedRun
import frc.robot.lib.namedRunOnce
import frc.robot.lib.sysid.SysIdable
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger

class Flywheel : SubsystemBase(), SysIdable {
    private val mainMotor =
        UniversalTalonFX(MAIN_MOTOR_PORT, config = MOTOR_CONFIG)
    private val auxMotor =
        UniversalTalonFX(AUX_MOTOR_PORT, config = MOTOR_CONFIG)
    private val velocityTorque = VelocityTorqueCurrentFOC(0.0)
    private val voltageOut = VoltageOut(0.0)
    private var velocitySetpoint = 0.rps
    val velocity
        get() = mainMotor.inputs.velocity
    init {
        auxMotor.setControl(Follower(MAIN_MOTOR_PORT, true))
    }

    val isAtSetVelocity =
        Trigger {
                mainMotor.inputs.velocity.isNear(velocitySetpoint, TOLERANCE)
            }
            .debounce(AT_SET_VELOCITY_DEBOUNCE[sec])

    fun setVelocity(velocity: AngularVelocity): Command = namedRunOnce {
        velocitySetpoint = velocity
        mainMotor.setControl(velocityTorque.withVelocity(velocitySetpoint))
    }

    fun setVelocity(velocity: () -> AngularVelocity): Command = namedRun {
        velocitySetpoint = velocity.invoke()
        mainMotor.setControl(velocityTorque.withVelocity(velocitySetpoint))
    }

    fun slowRotation() = setVelocity(SLOW_ROTATION).named()

    fun stop() = setVelocity(0.rps).named()

    override fun setVoltage(voltage: Voltage) {
        mainMotor.setControl(voltageOut.withOutput(voltage))
    }

    override fun periodic() {
        mainMotor.updateInputs()
        Logger.processInputs("Subsystems/$name", mainMotor.inputs)
        Logger.recordOutput("Subsystems/$name/IsAtSetVelocity", isAtSetVelocity)
        Logger.recordOutput("Subsystems/$name/SetVelocity", velocitySetpoint)
    }
}
