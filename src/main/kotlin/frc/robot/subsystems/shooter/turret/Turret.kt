package frc.robot.subsystems.shooter.turret

import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.deg
import frc.robot.lib.sysid.SysIdable
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d
import org.team5987.annotation.LoggedOutput

@AutoLogOutput private var mechanism = LoggedMechanism2d(6.0, 4.0)
private var root = mechanism.getRoot("Turret", 3.0, 2.0)
private val ligament =
    root.append(LoggedMechanismLigament2d("TurretLigament", 0.25, 90.0))

object Turret : SubsystemBase(), SysIdable {
    private val motor = UniversalTalonFX(MOTOR_ID, config = MOTOR_CONFIG)
    private val positionVoltage = PositionVoltage(0.0)
    @LoggedOutput var angleSetpoint = 0.deg
    private val voltageRequest = VoltageOut(0.0)
    val inputs
        get() = motor.inputs

    init {
        motor.reset()
    }

    @LoggedOutput
    val isAtSetpoint = Trigger {
        motor.inputs.position.isNear(angleSetpoint, TOLERANCE)
    }

    val isDisabled =
        Trigger { DriverStation.isEnabled() }
            .onTrue(setNeutralMode(NeutralModeValue.Brake))
            .onFalse(setNeutralMode(NeutralModeValue.Coast))

    fun setNeutralMode(mode: NeutralModeValue): Command =
        runOnce {
                MOTOR_CONFIG.MotorOutput.NeutralMode = mode
                motor.applyConfiguration(MOTOR_CONFIG)
            }
            .ignoringDisable(true)

    fun setAngle(angle: Angle) = runOnce {
        motor.setControl(positionVoltage.withPosition(angle))
    }

    fun setAngle(angle: () -> Angle) = run {
        angleSetpoint = angle.invoke()
        motor.setControl(positionVoltage.withPosition(angle.invoke()))
    }

    fun reset(angle: Angle = 0.deg) = this.runOnce { motor.reset(angle) }

    override fun periodic() {
        motor.updateInputs()
        ligament.setAngle(angleSetpoint.`in`(deg))
        Logger.processInputs("Subsystems/$name", motor.inputs)
        Logger.recordOutput("Subsystems/$name/isAtSetpoint", isAtSetpoint)
        Logger.recordOutput("Subsystems/$name/Ligament", mechanism)
    }

    override fun setVoltage(voltage: Voltage) {
        motor.setControl(voltageRequest.withOutput(voltage))
    }
}
