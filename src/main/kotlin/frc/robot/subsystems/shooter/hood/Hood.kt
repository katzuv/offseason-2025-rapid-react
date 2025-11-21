package frc.robot.subsystems.shooter.hood

import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.CANcoder
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.get
import frc.robot.lib.sysid.SysIdable
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber

@AutoLogOutput(key = "Hood/mechanism")
private var mechanism = LoggedMechanism2d(6.0, 4.0)
private var root = mechanism.getRoot("Hood", 3.0, 2.0)
private val ligament =
    root.append(LoggedMechanismLigament2d("HoodLigament", 0.25, 90.0))

object Hood : SubsystemBase(), SysIdable {

    private val motor =
        UniversalTalonFX(
            MOTOR_ID,
            config = MOTOR_CONFIG,
            gearRatio = MOTOR_TO_MECHANISM_RATIO,
            absoluteEncoderOffset = -ENCODER_OFFSET
        )

    val inputs
        get() = motor.inputs

    private val positionRequest = PositionVoltage(0.0)
    private val voltageRequest = VoltageOut(0.0)

    private val encoder = CANcoder(ENCODER_ID)

    private var setpoint: Angle = 0.deg

    private val calibrationAngle =
        LoggedNetworkNumber("/Tuning/calibrationHoodAngle", 0.0)

    val isAtSetpoint = Trigger {
        motor.inputs.position.isNear(setpoint, SETPOINT_TOLERANCE)
    }

    init {
        encoder.configurator.apply(ENCODER_CONFIG)
    }

    override fun setVoltage(voltage: Voltage) {
        motor.setControl(voltageRequest.withOutput(voltage))
    }

    fun setAngle(angle: Angle): Command = runOnce {
        setpoint = angle
        motor.setControl(positionRequest.withPosition(setpoint))
    }

    fun setAngle(angle: () -> Angle): Command = run {
        setpoint = angle.invoke()
        motor.setControl(positionRequest.withPosition(setpoint))
    }

    fun setCalibrationAngle(): Command = setAngle { calibrationAngle.get().deg }

    override fun periodic() {
        motor.updateInputs()
        ligament.setAngle(setpoint[deg])
        Logger.processInputs("Subsystems/$name", motor.inputs)
        Logger.recordOutput("Subsystems/$name/isAtSetpoint", isAtSetpoint)
        Logger.recordOutput("Subsystems/$name/setpoint", setpoint)
        Logger.recordOutput("Subsystems/$name/Ligament", mechanism)
    }
}
