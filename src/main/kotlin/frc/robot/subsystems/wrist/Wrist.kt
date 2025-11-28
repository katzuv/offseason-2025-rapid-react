package frc.robot.subsystems.wrist

import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VoltageOut
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.StartEndCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.degrees
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.kg2m
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d
import org.team5987.annotation.LoggedOutput

object Wrist : SubsystemBase() {
    @AutoLogOutput private var mechanism = LoggedMechanism2d(6.0, 4.0)

    private var root = mechanism.getRoot("Wrist", 3.0, 2.0)

    private val ligament =
        root.append(LoggedMechanismLigament2d("WristLigament", 0.25, 90.0))

    @LoggedOutput var setpoint: Angle = 0.degrees

    private val motor =
        UniversalTalonFX(
            MOTOR_PORT,
            momentOfInertia = 0.0025.kg2m,
            gearRatio = GEAR_RATIO,
            config = MOTOR_CONFIG,
            simGains = SIM_CONFIG
        )

    @LoggedOutput
    val atSetpoint = Trigger {
        motor.inputs.position.isNear(setpoint, SETPOINT_TOLERANCE)
    }

    val inputs = motor.inputs

    private val positionRequest = PositionVoltage(0.0)
    private val voltageRequest = VoltageOut(0.0)

    init {
        motor.reset()
    }

    fun reset(angle: Angle = WristAngles.OPEN.angle): Command =
        StartEndCommand(
            { motor.setControl(voltageRequest.withOutput(RESET_VOLTAGE)) },
            {
                motor.setControl(voltageRequest.withOutput(0.0))
                motor.reset(angle)
            }
        )

    fun setAngle(angle: WristAngles): Command = runOnce {
        setpoint = angle.angle
        motor.setControl(positionRequest.withPosition(angle.angle))
    }
    fun open(): Command = setAngle(WristAngles.OPEN)

    fun close(): Command = setAngle(WristAngles.CLOSED)

    fun default(): Command = setAngle(WristAngles.DEFAULT)

    override fun periodic() {
        motor.updateInputs()
        ligament.setAngle(setpoint[deg])
        Logger.recordOutput("Subsystems/$name/Ligament", mechanism)
        Logger.processInputs("Subsystems/$name", motor.inputs)
    }
}
