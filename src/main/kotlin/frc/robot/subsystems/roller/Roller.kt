package frc.robot.subsystems.roller

import com.ctre.phoenix6.controls.VoltageOut
import com.revrobotics.ColorSensorV3
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.CURRENT_MODE
import frc.robot.lib.Mode
import frc.robot.lib.colorSimilarity
import frc.robot.lib.extensions.*
import frc.robot.lib.universal_motor.UniversalTalonFX
import frc.robot.subsystems.shooter.hopper.BLUE_COLOR
import frc.robot.subsystems.shooter.hopper.RED_COLOR
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean
import org.team5987.annotation.LoggedOutput

object Roller : SubsystemBase() {
    private val motor =
        UniversalTalonFX(
            MOTOR_PORT,
            momentOfInertia = 0.002.kg2m,
            config = MOTOR_CONFIG
        )

    private val auxiliaryMotor =
        UniversalTalonFX(
            AUXILIARY_MOTOR_PORT,
            momentOfInertia = (0.002).kilogramSquareMeters,
            config = MOTOR_CONFIG
        )

    private val voltageRequest = VoltageOut(0.0)

    private val colorSensor = ColorSensorV3(COLOR_SENSOR_PORT)

    @LoggedOutput
    val ballColor: Color
        get() = colorSensor.color

    @LoggedOutput
    val redConfidence
        get() = ballColor.colorSimilarity(RED_COLOR)

    @LoggedOutput
    val blueConfidence
        get() = ballColor.colorSimilarity(BLUE_COLOR)

    @LoggedOutput
    val isBallRed: Trigger = Trigger {
        redConfidence > ROLLER_COLOR_SIMILARITY_THRESHOLD
    }

    @LoggedOutput
    val isBallBlue: Trigger = Trigger {
        blueConfidence > ROLLER_COLOR_SIMILARITY_THRESHOLD
    }

    private val simulatedHasBall =
        LoggedNetworkBoolean("/Tuning/Roller/hasBall", false)

    @LoggedOutput
    val hasBall: Trigger =
        if (CURRENT_MODE == Mode.REAL)
            (isBallBlue.or(isBallRed)).debounce(
                SENSOR_DEBOUNCE[sec],
                Debouncer.DebounceType.kRising
            )
        else Trigger { simulatedHasBall.get() }

    private fun setVoltage(voltage: Voltage): Command = runOnce { motor.setControl(voltageRequest.withOutput(voltage)) }

    fun intake(): Command = setVoltage(INTAKE)

    fun outtake(): Command = setVoltage(OUTTAKE)

    fun stop(): Command = setVoltage(STOP)

    override fun periodic() {
        motor.updateInputs()
        Logger.processInputs("Subsystems/$name", motor.inputs)
    }
}
