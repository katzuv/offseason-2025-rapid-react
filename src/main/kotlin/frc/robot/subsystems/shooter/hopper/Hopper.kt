package frc.robot.subsystems.shooter.hopper

import com.ctre.phoenix6.controls.VoltageOut
import com.revrobotics.ColorSensorV3
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.CURRENT_MODE
import frc.robot.lib.Mode
import frc.robot.lib.colorSimilarity
import frc.robot.lib.extensions.volts
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean
import org.team5987.annotation.LoggedOutput

object Hopper : SubsystemBase() {

    private val motor: UniversalTalonFX =
        UniversalTalonFX(MOTOR_ID, config = MOTOR_CONFIG)

    private val colorSensor = ColorSensorV3(COLOR_SENSOR_PORT)
    private val voltageRequest = VoltageOut(0.0)

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
        redConfidence > HOPPER_COLOR_SIMILARITY_THRESHOLD
    }

    @LoggedOutput
    val isBallBlue: Trigger = Trigger {
        blueConfidence > HOPPER_COLOR_SIMILARITY_THRESHOLD
    }

    private val simulatedHasBall =
        LoggedNetworkBoolean("/Tuning/Hopper/hasBall", false)

    @LoggedOutput
    val hasBall: Trigger =
        if (CURRENT_MODE == Mode.REAL) isBallBlue.or(isBallRed)
        else Trigger { simulatedHasBall.get() }

    private fun setVoltageCommand(voltage: Voltage): Command = runOnce {
        setVoltage(voltage)
    }

    private fun setVoltage(voltage: Voltage) {
        motor.setControl(voltageRequest.withOutput(voltage))
    }

    fun startIntake(): Command = setVoltageCommand(INTAKE_VOLTAGE)

    fun startShoot(): Command = setVoltageCommand(SHOOT_VOLTAGE)

    fun slowBack(): Command =
        Commands.sequence(
            this.run { setVoltage(SLOW_BACK_VOLTAGE) }
                .withTimeout(SLOW_BACK_TIMEOUT),
            stop()
        )

    fun stop(): Command = setVoltageCommand(0.volts)

    fun outtake(): Command = setVoltageCommand(-INTAKE_VOLTAGE)

    override fun periodic() {
        motor.updateInputs()
        Logger.processInputs("Subsystems/$name", motor.inputs)
    }
}
