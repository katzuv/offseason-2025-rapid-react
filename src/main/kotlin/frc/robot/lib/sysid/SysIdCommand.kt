package frc.robot.lib.sysid

import edu.wpi.first.units.VoltageUnit
import edu.wpi.first.units.measure.Time
import edu.wpi.first.units.measure.Velocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.waitTime
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.lib.extensions.div
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.sec
import frc.robot.lib.extensions.volts
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber

/**
 * Extension function that creates a [SysIdCommand] for any subsystem that
 * implements [SysIdable] and [SubsystemBase].
 * @param rampRate Ramp rate to apply during the quasistatic motion test.
 * @param stepVoltage Step voltage to apply during the dynamic test.
 * @param timeout Maximum duration the routine is allowed to run.
 *
 * @return A [SysIdCommand] instance.
 */
fun <T> T.sysId(
    rampRate: Velocity<VoltageUnit>, stepVoltage: Voltage, timeout: Time
): Command where T : SysIdable, T : SubsystemBase = SysIdCommand(this, rampRate, stepVoltage, timeout).command()

/**
 * Interface that allows a subsystem to be characterized via SysId. Must provide
 * a method to set voltage on the subsystem.
 */
interface SysIdable {
    /**
     * Function that consumes a voltage and applies it to the subsystem.
     * Defaults to using [setVoltage].
     */
    val setVoltageConsumer: (Voltage) -> Unit
        get() = { setVoltage(it) }

    /** Applies the specified [voltage] to the subsystem. */
    fun setVoltage(voltage: Voltage)
}

/**
 * A WPILib [Command] to run SysId characterization routine.
 *
 * @param T The subsystem which must implement [SysIdable] and extend
 * [SubsystemBase].
 * @param rampRate Ramp rate to apply during the quasistatic motion test.
 * @param stepVoltage Step voltage to apply during the dynamic test.
 * @param timeout Maximum duration the routine is allowed to run.
 */
class SysIdCommand<T>
internal constructor(
    private val subsystem: T, rampRate: Velocity<VoltageUnit>, stepVoltage: Voltage, timeout: Time
) where T : SysIdable, T : SubsystemBase {
    private val loggingPath = "Tuning/SysId/${subsystem.name}"
    private val rampRateTunableNumber =
        LoggedNetworkNumber("$loggingPath/Ramp rate [Quasistatic] [V per sec]", rampRate[volts / sec])
    private val stepVoltageTunableNumber =
        LoggedNetworkNumber("$loggingPath/Step voltage [Dynamic]", stepVoltage[volts])
    private val timeoutTunableNumber = LoggedNetworkNumber("$loggingPath/Timeout", timeout[sec])

    private val waitBetweenRuns = 1.sec

    /**
     * Creates the [SysIdRoutine] object with configuration values from tunable
     * numbers. Defaults to arguments passed in the constructor.
     */
    private fun createRoutine() = SysIdRoutine(
        SysIdRoutine.Config(
            rampRateTunableNumber.get().volts / sec,
            stepVoltageTunableNumber.get().volts,
            timeoutTunableNumber.get().sec,
        ) { state: SysIdRoutineLog.State ->
            Logger.recordOutput(
                "SysId/${subsystem.name}/state", state.toString()
            )
        }, SysIdRoutine.Mechanism(
            // `log` is `null` because data is already logged by AdvantageKit.
            subsystem.setVoltageConsumer, null, subsystem
        )
    )

    /**
     * Builds the full characterization command sequence:
     * 1. Initializes routine
     * 2. Runs forward dynamic
     * 3. Runs backward dynamic
     * 4. Runs forward quasistatic
     * 5. Runs backward quasistatic
     *
     * Waits `TIME_BETWEEN_ROUTINES` between each step.
     *
     * @return Full [Command] sequence.
     */
    fun command(): Command = subsystem.defer {
        val routine = createRoutine()
        Commands.sequence(
            Commands.print("${rampRateTunableNumber.get()} ${stepVoltageTunableNumber.get()} ${timeoutTunableNumber.get()}"),
            routine.dynamic(SysIdRoutine.Direction.kForward),
            waitTime(waitBetweenRuns),
            routine.dynamic(SysIdRoutine.Direction.kReverse),
            waitTime(waitBetweenRuns),
            routine.quasistatic(SysIdRoutine.Direction.kForward),
            waitTime(waitBetweenRuns),
            routine.quasistatic(SysIdRoutine.Direction.kReverse)
        )
    }.withName("${subsystem.name}/Characterize")
}
