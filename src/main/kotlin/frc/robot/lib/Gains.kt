package frc.robot.lib

import com.ctre.phoenix6.configs.MotionMagicConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber

/**
 * A class that creates tunable numbers for gains. Only gains that are provided
 * by the constructor will have gains created for them.
 *
 * @param name The name of the gain set. This is used as part of the network
 * table path.
 * @param kP Proportional gain.
 * @param kI Integral gain.
 * @param kD Derivative gain.
 * @param kS Static gain.
 * @param kV Velocity gain.
 * @param kA Acceleration gain.
 * @param kG Gravity gain.
 * @param cruiseVelocity The cruise velocity in m/s or rad/s.
 * @param acceleration The acceleration in m/s² or rad/s².
 * @param jerk The jerk in m/s³ or rad/s³.
 * @param key An optional key to further distinguish the network table path. By
 * default, this is the name of the file that instantiated the class.
 */
class TunableGains(
    name: String,
    key: String =
        (Throwable().stackTrace[1]?.fileName?.substringBeforeLast('.') + ""),
    kP: Any? = null,
    kI: Any? = null,
    kD: Any? = null,
    kS: Any? = null,
    kV: Any? = null,
    kA: Any? = null,
    kG: Any? = null,
    cruiseVelocity: Any? = null,
    acceleration: Any? = null,
    jerk: Any? = null,
) {
    private val path = "/Tuning/$key/$name"

    private fun getTunable(name: String, gain: Any?): LoggedNetworkNumber? {
        if (gain == null || gain is LoggedNetworkNumber) {
            return gain
        }
        if (gain !is Number) {
            throw IllegalArgumentException(
                "gain must be a double or a tunable, not $gain"
            )
        }
        return LoggedNetworkNumber("$path/$name", gain as Double)
    }

    val kPTunable = getTunable("kP", kP)
    val kITunable = getTunable("kI", kI)
    val kDTunable = getTunable("kD", kD)
    val kSTunable = getTunable("kS", kS)
    val kVTunable = getTunable("kV", kV)
    val kATunable = getTunable("kA", kA)
    val kGTunable = getTunable("kG", kG)
    val cruiseVelocityTunable = getTunable("Cruise Velocity", cruiseVelocity)
    val accelerationTunable = getTunable("Acceleration", acceleration)
    val jerkTunable = getTunable("Jerk", jerk)

    // If you call this, you probably should have a tunable for that.
    val kP
        get() = kPTunable!!.get()
    val kI
        get() = kITunable!!.get()
    val kD
        get() = kDTunable!!.get()
    val kS
        get() = kSTunable!!.get()
    val kV
        get() = kVTunable!!.get()
    val kA
        get() = kATunable!!.get()
    val cruiseVelocity
        get() = cruiseVelocityTunable!!.get()
    val acceleration
        get() = accelerationTunable!!.get()
    val jerk
        get() = jerkTunable!!.get()

    val slot0Config
        get() =
            Slot0Configs().apply {
                // `kP = kPTunable?.get() ?: kP` is also an option.
                kPTunable?.let { kP = it.get() }
                kITunable?.let { kI = it.get() }
                kDTunable?.let { kD = it.get() }
                kATunable?.let { kA = it.get() }
                kSTunable?.let { kS = it.get() }
                kVTunable?.let { kV = it.get() }
                kGTunable?.let { kG = it.get() }
            }

    val motionMagicConfig
        get() =
            MotionMagicConfigs().apply {
                cruiseVelocityTunable?.let {
                    MotionMagicCruiseVelocity = it.get()
                }
                accelerationTunable?.let { MotionMagicAcceleration = it.get() }
                jerkTunable?.let { MotionMagicJerk = it.get() }
            }
}
