package frc.robot.lib

import com.ctre.phoenix6.configs.MotionMagicConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
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

    private fun getTunable(name: String, gain: Any?) =
        when (gain) {
            is LoggedNetworkNumber,
            null -> gain
            !is Number ->
                throw IllegalArgumentException(
                    "gain must be a double or a tunable, not $gain"
                )
            else -> LoggedNetworkNumber("$path/$name", gain as Double)
        }

    private val kPTunable by lazy { getTunable("kP", kP) }
    private val kITunable by lazy { getTunable("kI", kI) }
    private val kDTunable by lazy { getTunable("kD", kD) }
    private val kSTunable by lazy { getTunable("kS", kS) }
    private val kVTunable by lazy { getTunable("kV", kV) }
    private val kATunable by lazy { getTunable("kA", kA) }
    private val kGTunable by lazy { getTunable("kG", kG) }
    private val cruiseVelocityTunable by lazy {
        getTunable("Cruise Velocity", cruiseVelocity)
    }
    private val accelerationTunable by lazy {
        getTunable("Acceleration", acceleration)
    }
    private val jerkTunable by lazy { getTunable("Jerk", jerk) }

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

    val pidController: PIDController by lazy {
        PIDController(
            kPTunable?.get() ?: 0.0,
            kITunable?.get() ?: 0.0,
            kDTunable?.get() ?: 0.0,
        )
    }

    val profiledPIDController: ProfiledPIDController by lazy {
        ProfiledPIDController(
            kPTunable?.get() ?: 0.0,
            kITunable?.get() ?: 0.0,
            kDTunable?.get() ?: 0.0,
            TrapezoidProfile.Constraints(
                cruiseVelocityTunable?.get() ?: 0.0,
                accelerationTunable?.get() ?: 0.0
            )
        )
    }
}

var PIDController.gains: TunableGains
    get() =
        TunableGains(
            "I don't think anyone will ever use this"
        ) // Dummy return to satisfy getter requirement.
    set(gains) {
        // It's possible that some PID gains are null, so we try/catch each one.
        try {
            p = gains.kP
        } catch (_: NullPointerException) {}
        try {
            i = gains.kI
        } catch (_: NullPointerException) {}
        try {
            d = gains.kD
        } catch (_: NullPointerException) {}
        reset()
    }

var ProfiledPIDController.gains: TunableGains
    get() =
        TunableGains(
            "I don't think anyone will ever use this"
        ) // Dummy return to satisfy getter requirement.
    set(gains) {
        // It's possible that some PID gains are null, so we try/catch each one. However, there
        // shouldn't be a case where
        // we use a profiled PID controller without cruise velocity and acceleration gains, so no
        // null check there.
        try {
            p = gains.kP
        } catch (_: NullPointerException) {}
        try {
            i = gains.kI
        } catch (_: NullPointerException) {}
        try {
            d = gains.kD
        } catch (_: NullPointerException) {}
        constraints =
            TrapezoidProfile.Constraints(
                gains.cruiseVelocity,
                gains.acceleration
            )
        reset(this.goal)
    }
