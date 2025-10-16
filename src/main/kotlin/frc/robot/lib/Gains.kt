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
 * @param kP Proportional gain. If null, this gain will not be tunable.
 * @param kI Integral gain. If null, this gain will not be tunable.
 * @param kD Derivative gain. If null, this gain will not be tunable.
 * @param kS Static gain. If null, this gain will not be tunable.
 * @param kV Velocity gain. If null, this gain will not be tunable.
 * @param kA Acceleration gain. If null, this gain will not be tunable.
 * @param kG Gravity gain. If null, this gain will not be tunable.
 * @param cruiseVelocity The cruise velocity in m/s or rad/s. If null, this
 * parameter will not be tunable.
 * @param acceleration The acceleration in m/s² or rad/s². If null, this
 * parameter will not be tunable.
 * @param jerk The jerk in m/s³ or rad/s³. If null, this parameter will not be
 * tunable.
 * @param key An optional key to further distinguish the network table path. By
 * default, this is the name of the file that instantiated the class.
 */
class TunableGains(
    name: String,
    key: String =
        (Throwable().stackTrace[1]?.fileName?.substringBeforeLast('.') + ""),
    kP: Double? = null,
    kI: Double? = null,
    kD: Double? = null,
    kS: Double? = null,
    kV: Double? = null,
    kA: Double? = null,
    kG: Double? = null,
    cruiseVelocity: Double? = null,
    acceleration: Double? = null,
    jerk: Double? = null,
) {
    private val path = "/Tuning/$key/$name"
    val kPTunable = kP?.let { LoggedNetworkNumber("$path/kP", it) }
    val kITunable = kI?.let { LoggedNetworkNumber("$path/kI", it) }
    val kDTunable = kD?.let { LoggedNetworkNumber("$path/kD", it) }
    val kSTunable = kS?.let { LoggedNetworkNumber("$path/kS", it) }
    val kVTunable = kV?.let { LoggedNetworkNumber("$path/kV", it) }
    val kATunable = kA?.let { LoggedNetworkNumber("$path/kA", it) }
    val kGTunable = kG?.let { LoggedNetworkNumber("$path/kG", it) }
    val cruiseVelocityTunable =
        cruiseVelocity?.let { LoggedNetworkNumber("$path/Cruise velocity", it) }
    val accelerationTunable =
        acceleration?.let { LoggedNetworkNumber("$path/Acceleration", it) }
    val jerkTunable = jerk?.let { LoggedNetworkNumber("$path/Jerk", it) }

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
