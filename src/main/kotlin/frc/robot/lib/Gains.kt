package frc.robot.lib

import com.ctre.phoenix6.configs.MotionMagicConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import edu.wpi.first.units.measure.AngularAcceleration
import edu.wpi.first.units.measure.AngularVelocity
import frc.robot.lib.extensions.*
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber

data class Gains(
    var kP: Double = 0.0,
    var kI: Double = 0.0,
    var kD: Double = 0.0,
    var kS: Double = 0.0,
    var kV: Double = 0.0,
    var kA: Double = 0.0,
    var kG: Double = 0.0,
    val motionMagicGains: MotionMagicGains = MotionMagicGains()
) {
    /**
     * A function to convert a [Gains] type to the [Slot0Configs] that the motor
     * uses.
     */
    fun toSlotConfig() =
        Slot0Configs().apply {
            kP = this@Gains.kP
            kI = this@Gains.kI
            kD = this@Gains.kD
            kA = this@Gains.kA
            kS = this@Gains.kS
            kV = this@Gains.kV
            kG = this@Gains.kG
        }
}

/**
 * A function to convert motion magic parameters to a [MotionMagicConfigs]
 * object.
 *
 * @param cruiseVelocity Maximum velocity Motion Magic® based control modes are
 * allowed to use. Motion Magic® Velocity control modes do not use this config.
 * When using Motion Magic® Expo control modes, setting this to 0 will allow the
 * profile to run to the max possible velocity based on Expo_kV.
 * @param acceleration Target acceleration Motion Magic® based control modes are
 * allowed to use. Motion Magic® Expo control modes do not use this config.
 * @param jerk Target jerk (acceleration derivative) Motion Magic® based control
 * modes are allowed to use. Motion Magic® Expo control modes do not use this
 * config. This allows Motion Magic® to generate S-Curve profiles.
 * @param expoKv Target kV used only by Motion Magic® Expo control modes. Unlike
 * the kV slot gain, this is always in units of V/rps. This represents the
 * amount of voltage necessary to hold a velocity. In terms of the Motion Magic®
 * Expo profile, a higher kV results in a slower maximum velocity.
 * @param expoKa Target kA used only by Motion Magic® Expo control modes. Unlike
 * the kA slot gain, this is always in units of V/rps². This represents the
 * amount of voltage necessary to achieve an acceleration. In terms of the
 * Motion Magic® Expo profile, a higher kA results in a slower acceleration.
 * @return A [MotionMagicConfigs] object with the provided parameters set.
 */
fun getMotionMagicConfigs(
    cruiseVelocity: AngularVelocity? = null,
    acceleration: AngularAcceleration? = null,
    jerk: AngularJerk? = null,
    expoKv: Double? = null,
    expoKa: Double? = null
) = MotionMagicConfigs().apply {
    cruiseVelocity?.let { MotionMagicCruiseVelocity = it[rps] }
    acceleration?.let { MotionMagicAcceleration = it[rps_squared] }
    jerk?.let { MotionMagicJerk = it[rps_tripled] }
    expoKv?.let { MotionMagicExpo_kV = it }
    expoKa?.let { MotionMagicExpo_kA = it }
}


data class MotionMagicGains(
    var cruiseVelocity: AngularVelocity = 0.rps,
    var acceleration: AngularAcceleration = 0.rps_squared,
    var jerk: Double = 0.0 // m/s³
) {
    fun toMotionMagicConfig() =
        MotionMagicConfigs().apply {
            MotionMagicCruiseVelocity =
                this@MotionMagicGains.cruiseVelocity[rps]
            MotionMagicAcceleration =
                this@MotionMagicGains.acceleration[rps_squared]
            MotionMagicJerk = this@MotionMagicGains.jerk
        }
}

class LoggedNetworkGains(
    name: String,
    kP: Double = 0.0,
    kI: Double = 0.0,
    kD: Double = 0.0,
    kS: Double = 0.0,
    kV: Double = 0.0,
    kA: Double = 0.0,
    kG: Double = 0.0,
    cruiseVelocity: AngularVelocity = 0.rps,
    acceleration: AngularAcceleration = 0.rps_squared,
    jerk: Double = 0.0, // m/s³
    key: String =
        (Throwable().stackTrace[1]?.fileName?.substringBeforeLast('.') + ""),
) {
    private val path = "/Tuning/$key/$name"
    val kP: LoggedNetworkNumber = LoggedNetworkNumber("$path/kP", kP)
    val kI: LoggedNetworkNumber = LoggedNetworkNumber("$path/kI", kD)
    val kD: LoggedNetworkNumber = LoggedNetworkNumber("$path/kD", kI)
    val kS: LoggedNetworkNumber = LoggedNetworkNumber("$path/kS", kS)
    val kV: LoggedNetworkNumber = LoggedNetworkNumber("$path/kV", kV)
    val kA: LoggedNetworkNumber = LoggedNetworkNumber("$path/kA", kA)
    val kG: LoggedNetworkNumber = LoggedNetworkNumber("$path/kG", kG)
    var jerk: LoggedNetworkNumber? = null
    var cruiseVelocity: LoggedNetworkNumber? = null
    var acceleration: LoggedNetworkNumber? = null

    init {
        if (
            jerk != 0.0 ||
            cruiseVelocity[rad_ps] != 0.0 ||
            acceleration[rad_ps_ps] != 0.0
        ) {
            this@LoggedNetworkGains.jerk =
                LoggedNetworkNumber("$path/jerk", jerk)
            this@LoggedNetworkGains.cruiseVelocity =
                LoggedNetworkNumber(
                    "$path/cruiseVelocity",
                    cruiseVelocity[rad_ps]
                )
            this@LoggedNetworkGains.acceleration =
                LoggedNetworkNumber(
                    "$path/acceleration",
                    acceleration[rad_ps_ps]
                )
        }
    }

    fun toSlotConfig() =
        Slot0Configs().apply {
            kP = this@LoggedNetworkGains.kP.get()
            kI = this@LoggedNetworkGains.kI.get()
            kD = this@LoggedNetworkGains.kD.get()
            kA = this@LoggedNetworkGains.kA.get()
            kS = this@LoggedNetworkGains.kS.get()
            kV = this@LoggedNetworkGains.kV.get()
            kG = this@LoggedNetworkGains.kG.get()
        }

    fun toMotionMagicConfig() =
        MotionMagicConfigs().apply {
            MotionMagicCruiseVelocity =
                this@LoggedNetworkGains.cruiseVelocity?.get() ?: 0.0
            MotionMagicAcceleration =
                this@LoggedNetworkGains.acceleration?.get() ?: 0.0
            MotionMagicJerk = this@LoggedNetworkGains.jerk?.get() ?: 0.0
        }
}
