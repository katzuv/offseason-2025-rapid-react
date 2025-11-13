package frc.robot.subsystems.wrist

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import edu.wpi.first.units.measure.Angle
import frc.robot.lib.Gains
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.rot
import frc.robot.lib.extensions.volts

val SETPOINT_TOLERANCE = 1.deg
const val MOTOR_PORT = 2
const val GEAR_RATIO = 1 / 69.82
val RESET_VOLTAGE = 2.0.volts

val SIM_CONFIG: Gains = Gains(kP = 0.7, kD = 0.1)

val MOTOR_CONFIG =
    TalonFXConfiguration().apply {
        Slot0 = Slot0Configs().apply { kP = 0.8 }
        CurrentLimits =
            CurrentLimitsConfigs().apply {
                StatorCurrentLimitEnable = true
                SupplyCurrentLimitEnable = true
                StatorCurrentLimit = 20.0
                SupplyCurrentLimit = 40.0
            }

        SoftwareLimitSwitch =
            SoftwareLimitSwitchConfigs().apply {
                ForwardSoftLimitEnable = true
                ForwardSoftLimitThreshold = 15.5
                ReverseSoftLimitEnable = true
            }
    }

enum class WristAngles(val angle: Angle) {
    CLOSED(15.5.rot),
    OPEN(0.rot),
    DEFAULT(2.4.rot)
}
