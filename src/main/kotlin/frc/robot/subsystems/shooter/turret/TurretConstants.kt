package frc.robot.subsystems.shooter.turret

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotionMagicConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import frc.robot.lib.Gains
import frc.robot.lib.extensions.amps
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.get

val TOLERANCE = 2.deg
const val MOTOR_ID = 7
val GAINS = Gains(kP = 95.0, kD = 12.0, kS = 0.04, kV = 3.0, kA = 0.5)
val STATOR_CURRENT_LIMIT = 80.amps
val SUPPLY_CURRENT_LIMIT = 40.amps
val SOFTWARE_LIMIT_CONFIG =
    SoftwareLimitSwitchConfigs().apply {
        ForwardSoftLimitEnable = true
        ForwardSoftLimitThreshold = 0.73291
        ReverseSoftLimitEnable = true
        ReverseSoftLimitThreshold = -0.005
    }
val MOTOR_CONFIG: TalonFXConfiguration =
    TalonFXConfiguration().apply {
        MotorOutput =
            MotorOutputConfigs().apply {
                NeutralMode = NeutralModeValue.Coast
                Inverted = InvertedValue.Clockwise_Positive
            }
        SoftwareLimitSwitch = SOFTWARE_LIMIT_CONFIG

        Feedback = FeedbackConfigs().apply { SensorToMechanismRatio = 56.0 }
        Slot0 = GAINS.toSlotConfig()
        CurrentLimits =
            CurrentLimitsConfigs().apply {
                StatorCurrentLimitEnable = true
                SupplyCurrentLimitEnable = true
                StatorCurrentLimit = STATOR_CURRENT_LIMIT[amps]
                SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT[amps]
            }
        MotionMagic =
            MotionMagicConfigs().apply {
                MotionMagicCruiseVelocity = 5.0
                MotionMagicAcceleration = 10.0
                MotionMagicJerk = 90.0
            }
    }
