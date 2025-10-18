package frc.robot.subsystems.shooter.turret

import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import frc.robot.lib.TunableGains
import frc.robot.lib.extensions.amps
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.get

val TOLERANCE = 2.deg
val MAX_ANGLE = 135.deg
val MIN_ANGLE = (-135).deg
const val HALL_EFFECT_SENSOR_PORT = 0
const val MOTOR_ID = 7
val CONTROL_GAINS =
    TunableGains(
        "Subsystems/Shooter",
        "Turret gains",
        kP = 1.0,
        kV = 1.0,
        kA = 1.0
    )
val STATOR_CURRENT_LIMIT = 80.amps
val SUPPLY_CURRENT_LIMIT = 40.amps
val MOTOR_CONFIG: TalonFXConfiguration =
    TalonFXConfiguration().apply {
        MotorOutput =
            MotorOutputConfigs().apply {
                NeutralMode = NeutralModeValue.Brake
                Inverted = InvertedValue.CounterClockwise_Positive
            }
        Feedback = FeedbackConfigs().apply { RotorToSensorRatio = 1.0 }
        Slot0 = CONTROL_GAINS.slot0Config
        CurrentLimits =
            CurrentLimitsConfigs().apply {
                StatorCurrentLimitEnable = true
                SupplyCurrentLimitEnable = true
                StatorCurrentLimit = STATOR_CURRENT_LIMIT[amps]
                SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT[amps]
            }
        MotionMagic =
            MotionMagicConfigs().apply {
                MotionMagicCruiseVelocity = 60.0
                MotionMagicAcceleration = 120.0
                MotionMagicJerk = 1300.0
            }
    }
