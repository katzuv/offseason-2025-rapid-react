package frc.robot.subsystems.shooter.hopper

import com.ctre.phoenix6.configs.CANrangeConfiguration
import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.FovParamsConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.ProximityParamsConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.Current
import edu.wpi.first.wpilibj.I2C
import edu.wpi.first.wpilibj.util.Color
import frc.robot.lib.extensions.*

const val MOTOR_ID = 4

val SLOW_BACK_TIMEOUT = 0.15.sec
val INTAKE_VOLTAGE = 3.0.volts
val SLOW_BACK_VOLTAGE = (-1.2).volts
val SHOOT_VOLTAGE = 8.volts
val STATOR_LIMIT = 30.amps

val SUPPLY_LIMIT: Current = STATOR_LIMIT * 2.0
val MOTOR_CONFIG =
    TalonFXConfiguration().apply {
        MotorOutput =
            MotorOutputConfigs().apply {
                Inverted = InvertedValue.Clockwise_Positive
                NeutralMode = NeutralModeValue.Brake
            }
        CurrentLimits =
            CurrentLimitsConfigs().apply {
                StatorCurrentLimitEnable = true
                SupplyCurrentLimitEnable = true
                StatorCurrentLimit = STATOR_LIMIT[amps]
                SupplyCurrentLimit = SUPPLY_LIMIT[amps]
            }
    }

val DISTANCE_SENSOR_ID = 12
val MIN_SIGNAL_STRENGTH = 55000.0
private val DISTANCE_THRESHOLD = 9.cm

val CANRANGE_CONFIG =
    CANrangeConfiguration().apply {
        ProximityParams =
            ProximityParamsConfigs().apply {
                ProximityThreshold = DISTANCE_THRESHOLD[m]
                MinSignalStrengthForValidMeasurement = MIN_SIGNAL_STRENGTH
            }
        FovParams =
            FovParamsConfigs().apply {
                FOVRangeX = 7.0
                FOVRangeY = 7.0
            }
    }

val RED_COLOR = Color(138, 87, 29)
val BLUE_COLOR = Color(73, 147, 33)
const val HOPPER_COLOR_SIMILARITY_THRESHOLD = 0.97

val COLOR_SENSOR_PORT = I2C.Port.kMXP
