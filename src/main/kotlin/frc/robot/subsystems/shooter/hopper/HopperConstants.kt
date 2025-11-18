package frc.robot.subsystems.shooter.hopper

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.units.measure.Current
import edu.wpi.first.wpilibj.I2C
import edu.wpi.first.wpilibj.util.Color
import frc.robot.lib.extensions.*

const val MOTOR_ID = 4

val SLOW_BACK_TIMEOUT = 0.2.sec
val INTAKE_VOLTAGE = 3
    .volts
val SLOW_BACK_VOLTAGE = (-1.7).volts
val SHOOT_VOLTAGE = 8.volts
val STATOR_LIMIT = 30.amps

val SUPPLY_LIMIT: Current = STATOR_LIMIT * 2.0
val MOTOR_CONFIG =
    TalonFXConfiguration().apply {
        MotorOutput =
            MotorOutputConfigs().apply {
                Inverted = InvertedValue.Clockwise_Positive
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

val DISTANCE_THRESHOLD = 50.mm

val RED_COLOR = Color(138, 87, 29)
val BLUE_COLOR = Color(73, 147, 33)
const val SIMILARITY_THRESHOLD = 0.93

val COLOR_SENSOR_PORT = I2C.Port.kMXP
