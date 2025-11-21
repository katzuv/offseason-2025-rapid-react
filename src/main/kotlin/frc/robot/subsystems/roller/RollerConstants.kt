package frc.robot.subsystems.roller

import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.wpilibj.I2C
import frc.robot.lib.extensions.sec
import frc.robot.lib.extensions.volts

const val ROLLER_COLOR_SIMILARITY_THRESHOLD = 0.96
const val MOTOR_PORT = 0
const val AUXILIARY_MOTOR_PORT = 1

const val SENSOR_ID = 11
val INTAKE = 8.volts
val OUTTAKE = -INTAKE
val STOP = 0.volts

val SENSOR_DEBOUNCE = 0.2.sec
val COLOR_SENSOR_PORT = I2C.Port.kOnboard

val MOTOR_CONFIG =
    TalonFXConfiguration().apply {
        MotorOutput =
            MotorOutputConfigs().apply {
                Inverted = InvertedValue.Clockwise_Positive
            }
    }
