package frc.robot.subsystems.shooter.hood

import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import edu.wpi.first.units.measure.Current
import edu.wpi.first.wpilibj.Filesystem
import frc.robot.lib.TunableGains
import frc.robot.lib.extensions.amps
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.rot
import frc.robot.lib.math.interpolation.InterpolatingDoubleMap
import frc.robot.lib.shooting.ShootingTableReader

const val MOTOR_ID = 3

val SETPOINT_TOLERANCE = 0.5.deg

val HOOD_ANGLE_BY_DISTANCE: InterpolatingDoubleMap =
    ShootingTableReader.parse(
        Filesystem.getDeployDirectory().path + "/shootData/distanceToAngle.csv"
    )

val STATOR_LIMIT = 30.amps
val SUPPLY_LIMIT: Current = STATOR_LIMIT * 2.0
val CONTROL_GAINS =
    TunableGains(
        "Hood",
        kP = 100.0,
        kD = 0.1,
        kS = 0.3,
        kV = 7.4,
        kA = 0.02,
        kG = 0.2
    )

val ENCODER_OFFSET = 294.1115625.deg

const val ENCODER_ID = 10
const val ENCODER_TO_MECHANISM_RATIO = 1.0
const val MOTOR_TO_MECHANISM_RATIO = 59.5
const val MOTOR_TO_SENSOR_RATIO = MOTOR_TO_MECHANISM_RATIO

val MOTOR_CONFIG =
    TalonFXConfiguration().apply {
        MotorOutput =
            MotorOutputConfigs().apply {
                Inverted = InvertedValue.Clockwise_Positive
                NeutralMode = NeutralModeValue.Brake
            }
        Slot0 = CONTROL_GAINS.slot0Config
        CurrentLimits =
            CurrentLimitsConfigs().apply {
                StatorCurrentLimitEnable = true
                SupplyCurrentLimitEnable = true
                StatorCurrentLimit = STATOR_LIMIT[amps]
                SupplyCurrentLimit = SUPPLY_LIMIT[amps]
            }

        Feedback =
            FeedbackConfigs().apply {
                FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder
                FeedbackRemoteSensorID = ENCODER_ID
                SensorToMechanismRatio = ENCODER_TO_MECHANISM_RATIO
                RotorToSensorRatio = MOTOR_TO_SENSOR_RATIO
            }
        SoftwareLimitSwitch =
            SoftwareLimitSwitchConfigs().apply {
                ForwardSoftLimitEnable = true
                ForwardSoftLimitThreshold = 0.055
                ReverseSoftLimitEnable = true
                ReverseSoftLimitThreshold = -0.0001
            }
    }

val ENCODER_CONFIG =
    CANcoderConfiguration().apply {
        MagnetSensor.SensorDirection =
            SensorDirectionValue.CounterClockwise_Positive
        MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.9
        MagnetSensor.MagnetOffset = ENCODER_OFFSET[rot]
    }
