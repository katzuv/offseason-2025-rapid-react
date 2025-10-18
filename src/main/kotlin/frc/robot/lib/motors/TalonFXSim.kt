package frc.robot.lib.motors

import com.ctre.phoenix6.controls.*
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Voltage
import frc.robot.lib.math.differential.Derivative

class TalonFXSim(
    numMotors: Int,
    gearing: Double,
    jKgMetersSquared: Double,
    conversionFactor: Double,
    talonType: TalonType
) :
    SimMotor(
        talonType.getDCMotor(numMotors),
        jKgMetersSquared,
        gearing,
        conversionFactor
    ) {
    private val acceleration = Derivative()

    override fun update(timestampSeconds: Double) {
        super.update(timestampSeconds)

        acceleration.update(
            this.velocity.`in`(Units.RotationsPerSecond),
            timestampSeconds
        )
    }

    fun setControl(request: DutyCycleOut) {
        setControl(VoltageOut(request.Output * 12))
    }

    fun setControl(request: VoltageOut) {
        voltageRequest = MotorSetpoint.simpleVoltage(request.Output)
    }

    fun setControl(request: PositionDutyCycle) {
        setControl(
            PositionVoltage(request.Position)
                .withFeedForward(request.FeedForward * 12)
        )
    }

    fun setControl(request: PositionVoltage) {
        voltageRequest = MotorSetpoint {
            controller.calculate(this.position, request.Position) +
                request.FeedForward
        }
    }

    fun setControl(request: VelocityDutyCycle) {
        setControl(
            VelocityVoltage(request.Velocity)
                .withFeedForward(request.FeedForward * 12)
        )
    }

    fun setControl(request: VelocityVoltage) {
        voltageRequest = MotorSetpoint {
            (controller.calculate(
                this.velocity.`in`(Units.RotationsPerSecond),
                request.Velocity
            ) + request.FeedForward)
        }
    }

    fun setControl(request: MotionMagicDutyCycle) {
        setControl(
            MotionMagicVoltage(request.Position)
                .withFeedForward(request.FeedForward * 12)
        )
    }

    fun setControl(request: MotionMagicVoltage) {
        voltageRequest = MotorSetpoint {
            (profiledController.calculate(this.position, request.Position) +
                request.FeedForward)
        }
    }

    fun setControl(request: VelocityTorqueCurrentFOC) {
        voltageRequest = MotorSetpoint {
            (controller.calculate(
                this.velocity.`in`(Units.RotationsPerSecond),
                request.Velocity
            ) + (request.FeedForward * 12))
        }
    }

    fun setControl(request: TorqueCurrentFOC) {
        setControl(VoltageOut(request.Output * 12))
    }

    fun setControl(request: PositionTorqueCurrentFOC) {
        voltageRequest = MotorSetpoint {
            (controller.calculate(this.position, request.Position) +
                request.FeedForward * 12)
        }
    }

    fun setControl(request: MotionMagicTorqueCurrentFOC) {
        voltageRequest = MotorSetpoint {
            (profiledController.calculate(this.position, request.Position) +
                request.FeedForward * 12)
        }
    }

    fun setControl(request: MotionMagicVelocityDutyCycle) {
        voltageRequest = MotorSetpoint {
            (profiledController.calculate(
                this.velocity.`in`(Units.RotationsPerSecond),
                request.Velocity
            ) + request.FeedForward * 12)
        }
    }

    fun setControl(request: MotionMagicVelocityTorqueCurrentFOC) {
        voltageRequest = MotorSetpoint {
            (profiledController.calculate(
                this.velocity.`in`(Units.RotationsPerSecond),
                request.Velocity
            ) + request.FeedForward * 12)
        }
    }

    fun setControl(request: ControlRequest) {
        if (request is DutyCycleOut) setControl(request)
        else if (request is Follower)
            return // TODO: Add support for follower requests.
        else if (request is TorqueCurrentFOC) setControl(request)
        else if (request is VoltageOut) setControl(request)
        else if (request is PositionDutyCycle) setControl(request)
        else if (request is PositionVoltage) setControl(request)
        else if (request is PositionTorqueCurrentFOC) setControl(request)
        else if (request is VelocityDutyCycle) setControl(request)
        else if (request is VelocityVoltage) setControl(request)
        else if (request is VelocityTorqueCurrentFOC) setControl(request)
        else if (request is MotionMagicDutyCycle) setControl(request)
        else if (request is MotionMagicVoltage) setControl(request)
        else if (request is MotionMagicTorqueCurrentFOC) setControl(request)
        else if (request is MotionMagicVelocityDutyCycle) setControl(request)
        else if (request is MotionMagicVelocityTorqueCurrentFOC)
            setControl(request)
    }

    val velocity: AngularVelocity
        get() =
            Units.Rotation.per(Units.Minutes)
                .of(motorSim.angularVelocityRPM)
                .times(conversionFactor)

    val position: Double
        get() = motorSim.angularPositionRotations * conversionFactor

    fun getAcceleration(): Double {
        return acceleration.get()
    }

    val appliedCurrent: Current
        get() = Units.Amps.of(motorSim.currentDrawAmps)

    val appliedVoltage: Voltage
        get() = Units.Volts.of(motorSim.inputVoltage)
}
