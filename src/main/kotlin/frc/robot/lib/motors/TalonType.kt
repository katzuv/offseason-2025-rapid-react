package frc.robot.lib.motors

import edu.wpi.first.math.system.plant.DCMotor

enum class TalonType {
    FALCON,
    FALCON_FOC,
    KRAKEN,
    KRAKEN_FOC;

    fun getDCMotor(numMotors: Int): DCMotor =
        when (this) {
            FALCON -> DCMotor.getFalcon500(numMotors)
            FALCON_FOC -> DCMotor.getFalcon500Foc(numMotors)
            KRAKEN -> DCMotor.getKrakenX60(numMotors)
            KRAKEN_FOC -> DCMotor.getKrakenX60Foc(numMotors)
        }
}
