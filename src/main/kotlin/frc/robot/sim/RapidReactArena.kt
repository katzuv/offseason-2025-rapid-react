package frc.robot.sim

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import org.dyn4j.geometry.Circle
import org.ironmaple.simulation.SimulatedArena

class RapidReactArena : SimulatedArena(RapidReactFieldObstacleMap()) {

    class RapidReactFieldObstacleMap : FieldMap() {
        init {
            // blue wall
            super.addBorderLine(
                Translation2d(0.0, 1.270),
                Translation2d(0.0, 6.782)
            )

            // red wall
            super.addBorderLine(
                Translation2d(17.548, 1.270),
                Translation2d(17.548, 6.782)
            )

            // upper walls
            super.addBorderLine(
                Translation2d(1.672, 8.052),
                Translation2d(17.548 - 1.672, 8.052)
            )

            // lower walls
            super.addBorderLine(
                Translation2d(1.672, 0.0),
                Translation2d(17.548 - 1.672, 0.0)
            )

            super.addCustomObstacle(
                Circle(0.75),
                Pose2d(8.774, 4.026, Rotation2d())
            )
        }
    }

    override fun placeGamePiecesOnField() {}

    @Synchronized
    override fun getGamePiecesByType(type: String): MutableList<Pose3d?> =
        super.getGamePiecesByType(type)

    @Synchronized override fun clearGamePieces() {}
}
