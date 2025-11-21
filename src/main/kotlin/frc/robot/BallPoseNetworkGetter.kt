package frc.robot

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.StructArraySubscriber

private const val tableKey = "RealsenseVision"
private const val poses3dKey = "/$tableKey/poses"
val ballPosesSubscriber: StructArraySubscriber<Pose3d> by lazy {
    val inst = NetworkTableInstance.getDefault()
    val table = inst.getTable(tableKey)
    table.getStructArrayTopic(poses3dKey, Pose3d.struct).subscribe(arrayOf())
}
val robotRelativeBallPoses
    get() = ballPosesSubscriber.get()
