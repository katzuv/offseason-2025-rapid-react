// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

/** IO implementation for real Limelight hardware. */
public class VisionIOLimelight implements VisionIO {
    private final Supplier<Rotation2d> rotationSupplier;
    private final DoubleArrayPublisher orientationPublisher;

    private final DoubleSubscriber latencySubscriber;
    private final DoubleSubscriber txSubscriber;
    private final DoubleSubscriber tySubscriber;
    private final DoubleArraySubscriber megatag1Subscriber;
    private final DoubleArraySubscriber megatag2Subscriber;
    private Supplier<Angle> turretAngle = null;
    /**
     * Creates a new VisionIOLimelight.
     *
     * @param name The configured name of the Limelight.
     * @param rotationSupplier Supplier for the current estimated rotation, used for MegaTag 2.
     */
    public VisionIOLimelight(
            String name, Supplier<Rotation2d> rotationSupplier, Supplier<Angle> turretAngle) {
        var table = NetworkTableInstance.getDefault().getTable(name);
        this.rotationSupplier = rotationSupplier;
        orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
        latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
        txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
        tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
        megatag1Subscriber =
                table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
        megatag2Subscriber =
                table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
    }

    public VisionIOLimelight useTurret(Supplier<Angle> turretAngle) {
        this.turretAngle = turretAngle;
        return this;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // Update connection status based on whether an update has been seen in the last 250ms
        inputs.connected =
                ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250;

        // Update target observation
        inputs.latestTargetObservation =
                new TargetObservation(
                        Rotation2d.fromDegrees(txSubscriber.get()),
                        Rotation2d.fromDegrees(tySubscriber.get()));

        // Update orientation for MegaTag 2
        orientationPublisher.accept(
                new double[] {rotationSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0});
        NetworkTableInstance.getDefault()
                .flush(); // Increases network traffic but recommended by Limelight

        // Read new pose observations from NetworkTables
        Set<Integer> tagIds = new HashSet<>();
        List<PoseObservation> poseObservations = new LinkedList<>();
        for (var rawSample : megatag1Subscriber.readQueue()) {
            if (rawSample.value.length == 0) continue;
            for (int i = 11; i < rawSample.value.length; i += 7) {
                tagIds.add((int) rawSample.value[i]);
            }
            Pose3d pose = parsePose(rawSample.value);
            if (turretAngle != null) pose = rotateByTurret(pose);
            poseObservations.add(
                    new PoseObservation(
                            // Timestamp, based on server timestamp of publish and latency
                            rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,

                            // 3D pose estimate
                            pose,

                            // Ambiguity, using only the first tag because ambiguity isn't
                            // applicable for multitag
                            rawSample.value.length >= 18 ? rawSample.value[17] : 0.0,

                            // Tag count
                            (int) rawSample.value[7],

                            // Average tag distance
                            rawSample.value[9],

                            // Observation type
                            PoseObservationType.MEGATAG_1));
        }
        for (var rawSample : megatag2Subscriber.readQueue()) {
            if (rawSample.value.length == 0) continue;
            for (int i = 11; i < rawSample.value.length; i += 7) {
                tagIds.add((int) rawSample.value[i]);
            }
            Pose3d pose = parsePose(rawSample.value);
            if (turretAngle != null) pose = rotateByTurret(pose);
            poseObservations.add(
                    new PoseObservation(
                            // Timestamp, based on server timestamp of publish and latency
                            rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,

                            // 3D pose estimate
                            pose,

                            // Ambiguity, zeroed because the pose is already disambiguated
                            0.0,

                            // Tag count
                            (int) rawSample.value[7],

                            // Average tag distance
                            rawSample.value[9],

                            // Observation type
                            PoseObservationType.MEGATAG_2));
        }

        // Save pose observations to inputs object
        inputs.poseObservations = new PoseObservation[poseObservations.size()];
        for (int i = 0; i < poseObservations.size(); i++) {
            inputs.poseObservations[i] = poseObservations.get(i);
        }

        // Save tag IDs to inputs objects
        inputs.tagIds = new int[tagIds.size()];
        int i = 0;
        for (int id : tagIds) {
            inputs.tagIds[i++] = id;
        }
    }

    /** Parses the 3D pose from a Limelight botpose array. */
    private static Pose3d parsePose(double[] rawLLArray) {
        return new Pose3d(
                rawLLArray[0],
                rawLLArray[1],
                rawLLArray[2],
                new Rotation3d(
                        Math.toRadians(rawLLArray[3]),
                        Math.toRadians(rawLLArray[4]),
                        Math.toRadians(rawLLArray[5])));
    }

    private Pose3d rotateByTurret(Pose3d fieldRelativePose) {
        return new Pose3d(
                fieldRelativePose
                        .rotateAround(
                                new Translation3d(
                                        Units.Millimeters.zero(),
                                        Units.Millimeters.zero(),
                                        Units.Millimeters.of(441.837)),
                                new Rotation3d(
                                        Units.Rotation.zero(),
                                        Units.Rotation.zero(),
                                        turretAngle.get()))
                        .getTranslation(),
                new Rotation3d(
                        Units.Rotation.zero(),
                        Units.Rotation.zero(),
                        Units.Degrees.of(180).minus(turretAngle.get())));
    }
}
