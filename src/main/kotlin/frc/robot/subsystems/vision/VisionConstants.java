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

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

public class VisionConstants {
    public static String LOG_PREFIX = "Subsystems/Vision/";
    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout;

    static {
        try {
            aprilTagLayout =
                    new AprilTagFieldLayout(
                            new File(Filesystem.getDeployDirectory(), "apriltag-locations.json")
                                    .toPath());
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    // Camera names, must match names configured on coprocessor
    public static String turretOVName = "turret";

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)
    // TODO: Check this!!!!!!!!!!
    public static Transform3d robotToTurretCamera =
            new Transform3d(
                    Millimeter.of(0.0),
                    Millimeter.of(183.105966196),
                    Millimeter.of(697.57321),
                    new Rotation3d(Degrees.of(0.0), Degrees.of(15.0), Degrees.of(180)));

    public static Map<String, Transform3d> OVNameToTransform =
            new HashMap<>() {
                {
                    put(turretOVName, robotToTurretCamera);
                }
            };

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.3;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors =
            new double[] {
                1.0, // OV1
                1.0, // OV2
                1.0 // OV3
            };

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor =
            Double.POSITIVE_INFINITY; // No rotation data available

    public static int frontCameraIndex = 1;

    public static Distance MAX_DELTA_BETWEEN_LOCAL_AND_GLOBAL = Units.Meters.of(1.5);
}
