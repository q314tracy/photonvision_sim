// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {
    public class SwerveDriveConstants {
        public static final double k_wheelradius = Units.inchesToMeters(2);
        public static final double k_wheelcircumference = 2 * Math.PI * k_wheelradius;
        public static final double k_trackwidth = Units.inchesToMeters(24); // need to update with proper distance to center
        public static final double k_drivegearratio = 5.27;
        public static final double k_turngearratio = 26;
        public static final double k_drivemotormaxRPM = 6784;
        public static final double k_maxlinspeed = (k_drivemotormaxRPM / k_drivegearratio) * k_wheelcircumference / 60; // meters/sec
        public static final double k_maxrotspeed = (2 * k_maxlinspeed) / k_trackwidth;
        public static final Pose2d k_initpose = new Pose2d(2, 2, new Rotation2d());
    }
    
    public class OIConstants {
        public static final int k_joystickport = 0;
        public static final double k_maxlinspeedteleop = 3;
        public static final double k_maxrotspeedteleop = 2 * Math.PI;
    }

    public class VisionConstants {
        public static final Transform3d k_frontleftcameraintrinsics = new Transform3d(
                Units.inchesToMeters(12),
                Units.inchesToMeters(12),
                Units.inchesToMeters(8),
                new Rotation3d(
                        0,
                        0,
                        Units.degreesToRadians(45)));
        public static final Transform3d k_frontcentercameraintrinsics = new Transform3d(
                Units.inchesToMeters(12),
                0,
                Units.inchesToMeters(8),
                new Rotation3d(
                        0,
                        0,
                        0));
        public static final Transform3d k_frontrightcameraintrinsics = new Transform3d(
                Units.inchesToMeters(12),
                Units.inchesToMeters(-12),
                Units.inchesToMeters(8),
                new Rotation3d(
                        0,
                        0,
                        Units.degreesToRadians(-45)));
        public static final Transform3d k_rearleftcameraintrinsics = new Transform3d(
                Units.inchesToMeters(-12),
                Units.inchesToMeters(12),
                Units.inchesToMeters(8),
                new Rotation3d(
                        0,
                        0,
                        Units.degreesToRadians(135)));
        public static final Transform3d k_rearrightcameraintrinsics = new Transform3d(
                Units.inchesToMeters(-12),
                Units.inchesToMeters(-12),
                Units.inchesToMeters(8),
                new Rotation3d(
                        0,
                        0,
                        Units.degreesToRadians(-135)));

        public static final AprilTagFieldLayout k_fieldlayout = AprilTagFieldLayout
                .loadField(AprilTagFields.k2025ReefscapeAndyMark);

        public static final List<Integer> k_tagblacklist = Arrays.asList(3, 16);

        public static final Matrix<N3, N1> k_singletagstddevs = VecBuilder.fill(2, 2, Double.MAX_VALUE);
        public static final Matrix<N3, N1> k_multitagstddevs = VecBuilder.fill(0.5, 0.5, Double.MAX_VALUE);
        public static final Matrix<N3, N1> k_ignorestddevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE,
                Double.MAX_VALUE);
    }
}
