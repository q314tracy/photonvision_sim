// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import static frc.robot.utils.Constants.DriveConstants.k_maxlinspeed;

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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {

    public class DriveConstants {
        public static final int k_leftPWMchannel = 0;
        public static final int k_rightPWMchannel = 1;
        public static final int[] k_leftENCchannels = new int[] { 0, 1 };
        public static final int[] k_rightENCchannels = new int[] { 2, 3 };

        public static final double k_ENCresolution = 4096; // counts per rev

        public static final double k_wheelradius = Units.inchesToMeters(3);
        public static final double k_wheelcircumference = Units.inchesToMeters(2 * Math.PI * k_wheelradius);
        public static final double k_trackwidth = Units.inchesToMeters(30);
        public static final double k_trackcircumference = Math.PI * k_trackwidth;

        public static final double k_gearratio = 10.71; // ratio of gearboxes
        public static final double k_NEOmaxspeed = 5676; // max rpm of drive motor
        public static final double k_maxlinspeed = (5676 / k_gearratio) * (2 * Math.PI * k_wheelradius) / 60; // meters/sec
        public static final double k_maxrotspeed = (2 * k_maxlinspeed) / k_trackwidth; // rads/sec

        public static final Pose2d k_initpose = new Pose2d(2, 2, new Rotation2d());

        public static final double k_headingPIDkP = 3;
        public static final double k_headingPIDkD = 0;

        public static final double k_distancePIDkP = 3;
        public static final double k_distancePIDkD = 0;

        public static final TrapezoidProfile.Constraints k_headingconstraints = new TrapezoidProfile.Constraints(
                2 * Math.PI, 4 * Math.PI);
        public static final TrapezoidProfile.Constraints k_drivingconstraints = new TrapezoidProfile.Constraints(1, 3);

        // SIMULATION CONSTANTS ONLY
        public static final double k_kVlinsim = 12 / k_maxlinspeed;
        public static final double k_kAlinsim = 0.31; // from recalc
        public static final double k_kVrotsim = 12 / k_maxlinspeed;
        public static final double k_kArotsim = 0.4; // estimated
    }

    public class OIConstants {
        public static final int k_joystickport = 0;
        public static final double k_maxlinspeedteleop = k_maxlinspeed;
        public static final double k_maxrotspeedteleop = 2 * Math.PI;
    }

    public class VisionConstants {
        public static final Transform3d k_cameraleft_intrinsics = new Transform3d(
                Units.inchesToMeters(12),
                Units.inchesToMeters(12),
                Units.inchesToMeters(8),
                new Rotation3d(
                        0,
                        0,
                        Units.degreesToRadians(30)));
        public static final Transform3d k_cameraright_intrinsics = new Transform3d(
                Units.inchesToMeters(12),
                Units.inchesToMeters(-12),
                Units.inchesToMeters(8),
                new Rotation3d(
                        0,
                        0,
                        Units.degreesToRadians(-30)));

        public static final AprilTagFieldLayout k_fieldlayout = AprilTagFieldLayout
                .loadField(AprilTagFields.k2025ReefscapeAndyMark);

        public static final List<Integer> k_tagblacklist = Arrays.asList(3, 16);

        public static final Matrix<N3, N1> k_singletagstddevs = VecBuilder.fill(2, 2, Double.MAX_VALUE);
        public static final Matrix<N3, N1> k_multitagstddevs = VecBuilder.fill(0.5, 0.5, Double.MAX_VALUE);
        public static final Matrix<N3, N1> k_ignorestddevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE,
                Double.MAX_VALUE);
    }
}
