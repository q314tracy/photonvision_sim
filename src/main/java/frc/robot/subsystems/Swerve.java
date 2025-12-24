// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.utils.Constants.SwerveDriveConstants.*;

import java.io.File;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class Swerve extends SubsystemBase {

  public final SwerveDrive m_swervedrive;

  public Swerve() {

    // attempt construction of YAGSL swerve object
    try {
      m_swervedrive = new SwerveParser(
        new File(Filesystem.getDeployDirectory(), "swerve"))
        .createSwerveDrive(k_maxlinspeed, k_initpose);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
  }

  /** Returns the current swerve drive instance. */
  public Field2d getField2d() {
    return m_swervedrive.field;
  }

  /** Returns the current field-relative chassis speeds. */
  public ChassisSpeeds getFieldSpeeds() {
    return m_swervedrive.getFieldVelocity();
  }

  /**
   * Basic field oriented driving method.
   * @param speeds Composed field-relative speeds.
   */
  public void drive(ChassisSpeeds speeds) {
    m_swervedrive.driveFieldOriented(speeds);
  }

  /** Returns the current pose of the robot. */
  public Pose2d getPose() {
    return m_swervedrive.getPose();
  }

  @Override
  public void periodic() {
  }
}
