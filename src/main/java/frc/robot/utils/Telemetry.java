// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Photon;

public class Telemetry extends SubsystemBase {

  private final Drive m_drive;
  private final Photon m_photon;

  private final Field2d m_field;
  private final AprilTagFieldLayout m_taglayout;
  private final FieldObject2d m_visionpose;

  private List<Integer> displayed_tags = new ArrayList<>();
  private List<Integer> removed_tags = new ArrayList<>();

  public Telemetry(Drive drive, Photon photon) {
    m_drive = drive;
    m_photon = photon;

    m_taglayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    m_field = new Field2d();
    m_visionpose = m_field.getObject("vision pose");
    SmartDashboard.putData(m_field);
  }

  public void addFiducialstoField(List<Integer> fiducials) {

    // check if empty
    if (fiducials.size() > 0) {
      // if not, check if tag is already displayed since last check
      for (var fiducial : fiducials) {
        if (!displayed_tags.contains(fiducial)) {
          // add the tag if yiss
          m_field.getObject("AprilTag " + fiducial).setPose(
              m_taglayout.getTagPose(fiducial).get().getX(),
              m_taglayout.getTagPose(fiducial).get().getY(),
              new Rotation2d());
          // record that we added it
          displayed_tags.add(fiducial);
        }
      }
    }

    // queue tags to be removed
    for (var tag : displayed_tags) {
      // check if fiducial list contains current checked tag
      // if returns true, add entry to removal queue
      if (!fiducials.contains(tag)) {
        removed_tags.add(tag);
      }
    }

    // remove said tags from field and display list and clear queue
    if (removed_tags.size() != 0) {
      // remove tags
      for (var tag : removed_tags) {
        // "remove" object and remove entry
        m_field.getObject("AprilTag " + tag).setPose(new Pose2d(50, 50, new Rotation2d()));
        displayed_tags.remove(displayed_tags.indexOf(tag));
      }
    }
    removed_tags.clear();
  }


  @Override
  public void periodic() {

    // update sim pose if simulation
    if (RobotBase.isSimulation()) {
      m_field.setRobotPose(m_drive.getSimPose()); // odometric pose
    }

    // update vision pose on field
    m_visionpose.setPose(m_drive.getEstimatedPose()); // vision fused pose

    // add fiducial ids to visualization
    addFiducialstoField(m_photon.getAllFiducials());
    SmartDashboard.putNumberArray("visible ficuials", m_photon.getAllFiducials().stream().mapToDouble(i -> i.doubleValue()).toArray());
  }
}
