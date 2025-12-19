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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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

  private final SendableChooser<Boolean> m_tagenable = new SendableChooser<>();
  private final SendableChooser<Integer> m_tagchooser = new SendableChooser<>();
  private List<Integer> tagID_blacklist = new ArrayList<>();

  public Telemetry(Drive drive, Photon photon) {
    m_drive = drive;
    m_photon = photon;

    m_taglayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    m_field = new Field2d();
    m_visionpose = m_field.getObject("vision pose");
    SmartDashboard.putData(m_field);

    // tag blacklist sendable choosers setup
    m_tagenable.setDefaultOption("Enable Tag", false);
    m_tagenable.addOption("Disable Tag", true);
    SmartDashboard.putData(m_tagenable);
    m_tagchooser.setDefaultOption("Tag# 1", 1);
    for (int i = 2; i <= 22; i++) {
      m_tagchooser.addOption("Tag# " + i, i);
    }
    SmartDashboard.putData(m_tagchooser);
  }

  // basically adds and removes fiducials from field2d to visualize visible
  // fiducials
  public void addFiducialstoField(List<Integer> fiducials_list) {

    // check if empty
    if (fiducials_list.size() > 0) {
      // if not, check if tag is already displayed since last check
      for (var fiducial : fiducials_list) {
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
      if (!fiducials_list.contains(tag)) {
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







  public List<Integer> getBlacklist() {
    // update current selected tag
    int tagID = m_tagchooser.getSelected();

    //check if tag enable is active and tag is not currently in list, if yes the add tag ID
    if (m_tagenable.getSelected() && !tagID_blacklist.contains(tagID)) {
      tagID_blacklist.add(tagID);
    }

    // check if tag is slated for removal and is contained within list, if yes, remove tag from list
    if (!m_tagenable.getSelected() && tagID_blacklist.contains(tagID)) {
      tagID_blacklist.remove(tagID_blacklist.indexOf(tagID));
    }
    //return the list
    return tagID_blacklist;
  }







  @Override
  public void periodic() {

    // update poses
    m_field.setRobotPose(m_drive.getOdometricPose()); // odometric pose
    m_visionpose.setPose(m_drive.getEstimatedPose()); // vision fused pose

    // add fiducial ids to visualization
    addFiducialstoField(m_photon.getFiducials());
    SmartDashboard.putNumberArray("fiducials",
        m_photon.getFiducials().stream().mapToDouble(i -> i.doubleValue()).toArray());
    SmartDashboard.putNumberArray("Tag Blacklist", getBlacklist().stream().mapToDouble(i -> i.doubleValue()).toArray());

  }
}
