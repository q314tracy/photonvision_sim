// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.utils.Constants.VisionConstants.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class PhotonTest extends SubsystemBase {

  // camera 1, front left
  private final PhotonCamera m_camera;
  private final PhotonPoseEstimator m_camerape;

  // simulation stuff
  private PhotonCameraSim m_camerasim;
  private VisionSystemSim m_visionsim;

  // tag layout
  private final AprilTagFieldLayout m_fieldlayout = AprilTagFieldLayout
      .loadField(AprilTagFields.k2025ReefscapeAndyMark);

  private Optional<EstimatedRobotPose> camera_estimate = Optional.empty();
  private Optional<PhotonPipelineResult> camera_results = Optional.empty();
  private List<Integer> fiducials = new ArrayList<>();

  public PhotonTest() {

    // camera and pose estimator
    m_camera = new PhotonCamera("camera 1");
    m_camerape = new PhotonPoseEstimator(
        m_fieldlayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        k_camera1_intrinsics);

    // simulation time
    if (RobotBase.isSimulation()) {

      // declare vision sim
      m_visionsim = new VisionSystemSim("main");
      m_visionsim.addAprilTags(m_fieldlayout);

      // set up camera 1 sim and properties
      SimCameraProperties camera1sim_properties = new SimCameraProperties();
      camera1sim_properties.setFPS(30);
      m_camerasim = new PhotonCameraSim(m_camera, camera1sim_properties);
      m_visionsim.addCamera(m_camerasim, k_camera1_intrinsics);

      // enable wireframe and raw stream
      m_camerasim.enableRawStream(false);
      m_camerasim.enableDrawWireframe(true);
    }
  }

  public Optional<EstimatedRobotPose> getEstimate() {
    return camera_estimate;
  }

  public Optional<PhotonPipelineResult> getResults() {
    return camera_results;
  }

  public List<Integer> getFiducials() {
    // return fiducials.stream().mapToInt(Integer::intValue).toArray();
    return fiducials;
  }

  // need to update periodically
  public void updatePose(Pose2d pose) {
    m_visionsim.update(pose);
  }

  @Override
  public void periodic() {

    // update estimate result
    camera_estimate = Optional.empty();
    camera_results = Optional.empty();
    for (var change : m_camera.getAllUnreadResults()) {
      camera_results = Optional.of(change);
      camera_estimate = m_camerape.update(change);
    }

    // check results and get fiducial ids in a list

    getResults().ifPresent(result -> {
      fiducials.clear();
      for (var fiducial : result.targets) {
        fiducials.add(fiducial.fiducialId);
      }
    });
  }
}
