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
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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

  // stuff
  private Optional<EstimatedRobotPose> raw_estimate = Optional.empty();
  private Optional<PhotonPipelineResult> raw_results = Optional.empty();
  private Optional<Pair<EstimatedRobotPose, Matrix<N3, N1>>> estimate = Optional.empty();
  private List<Integer> visible_fiducials = new ArrayList<>();


  public PhotonTest() {

    // camera and pose estimator
    m_camera = new PhotonCamera("camera");
    m_camerape = new PhotonPoseEstimator(
        m_fieldlayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        k_camera1_intrinsics);

    // IT'S SIMULATING TIME
    if (RobotBase.isSimulation()) {

      // declare vision sim
      m_visionsim = new VisionSystemSim("main");
      m_visionsim.addAprilTags(m_fieldlayout);

      // set up camera sim and properties
      SimCameraProperties camera1sim_properties = new SimCameraProperties();
      camera1sim_properties.setFPS(30);
      m_camerasim = new PhotonCameraSim(m_camera, camera1sim_properties);
      m_visionsim.addCamera(m_camerasim, k_camera1_intrinsics);

      // enable wireframe and raw stream
      m_camerasim.enableRawStream(false);
      m_camerasim.enableDrawWireframe(true);
    }
  }




  // main functions for estimate and heuristics

  /** Returns the most recent estimate, with standard deviations based on heuristics. Still a WIP. */
  public Optional<Pair<EstimatedRobotPose, Matrix<N3, N1>>> getEstimateWithStdDevs() {
    estimate = Optional.empty();
    raw_estimate.ifPresent(est -> {
      estimate = Optional
          .of(new Pair<EstimatedRobotPose, Matrix<N3, N1>>(est, calculateStdDevs(est)));
    });
    return estimate;
  }

  /** Calculates the appropriate standard deviations for the passed estimate. Still a WIP. */
  public Matrix<N3, N1> calculateStdDevs(EstimatedRobotPose est) {
    Matrix<N3, N1> stddevs = k_stdDevs_ignore;
    return stddevs;
  }

  





  // functions used in simulation and testing

  /** Used to update visible fiducials in Telemetry. */
  public List<Integer> getFiducials() {
    raw_results.ifPresent(result -> {
      visible_fiducials.clear();
      for (var fiducial : result.targets) {
        visible_fiducials.add(fiducial.fiducialId);
      }
    });
    return visible_fiducials;
  }

  /** Used to update the pose of the vision sim periodically. */
  public void updatePose(Pose2d pose) {
    m_visionsim.update(pose);
  }




  @Override
  public void periodic() {

    // empty and update
    raw_results = Optional.empty();
    for (var results : m_camera.getAllUnreadResults()) {
      raw_results = Optional.of(results);
    }
  }
}
