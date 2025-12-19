// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

import static frc.robot.utils.Constants.VisionConstants.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class Photon extends SubsystemBase {

  // camera 1, front left
  private final PhotonCamera m_camera;
  private final PhotonPoseEstimator m_cameraestimator;

  // simulation stuff
  private PhotonCameraSim m_camerasim;
  private VisionSystemSim m_visionsim;

  // the results wrapped in an optional to limit NPEs
  private Optional<PhotonPipelineResult> camera_results = Optional.empty();
  private Optional<EstimatedRobotPose> camera_estimate = Optional.empty();
  private List<Integer> target_blacklist = new ArrayList<>();
  private List<Double> target_yaws = new ArrayList<>();
  private List<Double> target_distances = new ArrayList<>();

  // persistent list of visible active fiducials
  private List<Integer> visible_fiducials = new ArrayList<>();

  public Photon() {

    // camera and pose estimator
    m_camera = new PhotonCamera("camera");
    m_cameraestimator = new PhotonPoseEstimator(
        k_fieldlayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        k_camera1_intrinsics);

    // IT'S SIMULATING TIME
    if (RobotBase.isSimulation()) {

      // declare vision sim
      m_visionsim = new VisionSystemSim("main");
      m_visionsim.addAprilTags(k_fieldlayout);

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




  /** Used to update the blacklist for the cameras periodically. */
  public void updateBlacklist(List<Integer> blacklist) {
    target_blacklist = blacklist;
  }






  /** Used to update and modify result based on desired heurstics. */
  // NOTES
  public PhotonPipelineResult runBlacklist(PhotonPipelineResult result) {
    // make a mutable copy of the original result
    PhotonPipelineResult result_mutated = new PhotonPipelineResult(
        result.metadata,
        result.targets,
        result.multitagResult);
    // modify said result with blacklist
    for (var target : result.targets) {
      if (target_blacklist.contains(target.fiducialId)) {
        result_mutated.targets.remove(result_mutated.targets.indexOf(target));
      }
    }
    // pass modified result
    return result_mutated;
  }





  /** Used to call the current estimate pose from the camera system. */
  public Optional<EstimatedRobotPose> getEstimate() {
    // return the estimate, duh
    return camera_estimate;
  }








  /** Used to update visible fiducials in Telemetry. */
  public List<Integer> getFiducials() {
    camera_results.ifPresent(result -> {
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

    // empty and update results, with blacklisting
    camera_results = Optional.empty();
    for (var results : m_camera.getAllUnreadResults()) {
      camera_results = Optional.of(runBlacklist(results));
    }

    // empty and update stuff
    camera_estimate = Optional.empty();
    target_yaws.clear();
    target_distances.clear();
    camera_results.ifPresent(res -> {
      for (var target : res.targets) {
        target_yaws.add(target.getYaw());
        target_distances.add(PhotonUtils.calculateDistanceToTargetMeters(
          Constants.VisionConstants.k_camera1_intrinsics.getZ(),
          AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark).getTagPose(target.fiducialId).get().getZ(),
          Constants.VisionConstants.k_camera1_intrinsics.getRotation().getY(),
          target.pitch
        ));
      }
      SmartDashboard.putNumberArray("target yaws", target_yaws.stream().mapToDouble(i -> i).toArray());
      SmartDashboard.putNumberArray("target distances", target_distances.stream().mapToDouble(i -> i).toArray());
      camera_estimate = m_cameraestimator.update(res);

    });


  }
}
