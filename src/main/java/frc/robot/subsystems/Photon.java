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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.utils.Constants.VisionConstants.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;

public class Photon extends SubsystemBase {

  // cameras and estimators
  private final PhotonCamera m_cameraleft;
  private final PhotonCamera m_cameraright;
  private final PhotonPoseEstimator m_cameraleft_estimator;
  private final PhotonPoseEstimator m_cameraright_estimator;

  // simulation stuff
  private PhotonCameraSim m_cameraleftsim;
  private PhotonCameraSim m_camerarightsim;
  private VisionSystemSim m_visionsim;

  // the results wrapped in optionals to limit NPEs
  private Optional<PhotonPipelineResult> cameraleft_results = Optional.empty();
  private Optional<PhotonPipelineResult> cameraright_results = Optional.empty();

  // the estimates wrapped in optionals to limit NPEs
  private Optional<EstimatedRobotPose> cameraleft_estimate = Optional.empty();
  private Optional<EstimatedRobotPose> cameraright_estimate = Optional.empty();

  // visible fiducials list
  private HashSet<Integer> visible_fiducials_all = new HashSet<>();

  public Photon() {

    // left camera and estimator
    m_cameraleft = new PhotonCamera("camera_left");
    m_cameraleft_estimator = new PhotonPoseEstimator(
        k_fieldlayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        k_cameraleft_intrinsics);
    m_cameraleft_estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    // right camera and estimator
    m_cameraright = new PhotonCamera("camera_right");
    m_cameraright_estimator = new PhotonPoseEstimator(
        k_fieldlayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        k_cameraright_intrinsics);
    m_cameraright_estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    // IT'S SIMULATIN TIME
    if (RobotBase.isSimulation()) {

      // declare vision sim
      m_visionsim = new VisionSystemSim("main");
      m_visionsim.addAprilTags(k_fieldlayout);

      // set up cameraleft sim and properties
      SimCameraProperties cameraleftsim_properties = new SimCameraProperties();
      cameraleftsim_properties.setFPS(30);
      m_cameraleftsim = new PhotonCameraSim(m_cameraleft, cameraleftsim_properties);
      m_visionsim.addCamera(m_cameraleftsim, k_cameraleft_intrinsics);
      m_cameraleftsim.enableRawStream(false);
      m_cameraleftsim.enableDrawWireframe(false);

      // set up cameraright sim and properties
      SimCameraProperties camerarightsim_properties = new SimCameraProperties();
      camerarightsim_properties.setFPS(30);
      m_camerarightsim = new PhotonCameraSim(m_cameraright, camerarightsim_properties);
      m_visionsim.addCamera(m_camerarightsim, k_cameraright_intrinsics);
      m_camerarightsim.enableRawStream(false);
      m_camerarightsim.enableDrawWireframe(false);
    }
  }







  /** Used to update visible fiducials in Telemetry.
   * 
   * @param results The results.
   * @return List of integers containing the visible targets in the results.
  */
  public List<Integer> getFiducials(Optional<PhotonPipelineResult> results) {
    List<Integer> visible_fiducials = new ArrayList<>(); //also clears list
    results.ifPresent(res -> {
      for (var target : res.targets) {
        visible_fiducials.add(target.fiducialId);
      }
    });
    return visible_fiducials;
  }








  /** Get left camera results. */
  public Optional<PhotonPipelineResult> getLeftResults() {
    return cameraleft_results;
  }

  /** Get right camera results. */
  public Optional<PhotonPipelineResult> getRightResults() {
    return cameraright_results;
  }






  /** Get the estimates in list format.
   * 
   * @return
   */
  public List<Optional<EstimatedRobotPose>> getEstimates() {
    return Arrays.asList(
      cameraleft_estimate,
      cameraright_estimate
    );
  }







  /** Used to update the pose of the vision sim periodically.
   * Possibly not necessary when real. Testing required.
   * 
   * @param pose The odometric pose of the robot.
  */
  public void updatePose(Pose2d pose) {
    m_visionsim.update(pose);
  }









  @Override
  public void periodic() {

    // clear results and update them
    for (var result : m_cameraleft.getAllUnreadResults()) {
      cameraleft_results = Optional.of(result);
    }
    for (var result : m_cameraright.getAllUnreadResults()) {
      cameraright_results = Optional.of(result);
    }

    // clear estimates and update them
    cameraleft_results.ifPresentOrElse(res -> {
      cameraleft_estimate = m_cameraleft_estimator.update(res);
    }, () -> cameraleft_estimate = Optional.empty());
    cameraright_results.ifPresentOrElse(res -> {
      cameraright_estimate = m_cameraright_estimator.update(res);
    }, () -> cameraright_estimate = Optional.empty());
  }
}
