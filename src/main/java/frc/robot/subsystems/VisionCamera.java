// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.utils.Constants.VisionConstants.k_fieldlayout;
import static frc.robot.utils.Constants.VisionConstants.k_ignorestddevs;
import static frc.robot.utils.Constants.VisionConstants.k_multitagstddevs;
import static frc.robot.utils.Constants.VisionConstants.k_singletagstddevs;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionCamera extends SubsystemBase {

  private final PhotonCamera m_camera;
  private final PhotonPoseEstimator m_estimator;
  private PhotonCameraSim m_camerasim;

  private Optional<PhotonPipelineResult> results = Optional.empty();
  private Pair<Optional<EstimatedRobotPose>, Matrix<N3, N1>> estimate = Pair.of(Optional.empty(), k_ignorestddevs);

  public VisionCamera(String name, Transform3d intrinsics) {

    m_camera = new PhotonCamera(name);
    m_estimator = new PhotonPoseEstimator(
        k_fieldlayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        intrinsics);
    m_estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    if (RobotBase.isSimulation()) {
      SimCameraProperties camerasim_properties = new SimCameraProperties();
      camerasim_properties.setFPS(30);
      m_camerasim = new PhotonCameraSim(m_camera, camerasim_properties);
      m_camerasim.enableRawStream(false);
      m_camerasim.enableDrawWireframe(false);
    }
  }

  /**
   * Used to update visible fiducials in Telemetry.
   * 
   * @return List of integers containing the visible targets in the results.
   */
  public List<Integer> getFiducials() {
    List<Integer> visible_fiducials = new ArrayList<>(); // also clears list
    results.ifPresent(res -> {
      for (var target : res.targets) {
        visible_fiducials.add(target.fiducialId);
      }
    });
    return visible_fiducials;
  }

  /**
   * Use to calculate the std devs for the resultant estimate. Used internally only.
   * @param estimate The estimate.
   * @param result The results.
   * @return The calculated standard deviations.
   */
  private Matrix<N3, N1> calculateStdDevs(EstimatedRobotPose estimate, PhotonPipelineResult result) {

    // zero out vars to recalculate avg distance and std devs
    double avgDist = 0;
    double avgAngle = 0;
    int numTags = 0;
    Matrix<N3, N1> stddevs = k_ignorestddevs;

    // iterate on target list to average distances to targets
    for (var target : result.targets) {
      avgDist += AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark)
          .getTagPose(target.fiducialId)
          .get()
          .toPose2d()
          .getTranslation()
          .getDistance(estimate.estimatedPose.toPose2d().getTranslation());
      avgAngle += Units.radiansToDegrees(MathUtil.angleModulus(target.getBestCameraToTarget().getRotation().getZ() - Math.PI));
      numTags++;
    }
    avgDist /= numTags;
    avgAngle /= numTags;

    // heuristic logic
    if (numTags > 1) {
      stddevs = k_multitagstddevs.times((Math.pow(Math.abs(avgAngle), 2) / 30) * (Math.pow(avgDist, 2) / 30));
    }
    else if (numTags == 1)
      stddevs = k_singletagstddevs;
    else
      stddevs = k_ignorestddevs;

    // return the composed std devs
    return stddevs;
  }

  /** Returns the most recent pipeline result. */
  public Optional<PhotonPipelineResult> getResults() {
    return results;
  }

  /** Returns the current estimate with standard deviations. */
  public Pair<Optional<EstimatedRobotPose>, Matrix<N3, N1>> getEstimate() {
    return estimate;
  }

  /** Returns the camera sim instance to interface with. */
  public PhotonCameraSim getSimInstance() {
    return m_camerasim;
  }

  @Override
  public void periodic() {

    // clear results and update them
    for (var result : m_camera.getAllUnreadResults()) {
      results = Optional.of(result);
    }

    // check if results are present for each camera, if yes, perform heuristics and
    // compose estimate
    results.ifPresentOrElse(result -> {
      m_estimator.update(result).ifPresent(est -> {
        estimate = Pair.of(Optional.of(est), calculateStdDevs(est, result));
      });
    }, () -> estimate = Pair.of(Optional.empty(), k_ignorestddevs)); // empty estimate and set stddevs to
                                                                     // ignore
  }
}
