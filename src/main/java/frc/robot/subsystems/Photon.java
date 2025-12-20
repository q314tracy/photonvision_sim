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
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
  private Optional<PhotonPipelineResult> results = Optional.empty();
  private Optional<EstimatedRobotPose> estimate = Optional.empty();
  private Optional<Pair<EstimatedRobotPose,Matrix<N3,N1>>> estimate_stddevs = Optional.empty();
  private Matrix<N3,N1> cur_stddevs;


  Matrix<N3,N1> std_devs;


  // target data
  private List<Double> targetdata_distance = new ArrayList<>();
  private List<Double> targetdata_yaw = new ArrayList<>();

  // persistent list of visible active fiducials
  private List<Integer> visible_fiducials = new ArrayList<>();

  public Photon() {

    // camera and pose estimator
    m_camera = new PhotonCamera("camera");
    m_cameraestimator = new PhotonPoseEstimator(
        k_fieldlayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        k_camera1_intrinsics);
    m_cameraestimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

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
      m_camerasim.enableRawStream(true);
      m_camerasim.enableDrawWireframe(true);
    }
  }











  public Optional<Pair<EstimatedRobotPose,Matrix<N3,N1>>> getEstimate() {
    return estimate_stddevs;
  }







  /** Used to update visible fiducials in Telemetry. */
  public List<Integer> getFiducials() {
    results.ifPresent(result -> {
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

  public List<Double> getTargetDistances() {
    return targetdata_distance;
  }

  public List<Double> getTargetYaws() {
    return targetdata_yaw;
  }

  private double getDistanceToTarget2D(PhotonTrackedTarget target) {
    return Math.hypot(target.getBestCameraToTarget().getX(), target.getBestCameraToTarget().getY());
  }

  private double getAngleOffsetZ(PhotonTrackedTarget target) {
    return Math.abs(MathUtil.angleModulus(target.bestCameraToTarget.getRotation().getZ() - Math.PI));










  }

  @Override
  public void periodic() {

    // empty everyone
    results = Optional.empty();
    estimate = Optional.empty();
    estimate_stddevs = Optional.empty();

    // update results
    for (var result : m_camera.getAllUnreadResults()) {
      results = Optional.of(result);
    }

    // check if results are present and update target data, also remove or change result data here
    results.ifPresent(res -> {
      // mutable instance of results
      PhotonPipelineResult results_mutable = new PhotonPipelineResult(
          res.metadata,
          res.targets,
          res.multitagResult);
      // post the results after mutation
      for (var target : res.targets) {
        if (k_tagblacklist.contains(target.fiducialId)) {
          results_mutable.targets.remove(results_mutable.targets.indexOf(target));
        }
      }
      results = Optional.of(results_mutable);
      // clear lists and add data to lists to be posted to dashboard
      targetdata_distance.clear();
      targetdata_yaw.clear();
      for (var target : results_mutable.targets) {
        targetdata_distance.add(getDistanceToTarget2D(target));
        targetdata_yaw.add(Units.radiansToDegrees(getAngleOffsetZ(target)));
      }
    });

    // check and update estimate, as well as calculate std devs
    results.ifPresent(res -> {
      // update raw estimate and declare data for std devs calcluations
      estimate = m_cameraestimator.update(res);
      var stddevs = k_singletagstddevs;
      int numtags = 0;
      double avgdist = 0;
      // run foreach to calculate avg distances of visible fiducials and find quantity of tags
      for (var target : res.targets) {
        var tagpose = m_cameraestimator.getFieldTags().getTagPose(target.fiducialId);
        if (tagpose.isEmpty()) continue; //skip if the tag pose is empty to prevent NPE
        numtags++; // add 1 to count how many tags
        avgdist += tagpose.get().toPose2d().getTranslation().getDistance( // add distances up
        estimate.get().estimatedPose.toPose2d().getTranslation());
      }
      avgdist /= numtags; // divide by number of tags to get average
      // if single tag, high ambiguity
      // if multitag, low ambiguity
      if (numtags == 0) stddevs = k_singletagstddevs;
      if (numtags > 1) stddevs = k_multitagstddevs;
      // if 1 tag and distance is greater than 4 meters, ignore tag
      // if 1 tag and less than 4 meters, scale by avg distance
      if (numtags == 1 && avgdist > 4) stddevs = k_ignorestddevs;
      else stddevs = stddevs.times(1 + (avgdist * avgdist / 30));
      // pass the standard devs out of block
      cur_stddevs = stddevs;
    });

    // check and update composed estimate with std devs
    estimate.ifPresent(est -> {
      estimate_stddevs = Optional.of(Pair.of(est, cur_stddevs));
    });
  }
}
