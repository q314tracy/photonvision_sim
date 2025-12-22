// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.utils.Constants.VisionConstants.k_cameraleft_intrinsics;
import static frc.robot.utils.Constants.VisionConstants.k_fieldlayout;
import static frc.robot.utils.Constants.VisionConstants.k_ignorestddevs;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionCamera extends SubsystemBase {

  private final PhotonCamera m_camera;
  private final PhotonPoseEstimator m_estimator;
  private PhotonCameraSim m_camerasim;

  private Optional<PhotonPipelineResult> results = Optional.empty();
  private Pair<Optional<EstimatedRobotPose>,Matrix<N3,N1>> estimate = Pair.of(Optional.empty(), k_ignorestddevs);

  public VisionCamera(String name) {
    
    m_camera = new PhotonCamera(name);
    m_estimator = new PhotonPoseEstimator(
      k_fieldlayout,
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      k_cameraleft_intrinsics
    );
    m_estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    if (RobotBase.isSimulation()) {
      SimCameraProperties camerasim_properties = new SimCameraProperties();
      camerasim_properties.setFPS(30);
      m_camerasim = new PhotonCameraSim(m_camera, camerasim_properties);
      m_camerasim.enableRawStream(false);
      m_camerasim.enableDrawWireframe(false);
    }
  }

  @Override
  public void periodic() {

  }
}
