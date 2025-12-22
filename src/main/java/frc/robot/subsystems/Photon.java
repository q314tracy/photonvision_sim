// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.utils.Constants.VisionConstants.*;

import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;

public class Photon extends SubsystemBase {

  // camera abstraction objects
  private final VisionCamera m_leftcamera;
  private final VisionCamera m_centercamera;
  private final VisionCamera m_rightcamera;
  private final VisionCamera m_rearcamera;

  // simulation stuff
  private VisionSystemSim m_visionsim;

  public Photon() {

    // cameras
    m_leftcamera = new VisionCamera("left_camera", k_cameraleft_intrinsics);
    m_centercamera = new VisionCamera("center_camera", k_cameracenter_intrinsics);
    m_rightcamera = new VisionCamera("right_camera", k_cameraright_intrinsics);
    m_rearcamera = new VisionCamera("rear_camera", k_camerarear_intrinsics);

    // IT'S SIMULATIN TIME
    if (RobotBase.isSimulation()) {

      // declare vision sim and add cameras
      m_visionsim = new VisionSystemSim("main");
      m_visionsim.addCamera(m_leftcamera.getSimInstance(), k_cameraleft_intrinsics);
      m_visionsim.addCamera(m_centercamera.getSimInstance(), k_cameracenter_intrinsics);
      m_visionsim.addCamera(m_rightcamera.getSimInstance(), k_cameraright_intrinsics);
      m_visionsim.addCamera(m_rearcamera.getSimInstance(), k_camerarear_intrinsics);
      m_visionsim.addAprilTags(k_fieldlayout);
    }
  }

  /** Use to return a HashSet containing all  */
  public HashSet<Integer> getAllFiducials() {

    // compose the set of fiducials
    // using a hashset automatically removes duplicate entries
    HashSet<Integer> set = new HashSet<>();
    set.addAll(m_leftcamera.getFiducials());
    set.addAll(m_centercamera.getFiducials());
    set.addAll(m_rightcamera.getFiducials());
    set.addAll(m_rearcamera.getFiducials());

    // return the composed set converted back to a list
    return set;
  }

  /** Returns the list of estimates from the camera pipelines. */
  public List<Pair<Optional<EstimatedRobotPose>,Matrix<N3,N1>>> getEstimates() {
    return Arrays.asList(
      m_leftcamera.getEstimate(),
      m_centercamera.getEstimate(),
      m_rightcamera.getEstimate(),
      m_rearcamera.getEstimate()
    );
  }

  /**
   * Used to update the pose of the vision sim periodically.
   * Possibly not necessary when real. Testing required.
   * 
   * @param pose The odometric pose of the robot.
   */
  public void updatePose(Pose2d pose) {
    m_visionsim.update(pose);
  }

  @Override
  public void periodic() {

  }
}
