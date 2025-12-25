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

import java.util.HashSet;
import java.util.List;
import java.util.Optional;

public class Vision extends SubsystemBase {

  // camera abstraction objects
  private final VisionCamera m_frontleft_camera;
  private final VisionCamera m_frontcenter_camera;
  private final VisionCamera m_frontright_camera;
  private final VisionCamera m_rearleft_camera;
  private final VisionCamera m_rearright_camera;

  // simulation stuff
  private VisionSystemSim m_visionsim;

  public Vision() {

    // cameras
    m_frontleft_camera = new VisionCamera("frontleft_camera", k_frontleftcameraintrinsics);
    m_frontcenter_camera = new VisionCamera("frontcenter_camera", k_frontcentercameraintrinsics);
    m_frontright_camera = new VisionCamera("frontright_camera", k_frontrightcameraintrinsics);
    m_rearleft_camera = new VisionCamera("rearleft_camera", k_rearleftcameraintrinsics);
    m_rearright_camera = new VisionCamera("rearright_camera", k_rearrightcameraintrinsics);

    // IT'S SIMULATIN TIME
    if (RobotBase.isSimulation()) {

      // declare vision sim, add cameras, add fiducials
      m_visionsim = new VisionSystemSim("main");
      m_visionsim.addCamera(m_frontleft_camera.getSimInstance(), k_frontleftcameraintrinsics);
      m_visionsim.addCamera(m_frontcenter_camera.getSimInstance(), k_frontcentercameraintrinsics);
      m_visionsim.addCamera(m_frontright_camera.getSimInstance(), k_frontrightcameraintrinsics);
      m_visionsim.addCamera(m_rearleft_camera.getSimInstance(), k_rearleftcameraintrinsics);
      m_visionsim.addCamera(m_rearright_camera.getSimInstance(), k_rearrightcameraintrinsics);
      m_visionsim.addAprilTags(k_fieldlayout);
    }
  }

  /** Get a list of all of the estimates from the camera pipelines. */
  public List<Pair<Optional<EstimatedRobotPose>,Matrix<N3,N1>>> getEstimates() {
    return List.of(
      m_frontleft_camera.getEstimate(),
      m_frontcenter_camera.getEstimate(),
      m_frontright_camera.getEstimate(),
      m_rearleft_camera.getEstimate(),
      m_rearright_camera.getEstimate()
    );
  }

  

  /** Returns a list containing all the visible fiducial IDs. */
  public List<Integer> getAllFiducials() {

    // compose the set of fiducials
    // using a hashset automatically removes duplicate entries
    HashSet<Integer> set = new HashSet<>();
    set.addAll(m_frontleft_camera.getFiducials());
    set.addAll(m_frontcenter_camera.getFiducials());
    set.addAll(m_frontright_camera.getFiducials());
    set.addAll(m_rearleft_camera.getFiducials());
    set.addAll(m_rearright_camera.getFiducials());

    // return the composed set converted back to a list
    return set.stream().toList();
  }

  /**
   * Used to update the pose of the vision sim periodically.
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
