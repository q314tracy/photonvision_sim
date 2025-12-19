// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.utils.Constants.DriveConstants.*;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

public class Drive extends SubsystemBase {

  //main objects for drivetrain
  private final PWMSparkMax m_leftmotor;
  private final PWMSparkMax m_rightmotor;
  private final Encoder m_leftenc;
  private final Encoder m_rightenc;
  private final ADXRS450_Gyro m_gyro;
  private final DifferentialDriveKinematics m_kinematics;
  private final DifferentialDriveOdometry m_odometry;
  private final DifferentialDrivePoseEstimator m_poseestimator;

  //classes used to sim the hardware
  private final EncoderSim m_leftencsim;
  private final EncoderSim m_rightencsim;
  private final ADXRS450_GyroSim m_gyrosim;

  //differential drive simulation object
  private final DifferentialDrivetrainSim m_drivetrainsim;







  public Drive() {

    //spark maxes
    m_leftmotor = new PWMSparkMax(k_leftPWMchannel);
    m_rightmotor = new PWMSparkMax(k_rightPWMchannel);

    //inversions for motors
    m_leftmotor.setInverted(false);
    m_rightmotor.setInverted(true);

    //instantiate encoders
    m_leftenc = new Encoder(k_leftENCchannels[0], k_leftENCchannels[1]);
    m_rightenc = new Encoder(k_rightENCchannels[0], k_rightENCchannels[1]);

    //set dpp
    m_leftenc.setDistancePerPulse((2 * Math.PI * k_wheelradius) / k_ENCresolution);
    m_rightenc.setDistancePerPulse((2 * Math.PI * k_wheelradius) / k_ENCresolution);

    //init sim encoders
    m_leftencsim = new EncoderSim(m_leftenc);
    m_rightencsim = new EncoderSim(m_rightenc);

    //reset at init
    m_leftenc.reset();
    m_rightenc.reset();

    //init and reset gyro
    m_gyro = new ADXRS450_Gyro();
    m_gyrosim = new ADXRS450_GyroSim(m_gyro);
    m_gyro.reset();

    //kinematics , odometry, and PE
    m_kinematics = new DifferentialDriveKinematics(k_trackwidth);
    m_odometry = new DifferentialDriveOdometry(
      m_gyro.getRotation2d(),
      m_leftenc.getDistance(),
      m_rightenc.getDistance(),
      k_initpose
    );
    m_poseestimator = new DifferentialDrivePoseEstimator(
      m_kinematics,
      m_gyro.getRotation2d(),
      m_leftenc.getDistance(),
      m_rightenc.getDistance(),
      k_initpose
    );


    //reset odometry to starting pose on field
    // m_odometry.resetPose(k_initpose);

    //drivetrain sim object
    m_drivetrainsim = new DifferentialDrivetrainSim(
      LinearSystemId.identifyDrivetrainSystem(
        k_kVlinsim,
        k_kAlinsim,
        k_kVrotsim,
        k_kArotsim),
      DCMotor.getNEO(1),
      k_gearratio,
      k_trackwidth,
      k_wheelradius,
      null);
      // VecBuilder.fill(
      //   0.05,
      //   0.05,
      //   0.001,
      //   0.5,
      //   0.5,
      //   0.01,
      //   0.01
      // ));
  }








  /** Drive in arcade mode.
   * @param xspeed Linear speed in meters/sec.
   * @param Zspeed Rotational speed in rads/sec.
   */
  public void driveArcade(double Xspeed, double Zspeed) {

    //compose wheel speeds
    DifferentialDriveWheelSpeeds speeds = m_kinematics.toWheelSpeeds(
      new ChassisSpeeds(
        Xspeed,
        0,
        Zspeed
    ));

    //apply wheel speeds
    m_leftmotor.setVoltage((speeds.leftMetersPerSecond / k_maxlinspeed) * 12);
    m_rightmotor.setVoltage((speeds.rightMetersPerSecond / k_maxlinspeed) * 12);
  }








  public void addVisionMeasurement(Optional<EstimatedRobotPose> estimate) {

    //check if estimate is present
    estimate.ifPresent(est -> {
      m_poseestimator.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds);
      }
    );
  }





  public Pose2d getEstimatedPose() {
    return m_poseestimator.getEstimatedPosition();
  }

  public Pose2d getOdometricPose() {
    return m_odometry.getPoseMeters();
  }






  @Override
  public void periodic() {

    //update the PE
    m_poseestimator.update(
      m_gyro.getRotation2d(),
      m_leftenc.getDistance(),
      m_rightenc.getDistance()
    );

    //update the odometry
    m_odometry.update(
      m_gyro.getRotation2d(),
      m_leftenc.getDistance(),
      m_rightenc.getDistance()
    );

    //check if simulation, if yes then run block in conditional
    if (RobotBase.isSimulation()) {

      //set inputs for simulation
      m_drivetrainsim.setInputs(
      m_leftmotor.get() * 12,
      m_rightmotor.get() * 12
      );

      //update simulation with new inputs
      m_drivetrainsim.update(0.020);
    
      //update hardware simulation interfaces
      m_leftencsim.setDistance(m_drivetrainsim.getLeftPositionMeters());
      m_leftencsim.setRate(m_drivetrainsim.getLeftVelocityMetersPerSecond());
      m_rightencsim.setDistance(m_drivetrainsim.getRightPositionMeters());
      m_leftencsim.setRate(m_drivetrainsim.getRightVelocityMetersPerSecond());
      m_gyrosim.setAngle(-m_drivetrainsim.getHeading().getDegrees());
    }
  }
}
