// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.DriveConstants;

import static frc.robot.utils.Constants.DriveConstants.*;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;

public class Drive extends SubsystemBase {

  // main objects for drivetrain
  private final PWMSparkMax m_leftmotor;
  private final PWMSparkMax m_rightmotor;
  private final Encoder m_leftenc;
  private final Encoder m_rightenc;
  private final ADXRS450_Gyro m_gyro;
  private final DifferentialDriveKinematics m_kinematics;
  private final DifferentialDrivePoseEstimator m_poseestimator;

  // classes used to sim the hardware
  private final EncoderSim m_leftencsim;
  private final EncoderSim m_rightencsim;
  private final ADXRS450_GyroSim m_gyrosim;

  // robot configuration
  private RobotConfig robotConfig;

  // differential drive simulation object
  private DifferentialDrivetrainSim m_drivetrainsim;
  private DifferentialDriveOdometry m_odometrysim;

  public Drive() {

    // spark maxes
    m_leftmotor = new PWMSparkMax(k_leftPWMchannel);
    m_rightmotor = new PWMSparkMax(k_rightPWMchannel);

    // inversions for motors
    m_leftmotor.setInverted(false);
    m_rightmotor.setInverted(true);

    // instantiate encoders
    m_leftenc = new Encoder(k_leftENCchannels[0], k_leftENCchannels[1]);
    m_rightenc = new Encoder(k_rightENCchannels[0], k_rightENCchannels[1]);

    // set dpp
    m_leftenc.setDistancePerPulse((2 * Math.PI * k_wheelradius) / k_ENCresolution);
    m_rightenc.setDistancePerPulse((2 * Math.PI * k_wheelradius) / k_ENCresolution);

    // reset at init
    m_leftenc.reset();
    m_rightenc.reset();

    // init and reset gyro
    m_gyro = new ADXRS450_Gyro();
    m_gyrosim = new ADXRS450_GyroSim(m_gyro);
    m_gyro.calibrate();

    // kinematics and PE
    m_kinematics = new DifferentialDriveKinematics(k_trackwidth);
    m_poseestimator = new DifferentialDrivePoseEstimator(
        m_kinematics,
        m_gyro.getRotation2d(),
        m_leftenc.getDistance(),
        m_rightenc.getDistance(),
        k_initpose);

    // robot config
    try {
      robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }

    // simulation stuff
    if (RobotBase.isSimulation()) {

      // drivetrain sim object
      m_drivetrainsim = new DifferentialDrivetrainSim(
          DCMotor.getNEO(2),
          DriveConstants.k_gearratio,
          DriveConstants.k_rotateMOI,
          DriveConstants.k_massKG,
          DriveConstants.k_wheelradius,
          DriveConstants.k_trackwidth,
          null);

      // drivetrain odometry explicitly for simulation
      m_odometrysim = new DifferentialDriveOdometry(
          m_gyro.getRotation2d(),
          m_leftenc.getDistance(),
          m_rightenc.getDistance(),
          k_initpose);
    }

    // init sim encoders
    m_leftencsim = new EncoderSim(m_leftenc);
    m_rightencsim = new EncoderSim(m_rightenc);
  }

  /**
   * Drive in arcade mode.
   * 
   * @param xspeed Linear speed in meters/sec.
   * @param Zspeed Rotational speed in rads/sec.
   */
  public void driveArcade(double Xspeed, double Zspeed) {
    DifferentialDriveWheelSpeeds speeds = m_kinematics.toWheelSpeeds(
        new ChassisSpeeds(
            Xspeed,
            0,
            Zspeed));
    m_leftmotor.setVoltage((speeds.leftMetersPerSecond / k_maxlinspeed) * 12);
    m_rightmotor.setVoltage((speeds.rightMetersPerSecond / k_maxlinspeed) * 12);
  }

  /**
   * Drive in auto mode with ChassisSpeeds.
   * 
   * @param speeds The robot-relative chassis speeds.
   */
  public void autoDrive(ChassisSpeeds speeds) {
    DifferentialDriveWheelSpeeds wheelspeeds = m_kinematics.toWheelSpeeds(speeds);
    m_leftmotor.setVoltage((wheelspeeds.leftMetersPerSecond / k_maxlinspeed) * 12);
    m_rightmotor.setVoltage((wheelspeeds.rightMetersPerSecond / k_maxlinspeed) * 12);
  }

  /**
   * Returns the current robot-relative chassis speeds.
   * 
   * @return
   */
  public ChassisSpeeds getChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(
        new DifferentialDriveWheelSpeeds(
            m_leftenc.getRate(),
            m_rightenc.getRate()));
  }

  /**
   * Resets the current pose of the pose estimator.
   * 
   * @param pose The pose.
   */
  public void resetPose(Pose2d pose) {
    if (RobotBase.isSimulation()) m_odometrysim.resetPose(pose);
    m_poseestimator.resetPose(pose);
  }

  /**
   * Adds all of the vision estimates in a list to the pose estimator.
   * 
   * @param estimates The list of estimates.
   */
  public void addVisionMeasurement(List<Pair<Optional<EstimatedRobotPose>, Matrix<N3, N1>>> estimates) {
    // check if estimates are present, iterate on list and add measurements
    // including std devs
    if (!estimates.isEmpty()) {
      for (var estimate : estimates) {
        estimate.getFirst().ifPresent(est -> {
          m_poseestimator.addVisionMeasurement(
              est.estimatedPose.toPose2d(), // pose
              est.timestampSeconds, // timestamp
              estimate.getSecond() // std devs
          );
        });
      }
    }
  }

  /** Runs the autobuilder at the start of the code. */
  public void runAutoBuilder() {
    AutoBuilder.configure(
        this::getEstimatedPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> autoDrive(speeds), // Method that will drive the robot given ROBOT RELATIVE
                                                     // ChassisSpeeds. Also optionally outputs individual module
                                                     // feedforwards
        new PPLTVController(
            0.020,
            DriveConstants.k_maxlinspeed),
        robotConfig, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );
  }

  /** Returns the current estimated pose. */
  public Pose2d getEstimatedPose() {
    return m_poseestimator.getEstimatedPosition();
  }

  /** Returns the current simulated odometric pose. */
  public Pose2d getSimPose() {
    if (RobotBase.isSimulation()) {
      return m_odometrysim.getPoseMeters();
    } else {
      return new Pose2d();
    }
  }

  @Override
  public void periodic() {

    // check if simulation, if yes then run block in conditional
    if (RobotBase.isSimulation()) {

      // set inputs for simulation
      m_drivetrainsim.setInputs(
          m_leftmotor.get() * 12,
          m_rightmotor.get() * 12);

      // update simulation with new inputs
      m_drivetrainsim.update(0.020);

      // update hardware simulation interfaces
      m_leftencsim.setDistance(m_drivetrainsim.getLeftPositionMeters());
      m_leftencsim.setRate(m_drivetrainsim.getLeftVelocityMetersPerSecond());
      m_rightencsim.setDistance(m_drivetrainsim.getRightPositionMeters());
      m_rightencsim.setRate(m_drivetrainsim.getRightVelocityMetersPerSecond());
      m_gyrosim.setAngle(-m_drivetrainsim.getHeading().getDegrees());

      // odometry for simulation
      m_odometrysim.update(
          m_gyro.getRotation2d(),
          m_leftenc.getDistance(),
          m_rightenc.getDistance());
    }

    // update the PE
    m_poseestimator.update(
        m_gyro.getRotation2d(),
        m_leftenc.getDistance(),
        m_rightenc.getDistance());
  }
}
