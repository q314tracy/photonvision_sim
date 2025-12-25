// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.Telemetry;

import static frc.robot.utils.Constants.OIConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

public class RobotContainer {

  // objects
  private final CommandXboxController m_joystick = new CommandXboxController(k_joystickport);
  private final Swerve m_swerve = new Swerve();
  private final Vision m_vision = new Vision();
  private final Telemetry m_telemetry = new Telemetry(m_vision, m_swerve);

  // autochooser
  private final SendableChooser<Command> m_autochooser = new SendableChooser<>();

  public RobotContainer() {

    // run autobuilder and configure OI bindings
    Pathfinding.setPathfinder(new LocalADStar());
    m_swerve.runAutoBuilder();
    configureBindings();

    // autos
    m_autochooser.setDefaultOption("no auto", print("WARNING: no auto scheduled"));
    m_autochooser.addOption("auto 1", AutoBuilder.buildAuto("Auto 1"));
    SmartDashboard.putData(m_autochooser);

    // default commands for subsystems
    m_swerve.setDefaultCommand(swerveDefaultCommand());
    m_vision.setDefaultCommand(visionDefaultCommand());
  }

  // triggers
  private void configureBindings() {
    m_joystick.a().onTrue(m_swerve.pathfindToPose(new Pose2d(3, 3, new Rotation2d())));
  }

  /** Composed default command for photon vision subsystem. */
  private ParallelCommandGroup visionDefaultCommand() {
    ParallelCommandGroup cmd = new ParallelCommandGroup();
    if (RobotBase.isSimulation()) cmd.addCommands(run(() -> m_vision.updatePose(m_swerve.getSimPose())));
    cmd.addCommands(run(() -> m_swerve.addVisionMeasurements(m_vision.getEstimates())));
    cmd.addRequirements(m_vision);
    return cmd;
  }
  
  /** Composed default command for swerve subsystem. */
  private ParallelCommandGroup swerveDefaultCommand() {
    ParallelCommandGroup cmd = new ParallelCommandGroup();
    cmd.addCommands(new RunCommand(
      () -> m_swerve.drive(new ChassisSpeeds(
        MathUtil.applyDeadband(m_joystick.getLeftX(), 0.2) * k_maxlinspeedteleop,
        -MathUtil.applyDeadband(m_joystick.getLeftY(), 0.2) * k_maxlinspeedteleop,
        -MathUtil.applyDeadband(m_joystick.getRightX(), 0.2) * k_maxrotspeedteleop
      )), m_swerve));
    return cmd;
  }

  public Command getAutonomousCommand() {
    return m_autochooser.getSelected();
  }
}