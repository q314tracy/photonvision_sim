// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Photon;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.Telemetry;

import static frc.robot.utils.Constants.OIConstants.*;
import static frc.robot.utils.Constants.SwerveDriveConstants.k_maxlinspeed;
import static frc.robot.utils.Constants.SwerveDriveConstants.k_maxrotspeed;

public class RobotContainer {

  // objects
  private final CommandXboxController m_joystick = new CommandXboxController(k_joystickport);
  private final Swerve m_swerve = new Swerve();
  private final Photon m_photon = new Photon();
  private final Telemetry m_telemetry = new Telemetry(m_photon, m_swerve);

  // autochooser
  private final SendableChooser<Command> m_autochooser = new SendableChooser<>();

  public RobotContainer() {

    // configure OI bindings
    configureBindings();

    // autos
    m_autochooser.setDefaultOption("no auto", print("WARNING: no auto scheduled"));
    SmartDashboard.putData(m_autochooser);

    // drive command
    m_swerve.setDefaultCommand(new RunCommand(
      () -> m_swerve.drive(new ChassisSpeeds(
        -MathUtil.applyDeadband(m_joystick.getLeftY(), 0.2) * k_maxlinspeed,
        -MathUtil.applyDeadband(m_joystick.getLeftX(), 0.2) * k_maxlinspeed,
        -MathUtil.applyDeadband(m_joystick.getRightX(), 0.2) * k_maxrotspeed
      )), m_swerve));

    // run photon vision stuff
    m_photon.setDefaultCommand(new RunCommand(
      () -> m_photon.updatePose(m_swerve.getPose()), m_photon));
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return m_autochooser.getSelected();
  }
}