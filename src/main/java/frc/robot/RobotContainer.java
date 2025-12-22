// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Photon;
import frc.robot.utils.Telemetry;
import static frc.robot.utils.Constants.OIConstants.*;

public class RobotContainer {

  // objects
  private final CommandXboxController m_joystick = new CommandXboxController(k_joystickport);
  private final Drive m_drive = new Drive();
  private final Photon m_photon = new Photon();
  private final Telemetry m_telemetry = new Telemetry(m_drive, m_photon);

  // autochooser
  private final SendableChooser<Command> m_autochooser = new SendableChooser<>();

  public RobotContainer() {

    // configure trigger bindings
    configureBindings();

    // void default method for drive subsystem
    m_drive.setDefaultCommand(new RunCommand(() -> {
      m_drive.driveArcade(
        MathUtil.applyDeadband(-m_joystick.getLeftY(), 0.2) * k_maxlinspeedteleop,
        MathUtil.applyDeadband(-m_joystick.getRightX(), 0.2) * k_maxrotspeedteleop
      );
    }, this.m_drive));

    //void to run continuously for photon, do not interrupt
    m_photon.setDefaultCommand(new RunCommand(() -> {
      m_drive.addVisionMeasurement(m_photon.getEstimatesWithStdDevs());
      m_photon.updatePose(m_drive.getOdometricPose());
    }, m_photon));

    // autochooser
    m_autochooser.setDefaultOption("no auto", print("WARNING: no auto scheduled"));
    SmartDashboard.putData(m_autochooser);
  }

  private void configureBindings() {

  }

  public Command getAutonomousCommand() {
    return m_autochooser.getSelected();
  }
}