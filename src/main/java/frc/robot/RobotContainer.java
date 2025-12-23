// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

public class RobotContainer {

  // objects
  private final CommandXboxController m_joystick = new CommandXboxController(k_joystickport);
  private final Drive m_drive = new Drive();
  private final Photon m_photon = new Photon();
  private final Telemetry m_telemetry = new Telemetry(m_drive, m_photon);

  // autos
  private final PathPlannerAuto auto_1;

  // autochooser
  private final SendableChooser<Command> m_autochooser = new SendableChooser<>();

  public RobotContainer() {

    // autobuilder and autos
    m_drive.runAutoBuilder();
    auto_1 = new PathPlannerAuto("Auto 1");

    // configure OI bindings
    configureBindings();

    // autobuilder and autos
    Pathfinding.setPathfinder(new LocalADStar());
    m_autochooser.setDefaultOption("no auto", print("WARNING: no auto scheduled"));
    m_autochooser.addOption("Auto 1", pathfind(auto_1.getStartingPose()).andThen(auto_1));
    SmartDashboard.putData(m_autochooser);

    // void default method for drive subsystem
    m_drive.setDefaultCommand(new RunCommand(() -> {
      m_drive.driveArcade(
        MathUtil.applyDeadband(-m_joystick.getLeftY(), 0.2) * k_maxlinspeedteleop,
        MathUtil.applyDeadband(-m_joystick.getRightX(), 0.2) * k_maxrotspeedteleop
      );
    }, m_drive));

    // void to run continuously for photon, do not interrupt or i KEEL u
    m_photon.setDefaultCommand(new RunCommand(() -> {
      if (RobotBase.isSimulation()) m_photon.updatePose(m_drive.getSimPose());
      m_drive.addVisionMeasurement(m_photon.getEstimates());
    }, m_photon));
  }

  private void configureBindings() {
    m_joystick.a().onTrue(pathfind(new Pose2d(6, 1, new Rotation2d(Math.PI))));
  }

  public Command pathfind(Pose2d pose) {
    Command cmd = AutoBuilder.pathfindToPose(
      pose,
      new PathConstraints(
        4,
        4,
        2 * Math.PI,
        4 * Math.PI,
        12));
    cmd.addRequirements(m_drive);
    return cmd;
  }

  public Command getAutonomousCommand() {
    return m_autochooser.getSelected();
  }
}