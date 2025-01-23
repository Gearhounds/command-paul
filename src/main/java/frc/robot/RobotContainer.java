// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private final XboxController driverController = new XboxController(0);
  public RobotContainer() {
    driverController.getLeftY();
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem,
      () -> -driverController.getLeftX(), 
      () -> driverController.getLeftY(), 
      () -> driverController.getRightX(), 
      () -> false));
      SmartDashboard.putBoolean("Running Robot Container", true);
    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(driverController, 1).onTrue(Commands.run(() -> swerveSubsystem.zeroHeading()));
    
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
