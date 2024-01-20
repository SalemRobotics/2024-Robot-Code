// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
  
  final XboxController mDriveController = new XboxController(ControllerConstants.kDriverPort); 

  final Drivetrain mDrivetrain = new Drivetrain();

  final Shooter mShooter = new Shooter();
  
  public RobotContainer() {
    configureBindings();

    mDrivetrain.setDefaultCommand(
      new RunCommand(
        () -> mDrivetrain.drive(
          -MathUtil.applyDeadband(mDriveController.getLeftY(), ControllerConstants.kDriveDeadband),
          -MathUtil.applyDeadband(mDriveController.getLeftX(), ControllerConstants.kDriveDeadband),
          -MathUtil.applyDeadband(mDriveController.getRightX(), ControllerConstants.kDriveDeadband),
          true, true), 
        mDrivetrain)
    );
  }

  private void configureBindings() {
    new JoystickButton(mDriveController, Button.kR1.value).whileTrue(
      new RunCommand(() -> mDrivetrain.setX(), mDrivetrain)
    );
    
    new JoystickButton(mDriveController, Button.kCross.value).whileTrue(
      mShooter.shootRing()
    );
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
