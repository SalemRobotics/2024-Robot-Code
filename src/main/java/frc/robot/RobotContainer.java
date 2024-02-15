// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.util.SwerveUtils;

public class RobotContainer {
  
  final XboxController mDriveController = new XboxController(ControllerConstants.kDriverPort); 

  final Drivetrain mDrivetrain = new Drivetrain();
  final Indexer mIndexer = new Indexer();

  public RobotContainer() {
    configureBindings();

    mDrivetrain.setDefaultCommand(
      new RunCommand(
        () -> mDrivetrain.drive(
          -SwerveUtils.SquareInputs(mDriveController.getLeftY(), ControllerConstants.kDriveDeadband),
          -SwerveUtils.SquareInputs(mDriveController.getLeftX(), ControllerConstants.kDriveDeadband),
          -MathUtil.applyDeadband(mDriveController.getRightX(), ControllerConstants.kDriveDeadband),
          true, true), 
        mDrivetrain)
    );
  }

  private void configureBindings() {
    new JoystickButton(mDriveController, Button.kRightBumper.value).whileTrue(
      mDrivetrain.setX()
    );

    new JoystickButton(mDriveController, Button.kB.value).whileTrue(
       mIndexer.runIndexer()
    );
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

}
