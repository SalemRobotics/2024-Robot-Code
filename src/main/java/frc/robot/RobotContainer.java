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
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.util.SwerveUtils;

public class RobotContainer {
  
  final XboxController mDriveController = new XboxController(ControllerConstants.kDriverPort); 
  final XboxController mOperatorController = new XboxController(ControllerConstants.kOperatorPort);

  final Drivetrain mDrivetrain = new Drivetrain();
  final Intake mIntake = new Intake();

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
    
    new JoystickButton(mOperatorController, Button.kRightBumper.value).whileTrue(
      mIntake.intakeRing(IntakeConstants.kIntakeSpeedIn)
    );
    
    new JoystickButton(mOperatorController, Button.kLeftBumper.value).whileTrue(
      mIntake.intakeRing(IntakeConstants.kIntakeSpeedOut)
    );
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
