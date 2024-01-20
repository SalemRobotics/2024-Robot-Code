// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
  
  final XboxController mDriveController = new XboxController(ControllerConstants.kDriverPort); 

  final Drivetrain mDrivetrain = new Drivetrain();

  final SendableChooser<Command> mAutoChooser = AutoBuilder.buildAutoChooser(AutoConstants.kTestAuto2);
  
  public RobotContainer() {
    configureBindings();

    SmartDashboard.putData(mAutoChooser);

    // Set default Drivetrain command to a RunCommand containing Drivetrain::drive.
    mDrivetrain.setDefaultCommand(
      new RunCommand(
        // Controllers should have a deadband so that there is no drifting. 
        // Field relativity and rate limiting should always be true.
        () -> mDrivetrain.drive(
          -MathUtil.applyDeadband(mDriveController.getLeftY(), ControllerConstants.kDriveDeadband),
          -MathUtil.applyDeadband(mDriveController.getLeftX(), ControllerConstants.kDriveDeadband),
          -MathUtil.applyDeadband(mDriveController.getRightX(), ControllerConstants.kDriveDeadband),
          true, true), 
        mDrivetrain)
    );
  }

  private void configureBindings() {
    // Set wheels to X configuration when R trigger is being pressed.
    new JoystickButton(mDriveController, Button.kR1.value).whileTrue(
      new RunCommand(() -> mDrivetrain.setX(), mDrivetrain)
    );
  }

  public Command getAutonomousCommand() {
    return mAutoChooser.getSelected();
  }
}
