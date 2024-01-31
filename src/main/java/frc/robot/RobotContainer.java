// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.command.IntakeInAndIndex;
import frc.robot.command.IntakeOutAndIndex;
import frc.robot.command.SpinUpShooterAndIndex;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class RobotContainer {
  
  public static final Field2d m_field = new Field2d();

  final XboxController mDriveController = new XboxController(ControllerConstants.kDriverPort); 
  final XboxController mOperatorController = new XboxController(ControllerConstants.kOperatorPort);

  final Drivetrain mDrivetrain = new Drivetrain();

  final Shooter mShooter = new Shooter();
  
  final Indexer mIndexer = new Indexer();
  
  final Intake mIntake = new Intake();
  
  final SendableChooser<Command> mAutoChooser = AutoBuilder.buildAutoChooser(AutoConstants.kTestAuto2);

  public RobotContainer() {
    configureBindings();

    SmartDashboard.putData("Field", m_field);

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
    new JoystickButton(mDriveController, Button.kRightBumper.value).whileTrue(
      new RunCommand(() -> mDrivetrain.setX(), mDrivetrain)
    );
    
    new JoystickButton(mOperatorController, Button.kX.value).whileTrue(
      new SpinUpShooterAndIndex(mIndexer, mShooter)
    );
    
    new JoystickButton(mOperatorController, Button.kRightBumper.value).whileTrue(
      new IntakeInAndIndex(mIntake, mIndexer)
    );
    
    new JoystickButton(mOperatorController, Button.kLeftBumper.value).whileTrue(
      new IntakeOutAndIndex(mIntake, mIndexer)
    );
  }

  public Command getAutonomousCommand() {
    return mAutoChooser.getSelected();
  }

}
