// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.IntakeInAndIndex;
import frc.robot.commands.IntakeOutAndIndex;
import frc.robot.commands.ScoreAmpFromIndexer;
import frc.robot.commands.SourceIntakeAndIndex;
import frc.robot.commands.SpinUpShooterAndIndex;
import frc.robot.commands.TrackTargetAndShoot;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.StrongArmMachine;
import frc.util.SwerveUtils;

public class RobotContainer {
  
  public static final Field2d m_field = new Field2d();

  final XboxController mDriveController = new XboxController(ControllerConstants.kDriverPort); 
  final XboxController mOperatorController = new XboxController(ControllerConstants.kOperatorPort);

  final Drivetrain mDrivetrain = new Drivetrain(true, true);
  final StrongArmMachine mStrongArmMachine = new StrongArmMachine();
  final Shooter mShooter = new Shooter();
  final Indexer mIndexer = new Indexer();
  final Intake mIntake = new Intake();
  final Vision mVision = new Vision();
  final AutoPicker mAutoPicker = new AutoPicker();
  
  public RobotContainer() {
    configureBindings();
    configureNamedCommands();

    SmartDashboard.putData("Field", m_field);

    // Set default Drivetrain command to a RunCommand containing Drivetrain::drive.
    mDrivetrain.setDefaultCommand(
      new RunCommand(
        () -> mDrivetrain.drive(
          -SwerveUtils.squareInputs(mDriveController.getLeftY(), ControllerConstants.kDriveDeadband),
          -SwerveUtils.squareInputs(mDriveController.getLeftX(), ControllerConstants.kDriveDeadband),
          -MathUtil.applyDeadband(mDriveController.getRightX(), ControllerConstants.kDriveDeadband)), 
        mDrivetrain)
    );
  }

  private void configureBindings() {
    new JoystickButton(mDriveController, Button.kLeftBumper.value).whileTrue(
      mDrivetrain.setX()
    );

    new JoystickButton(mDriveController, Button.kStart.value).onTrue(
      mDrivetrain.resetHeading()
    );

    // #region Cardinal Direction Commands
    new JoystickButton(mDriveController, Button.kY.value).whileTrue(
      mDrivetrain.trackCardinal(Direction.North, 
        () -> -SwerveUtils.squareInputs(mDriveController.getLeftY(), ControllerConstants.kDriveDeadband),
        () -> -SwerveUtils.squareInputs(mDriveController.getLeftX(), ControllerConstants.kDriveDeadband))
    );

    new JoystickButton(mDriveController, Button.kB.value).whileTrue(
      mDrivetrain.trackCardinal(Direction.East,
        () -> -SwerveUtils.squareInputs(mDriveController.getLeftY(), ControllerConstants.kDriveDeadband),
        () -> -SwerveUtils.squareInputs(mDriveController.getLeftX(), ControllerConstants.kDriveDeadband))
    );

    new JoystickButton(mDriveController, Button.kA.value).whileTrue(
      mDrivetrain.trackCardinal(Direction.South,
        () -> -SwerveUtils.squareInputs(mDriveController.getLeftY(), ControllerConstants.kDriveDeadband),
        () -> -SwerveUtils.squareInputs(mDriveController.getLeftX(), ControllerConstants.kDriveDeadband))
    );

    new JoystickButton(mDriveController, Button.kX.value).whileTrue(
      mDrivetrain.trackCardinal(Direction.West,
        () -> -SwerveUtils.squareInputs(mDriveController.getLeftY(), ControllerConstants.kDriveDeadband),
        () -> -SwerveUtils.squareInputs(mDriveController.getLeftX(), ControllerConstants.kDriveDeadband))
    );
    // #endregion
    
    // #region Operator Controls
    new JoystickButton(mDriveController, Button.kRightBumper.value).whileTrue(
      new TrackTargetAndShoot(
        mDrivetrain, 
        mVision, 
        mIndexer, 
        mShooter,
        () -> -SwerveUtils.squareInputs(mDriveController.getLeftY(), ControllerConstants.kDriveDeadband),
        () -> -SwerveUtils.squareInputs(mDriveController.getLeftX(), ControllerConstants.kDriveDeadband)
      )
    );
    
    new JoystickButton(mOperatorController, Button.kRightBumper.value).whileTrue(
      new IntakeInAndIndex(mIntake, mIndexer)
    );
    
    new JoystickButton(mOperatorController, Button.kLeftBumper.value).whileTrue(
      new IntakeOutAndIndex(mIntake, mIndexer)
    );

    new JoystickButton(mOperatorController, Button.kX.value).whileTrue(
      new SourceIntakeAndIndex(mIntake, mIndexer, mStrongArmMachine)
    );

    new JoystickButton(mOperatorController, Button.kY.value).whileTrue(
      new ScoreAmpFromIndexer(mIntake, mIndexer, mStrongArmMachine)
    );
    // #endregion
  }

  public void configureNamedCommands(){
    NamedCommands.registerCommand("far intake", new IntakeInAndIndex(mIntake, mIndexer));
    NamedCommands.registerCommand("close intake", new IntakeInAndIndex(mIntake, mIndexer, IndexerConstants.kIndexerSpeedIn, IndexerConstants.kIndexerSpeedIn ));
    NamedCommands.registerCommand("shoot", new SpinUpShooterAndIndex(mIndexer, mShooter, mVision));
    NamedCommands.registerCommand("index to shoot", mIndexer.runUpperIndexer(IndexerConstants.kIndexerSpeedIn));
    NamedCommands.registerCommand("run shooter", mShooter.shootRing(mVision::getDistance));
    NamedCommands.registerCommand("tune shooter", mShooter.shootRing(mVision::getDistance));
    NamedCommands.registerCommand("angle45", mShooter.setShooterAngle(45.0));
  }

  public Command getAutonomousCommand() {
    return mAutoPicker.getSelected();
  }

}
