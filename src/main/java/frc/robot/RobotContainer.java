// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.HandoffFromIndexer;
import frc.robot.commands.HandoffToIndexer;
import frc.robot.commands.IntakeInAndIndex;
import frc.robot.commands.IntakeOutAndIndex;
import frc.robot.commands.LobNote;
import frc.robot.commands.SpinUpShooterAndIndex;
import frc.robot.commands.TrackTargetAndShoot;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SAMConstants;
import frc.robot.auto.AutoPicker;
import frc.robot.auto.NonAmpRaceAuto;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.SourceAmpMech.SAMPositions;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SAMRoller;
import frc.robot.subsystems.SourceAmpMech;
import frc.robot.subsystems.StatusLED;
import frc.util.SwerveUtils;

public class RobotContainer {
  
  public static final Field2d mField = new Field2d();

  final CommandXboxController mDriveController = new CommandXboxController(ControllerConstants.kDriverPort); 
  final CommandXboxController mOperatorController = new CommandXboxController(ControllerConstants.kOperatorPort);

  final Drivetrain mDrivetrain = new Drivetrain(true, true);
  final Intake mIntake = new Intake();
  final Indexer mIndexer = new Indexer();
  final Shooter mShooter = new Shooter();
  final Vision mVision = new Vision();
  final SourceAmpMech mSourceAmpMech = new SourceAmpMech();
  final SAMRoller mSamRoller = new SAMRoller();

  final AutoPicker mAutoPicker;
  final StatusLED mLED = new StatusLED();
  
  public RobotContainer() {
    configureBindings();
    configureNamedCommands();

    mAutoPicker = new AutoPicker();
    mAutoPicker.initializeCommands("Race Autos", 
      new NonAmpRaceAuto(mDrivetrain, mIntake, mIndexer, mShooter, mVision)
		);

    SmartDashboard.putData("Field", mField);

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
    // #region Cardinal Direction Commands
    mDriveController.y().whileTrue(
      mDrivetrain.trackCardinal(Direction.North, 
        () -> -SwerveUtils.squareInputs(mDriveController.getLeftY(), ControllerConstants.kDriveDeadband),
        () -> -SwerveUtils.squareInputs(mDriveController.getLeftX(), ControllerConstants.kDriveDeadband))
    );

    mDriveController.b().whileTrue(
      mDrivetrain.trackCardinal(Direction.East,
        () -> -SwerveUtils.squareInputs(mDriveController.getLeftY(), ControllerConstants.kDriveDeadband),
        () -> -SwerveUtils.squareInputs(mDriveController.getLeftX(), ControllerConstants.kDriveDeadband))
    );

    mDriveController.a().whileTrue(
      mDrivetrain.trackCardinal(Direction.South,
        () -> -SwerveUtils.squareInputs(mDriveController.getLeftY(), ControllerConstants.kDriveDeadband),
        () -> -SwerveUtils.squareInputs(mDriveController.getLeftX(), ControllerConstants.kDriveDeadband))
    );

    mDriveController.x().whileTrue(
      mDrivetrain.trackCardinal(Direction.West,
        () -> -SwerveUtils.squareInputs(mDriveController.getLeftY(), ControllerConstants.kDriveDeadband),
        () -> -SwerveUtils.squareInputs(mDriveController.getLeftX(), ControllerConstants.kDriveDeadband))
    );
    // #endregion

    // #region Driver controls
    mDriveController.leftBumper().whileTrue(
      mDrivetrain.setX()
    );

    // resets heading
    mDriveController.start().whileTrue(
      mDrivetrain.resetHeading()
    );

    // standard control for tracking and targeting with an apriltag 
    mDriveController.rightBumper().whileTrue(
      new TrackTargetAndShoot(
        mDrivetrain, mVision, mIndexer, mShooter,
        () -> -SwerveUtils.squareInputs(mDriveController.getLeftY(), ControllerConstants.kDriveDeadband),
        () -> -SwerveUtils.squareInputs(mDriveController.getLeftX(), ControllerConstants.kDriveDeadband)
      )
    );
    
    // Failsafe for shooting without a target
    mDriveController.rightTrigger().whileTrue(
      new SpinUpShooterAndIndex(
        mIndexer, mShooter, mVision
      )
    );
    // #endregion
    
    // #region Operator Controls
    
    // run SAM roller if SAM is active until break beam is hit, otherwise run Intake/Indexer
    mOperatorController.rightBumper().whileTrue(
      new ConditionalCommand(
        mSamRoller.runRoller(SAMConstants.kSAMspeedOut),
          // () -> mSamRoller.hasNoteHitBreakbeam(mSourceAmpMech::getCurrentSetpoint)),
        new IntakeInAndIndex(mIntake, mIndexer), 
        mSourceAmpMech::isEnabled)
    );
    
    // run SAM roller if SAM is active, otherwise run Intake/Indexer
    mOperatorController.leftBumper().whileTrue(
      new ConditionalCommand(
        mSamRoller.runRoller(SAMConstants.kSAMspeedEject), 
        new IntakeOutAndIndex(mIntake, mIndexer), 
        mSourceAmpMech::isEnabled)
    );

    // set sam position to INTAKE_SOURCE, then handoff to indexer on interrupt
    mOperatorController.y().whileTrue(
      mSourceAmpMech.runSAM(SAMPositions.INTAKE_SOURCE)
    ).onFalse(new HandoffToIndexer(mSourceAmpMech, mSamRoller, mIntake, mIndexer));

    // handoff to SAM, then stay at EJECT_AMP, then handoff to indexer on interrupt
    mOperatorController.x().whileTrue(
      new HandoffFromIndexer(mSourceAmpMech, mSamRoller, mIntake, mIndexer)
    ).onFalse(new HandoffToIndexer(mSourceAmpMech, mSamRoller, mIntake, mIndexer));

    mOperatorController.rightTrigger().whileTrue(
      new LobNote(mIndexer, mShooter)
    );
    // #endregion
  }

  public void configureNamedCommands(){
    NamedCommands.registerCommand("intake without indexing", mIntake.intakeRing(IntakeConstants.kIntakeSpeedIn));
    NamedCommands.registerCommand("shoot", new SpinUpShooterAndIndex(mIndexer, mShooter, mVision));
    NamedCommands.registerCommand("index without shooting", mIndexer.runLowerIndexer(IndexerConstants.kIndexerSpeedIn));
    NamedCommands.registerCommand("index to shoot", mIndexer.runAllIndexer(IndexerConstants.kIndexerSpeedIn));
    NamedCommands.registerCommand("run shooter", mShooter.shootRing(() -> mVision.getDistance()));
    NamedCommands.registerCommand("track target", 
      mDrivetrain.trackTarget(mVision::getYaw, () -> 0, () -> 0, mVision::getDistance));
  }

  public Command getAutonomousCommand() {
    return mAutoPicker.getSelected();
  }

  public Command getDisabledCommand() {
    return mLED.getDisabledCommand();
  }
}
