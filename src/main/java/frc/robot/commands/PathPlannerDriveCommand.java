package frc.robot.commands;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class PathPlannerDriveCommand {
    final Drivetrain drive;
    final SendableChooser<Command> m_chooser = new SendableChooser<>();

    public PathPlannerDriveCommand(Drivetrain drive){
        this.drive = drive;

        m_chooser.setDefaultOption(Constants.AutoConstants.kTestAuto, AutoBuilder.buildAuto(Constants.AutoConstants.kTestAuto));
    }

}
