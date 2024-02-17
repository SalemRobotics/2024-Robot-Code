package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;

public final class AutoPicker {
    static Command mCurrentCommand;

    public AutoPicker() {
        AutoConstants.kAutoFolders.forEach(this::setupChooser);
    }

    void setupChooser(String folderName, SendableChooser<Command> chooser) {
        chooser.onChange(this::setCurrentCommand);
        SmartDashboard.putData(folderName, chooser);
    }

    void setCurrentCommand(Command command) {
        mCurrentCommand = command;
        SmartDashboard.putString("Current Auto", mCurrentCommand.getName());
    }

    public static Command getSelected() {
        return mCurrentCommand;
    }
}
