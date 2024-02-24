package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;

public class AutoPicker {
    Command mCurrentCommand;
    String mCurrentName;
    final HashMap<Command, String> mCommandNameLookup = new HashMap<>();

    public AutoPicker() {
        AutoConstants.kAutoFolders.forEach(this::setupChooser);
    }

    void setupChooser(String folderName, List<String> names) {
        var chooser = new SendableChooser<Command>();

        chooser.setDefaultOption("None", null);
        mCommandNameLookup.putIfAbsent(null, "None");
        
        names.forEach((name) -> createChooserOptions(chooser, name));
        chooser.onChange((command) -> setCurrentCommand(chooser, folderName));
        
        SmartDashboard.putData(folderName, chooser);
    }

    void createChooserOptions(SendableChooser<Command> chooser, String name) {
        var command = AutoBuilder.buildAuto(name);
        mCommandNameLookup.putIfAbsent(command, name);
        chooser.addOption(name, command);
    }

    void setCurrentCommand(SendableChooser<Command> chooser, String name) {
        mCurrentCommand = chooser.getSelected();
        SmartDashboard.putString("Current Auto", mCommandNameLookup.get(mCurrentCommand));
    }

    public Command getSelected() {
        return mCurrentCommand;
    }
}
