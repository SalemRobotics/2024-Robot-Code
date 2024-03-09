package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;

/** Class to pick an auto from a multitude of 'folders' from Shuffleboard */
public class AutoPicker {
    Command mCurrentCommand;
    String mCurrentName;
    final HashMap<Command, String> mCommandNameLookup = new HashMap<>();

    public AutoPicker() {
        AutoConstants.kAutoFolders.forEach(this::setupChooser);
    }

    /** Sets up each of the sendable choosers per each folder */
    void setupChooser(String folderName, List<String> names) {
        var chooser = new SendableChooser<Command>();

        // creates default option, to avoid issues with folders containing only one command
        chooser.setDefaultOption("None", null);
        mCommandNameLookup.putIfAbsent(null, "None");
        
        names.forEach((name) -> createChooserOptions(chooser, name));
        chooser.onChange((command) -> setCurrentCommand(chooser, folderName));
        
        SmartDashboard.putData(folderName, chooser);
    }

    /** Creates options for each of the commands and places them in their respective chooser */
    void createChooserOptions(SendableChooser<Command> chooser, String name) {
        var command = AutoBuilder.buildAuto(name);
        mCommandNameLookup.putIfAbsent(command, name);
        chooser.addOption(name, command);
    }

    /** Called any time the chooser is changed and sets current command to 
     * the selected command from whichever chooser has been changed */
    void setCurrentCommand(SendableChooser<Command> chooser, String name) {
        mCurrentCommand = chooser.getSelected();
        SmartDashboard.putString("Current Auto", mCommandNameLookup.get(mCurrentCommand));
    }

    /**
     * Gets the selected active command from the auto folders.
     * @return Selected Command
     * @see Command
     */
    public Command getSelected() {
        return mCurrentCommand;
    }
}
