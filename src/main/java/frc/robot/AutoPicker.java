package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

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

    /**
     * Sets up each of the sendable choosers per each folder
     * @param folderName Name of the folder to get command names from
     * @param names List of command names
     * @see List
     */
    private void setupChooser(String folderName, List<String> names) {
        var chooser = new SendableChooser<Command>();

        // creates default option, to avoid issues with folders containing only one command
        chooser.setDefaultOption("None", null);
        mCommandNameLookup.putIfAbsent(null, "None");
        
        names.forEach((name) -> createChooserOptions(chooser, name));
        chooser.onChange((command) -> setCurrentCommand(chooser));
        
        SmartDashboard.putData(folderName, chooser);
    }

    /**
     * Creates options for each of the commands using AutoBuilder and places them in their respective chooser
     * @param chooser Sendable chooser to create options for
     * @param name Name of the command option 
     * @see SendableChooser
     * @see Command
     * @see AutoBuilder
     */
    private void createChooserOptions(SendableChooser<Command> chooser, String name) {
        var command = new PathPlannerAuto(name);
        mCommandNameLookup.putIfAbsent(command, name);
        chooser.addOption(name, command);
    }

    /**
     * Called any time the chooser is changed and sets current command to 
     * the selected command from whichever chooser has been changed
     * @param chooser Chooser to get selected command from
     * @see SendableChooser
     */
    private void setCurrentCommand(SendableChooser<Command> chooser) {
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
