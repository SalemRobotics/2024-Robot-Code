package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.StrongArmMachine;

public class ScoreAmp extends SequentialCommandGroup {
    final StrongArmMachine mSAM;

    /**
     * While holding a note, dunk it into the amp.
     * @param sam
     */
    public ScoreAmp(StrongArmMachine sam) {
        mSAM = sam;

        addCommands(
            mSAM.scoreAmp()
        );

        addRequirements(mSAM);
    }
}