package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Shooter;

public class RunShooter extends ParallelCommandGroup {
    final Shooter mShooter;

    public spinUpShooter(Shooter shooter) {
        mShooter = shooter;

        addCommands(
            mShooter.shootRing()
        );

        addRequirements(mShooter);
    }
}
