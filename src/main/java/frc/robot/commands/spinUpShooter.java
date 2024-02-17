package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Shooter;

public class SpinUpShooter extends ParallelCommandGroup {
    final Shooter mShooter;

    public SpinUpShooter(Shooter shooter) {
        mShooter = shooter;

        addCommands(
            mShooter.shootRing()
        );

        addRequirements(mShooter);
    }
}
