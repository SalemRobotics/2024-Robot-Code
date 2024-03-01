package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    final CANSparkMax mIntakeMotor = new CANSparkMax(IntakeConstants.kSparkMaxID, MotorType.kBrushless);
    
    final DigitalInput mBreakBeam = new DigitalInput(IntakeConstants.kBreakBeamChannel);
    public final Trigger mRingInRobot = new Trigger(() -> mBreakBeam.get());

    public Command intakeRing(double speed) {
        return runEnd(
            () -> {
                double newSpeed = mBreakBeam.get() ? -speed : speed;
                mIntakeMotor.set(speed);
            }, 
            () -> mIntakeMotor.stopMotor()
        );
    }
}
