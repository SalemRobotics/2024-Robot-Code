package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    CANSparkMax shooterSparkMaxOne = new CANSparkMax(0,MoterType.kBrushless);
    CANSparkMax shooterSparkMaxTwo = new CANSparkMax(0,MoterType.kBrushless);

    public Shooter() {
        shooterSparkMaxTwo.follow(shooterSparkMaxOne);
    }


}
