package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    CANSparkMax IntakeSpinMotor;
    CANSparkMax IntakeRotateMotor;

    public IntakeSubsystem() {
        IntakeSpinMotor = new CANSparkMax(IntakeConstants.IntakeSpinMotorCanID, MotorType.kBrushless);
        IntakeRotateMotor = new CANSparkMax(IntakeConstants.IntakeRotateMotorCanID, MotorType.kBrushless);
    }

    public void setSpin(double speed) {
        IntakeSpinMotor.set(speed);
    }

    public void setRotate(double speed) {
        IntakeRotateMotor.set(speed);
    }
    
}
