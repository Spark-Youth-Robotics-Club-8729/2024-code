package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.ClimbConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class ClimbSubsystem extends SubsystemBase {
    CANSparkMax ClimbSpinMotor;
    RelativeEncoder ClimbEncoder;

    private final String CLIMBER_SPEED_LOG_PATH = "/Climber/Speeds";
    private final String CLIMBER_ENCODER_LOG_PATH = "/Climber/Encoder";

    public ClimbSubsystem() {
        ClimbSpinMotor = new CANSparkMax(ClimbConstants.ClimbSpinMotorCanID, MotorType.kBrushless);
        ClimbEncoder = ClimbSpinMotor.getEncoder();

    }

    public void setSpin(double speed) {
        ClimbSpinMotor.set(speed);
    }

    public double getSpin(){
        return ClimbSpinMotor.get();
    }

    public double getEncoder(){
        return ClimbEncoder.getPosition();
    }

    public void logOutputs(){
        Logger.recordOutput(getName() + CLIMBER_SPEED_LOG_PATH, getSpin());
        Logger.recordOutput(getName() + CLIMBER_ENCODER_LOG_PATH, getEncoder());
    }

}