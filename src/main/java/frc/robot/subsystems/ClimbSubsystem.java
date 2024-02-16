package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.ClimbConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
    CANSparkMax ClimbSpinMotor;

    public ClimbSubsystem() {
        ClimbSpinMotor = new CANSparkMax(ClimbConstants.ClimbSpinMotorCanID, MotorType.kBrushless);
        
    }  
    
    public void setSpin(double speed){
        ClimbSpinMotor.set(speed);
    }

}