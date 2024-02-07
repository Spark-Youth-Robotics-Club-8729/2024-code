package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSubsystem extends SubsystemBase {
    CANSparkMax IntakeSpinMotor;
    CANSparkMax IntakeRotateMotor;
    RelativeEncoder IntakeRotateEncoder;

    private final PIDController rotationPID = new PIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD);

    public IntakeSubsystem() {
        IntakeSpinMotor = new CANSparkMax(IntakeConstants.IntakeSpinMotorCanID, MotorType.kBrushless);
        IntakeRotateMotor = new CANSparkMax(IntakeConstants.IntakeRotateMotorCanID, MotorType.kBrushless);
        IntakeRotateEncoder = IntakeRotateMotor.getEncoder();
    }

    public void setSpin(double speed) {
        IntakeSpinMotor.set(speed);
    }

    public void setRotate(double speed) {
        IntakeRotateMotor.set(speed);
    }

    public void rotatePID(){
        IntakeRotateMotor.set(-rotationPID.calculate(IntakeConstants.PIDSetpoint, IntakeRotateEncoder.getPosition())+IntakeConstants.stall);
    }

    public void periodic(){
      SmartDashboard.putNumber("Rotation Encoder",IntakeRotateEncoder.getPosition());
      SmartDashboard.putNumber("Rotation actual speed", -rotationPID.calculate(IntakeConstants.PIDSetpoint, IntakeRotateEncoder.getPosition())+IntakeConstants.stall);
      SmartDashboard.putNumber("Rotation PID Speed", -rotationPID.calculate(IntakeConstants.PIDSetpoint, IntakeRotateEncoder.getPosition()));
    }   


    
}
