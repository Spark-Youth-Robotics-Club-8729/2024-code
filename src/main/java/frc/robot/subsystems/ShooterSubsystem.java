package frc.robot.subsystems;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    CANSparkMax leftShooterWheel;
    CANSparkMax rightShooterWheel;
    RelativeEncoder leftShooterWheelEncoder;
    RelativeEncoder rightShooterWheelEncoder;
    PIDController shooterRightPID;
    PIDController shooterLeftPID;

    public ShooterSubsystem() { // settings + configs
        // initializing shooter wheels
        leftShooterWheel = new CANSparkMax(ShooterConstants.ShooterLeftCanID, MotorType.kBrushless);
        rightShooterWheel = new CANSparkMax(ShooterConstants.ShooterRightCanID, MotorType.kBrushless);

        leftShooterWheelEncoder = leftShooterWheel.getEncoder();
        rightShooterWheelEncoder = rightShooterWheel.getEncoder();

        shooterRightPID = new PIDController(0.0001, 0, 0);
        shooterLeftPID = new PIDController(0.0001, 0, 0);

        // invert -> both wheels spin both ways
        leftShooterWheel.setInverted(true);
    }

    public synchronized void setShooter(double speed) {
        leftShooterWheel.set(speed);
        rightShooterWheel.set(speed);
    }

    public synchronized void setShooterVelocity(double rpm) {
        leftShooterWheel.set(rpm / 5676 + shooterLeftPID.calculate(leftShooterWheelEncoder.getVelocity(), rpm));
        rightShooterWheel.set(rpm / 5676 + shooterRightPID.calculate(rightShooterWheelEncoder.getVelocity(), rpm));
    }

    public void periodic() {
        SmartDashboard.putNumber("Left Velocity PID Speed", ShooterConstants.ShooterDesiredRPM / 5676
                + shooterLeftPID.calculate(leftShooterWheelEncoder.getVelocity(), ShooterConstants.ShooterDesiredRPM));
        SmartDashboard.putNumber("Right Velocity PID Speed", ShooterConstants.ShooterDesiredRPM / 5676 + shooterRightPID
                .calculate(rightShooterWheelEncoder.getVelocity(), ShooterConstants.ShooterDesiredRPM));

    }
}
