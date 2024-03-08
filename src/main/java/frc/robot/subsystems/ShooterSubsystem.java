package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
    CANSparkMax leftShooterWheel;
    CANSparkMax rightShooterWheel;
    RelativeEncoder leftShooterWheelEncoder;
    RelativeEncoder rightShooterWheelEncoder;
    PIDController shooterRightPID;
    PIDController shooterLeftPID;

    private final String LEFT_WHEEL_LOG_PATH = "/Speeds/LeftWheel";
    private final String RIGHT_WHEEL_LOG_PATH = "/Speeds/RightWheel";
    private final String LEFT_ENCODER_LOG_PATH = "/Encoder/LeftEncoder";
    private final String RIGHT_ENCODER_LOG_PATH = "/Encoder/RightEncoder";
    // private final String LEFT_PID_LOG_PATH = "/PID/LeftPID";
    // private final String RIGHT_PID_LOG_PATH = "/PID/RightPID";
    private final String LEFT_VELOCITY_LOG_PATH = "/Velocity/LeftPIDVelocity";
    private final String RIGHT_VELOCITY_LOG_PATH = "/Velocity/RIghtPIDVelocity";

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

    public synchronized void setShooter(double leftSpeed, double rightSpeed) {
        leftShooterWheel.set(leftSpeed);
        rightShooterWheel.set(rightSpeed);
    }

    public synchronized void setShooterVelocity(double rpm) {
        leftShooterWheel.set(rpm / 5676 + shooterLeftPID.calculate(leftShooterWheelEncoder.getVelocity(), rpm));
        rightShooterWheel.set(rpm / 5676 + shooterRightPID.calculate(rightShooterWheelEncoder.getVelocity(), rpm));
    }

    public void periodic() {
        SmartDashboard.putNumber("Left Wheel Velocity PID Speed", ShooterConstants.ShooterDesiredRPM / 5676
                + shooterLeftPID.calculate(leftShooterWheelEncoder.getVelocity(), ShooterConstants.ShooterDesiredRPM));
        SmartDashboard.putNumber("Right Wheel Velocity PID Speed", ShooterConstants.ShooterDesiredRPM / 5676 + shooterRightPID
                .calculate(rightShooterWheelEncoder.getVelocity(), ShooterConstants.ShooterDesiredRPM));
        logOutputs();
    }

    /// Log ShooterSubsystem outouts
    private void logOutputs() {
    Logger.recordOutput(getName() + LEFT_WHEEL_LOG_PATH, getLeftShooter());
    Logger.recordOutput(getName() + RIGHT_WHEEL_LOG_PATH, getRightShooter());
    Logger.recordOutput(getName() + LEFT_ENCODER_LOG_PATH, getLeftEncoder());
    Logger.recordOutput(getName() + RIGHT_ENCODER_LOG_PATH, getRightEncoder());
    // Logger.recordOutput(getName() + LEFT_PID_LOG_PATH, getLeftPID());
    // Logger.recordOutput(getName() + RIGHT_PID_LOG_PATH, getLeftPID());
    Logger.recordOutput(getName() + LEFT_VELOCITY_LOG_PATH, getLeftPIDVelocity());
    Logger.recordOutput(getName() + RIGHT_VELOCITY_LOG_PATH, getRightPIDVelocity());
    }

    public double getLeftShooter(){
        return leftShooterWheel.get();
    }

    public double getRightShooter(){
        return rightShooterWheel.get();
    }

    public double getLeftEncoder(){
        return leftShooterWheelEncoder.getPosition();
    }

    public double getRightEncoder(){
        return rightShooterWheelEncoder.getPosition();
    }

    // public double getLeftPID(){
    //     return shooterLeftPID.calculate(leftShooterWheelEncoder.getVelocity(), ShooterConstants.ShooterDesiredRPM);
    // }

    // public double getRightPID(){
    //     return shooterRightPID.calculate(rightShooterWheelEncoder.getVelocity(), ShooterConstants.ShooterDesiredRPM);
    // }

    public double getLeftPIDVelocity(){
        return ShooterConstants.ShooterDesiredRPM / 5676 + shooterLeftPID.calculate(leftShooterWheelEncoder.getVelocity(), ShooterConstants.ShooterDesiredRPM);
    }

    public double getRightPIDVelocity(){
        return ShooterConstants.ShooterDesiredRPM / 5676 + shooterRightPID.calculate(rightShooterWheelEncoder.getVelocity(), ShooterConstants.ShooterDesiredRPM);
    }
}
