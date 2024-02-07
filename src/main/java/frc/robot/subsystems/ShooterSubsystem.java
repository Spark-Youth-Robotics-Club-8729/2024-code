package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  CANSparkMax leftShooterWheel;
  CANSparkMax rightShooterWheel;

  public ShooterSubsystem(int ShooterLeftCanID, int ShooterRightCanID) { // settings + configs
    // initializing shooter wheels
    leftShooterWheel = new CANSparkMax(ShooterLeftCanID, MotorType.kBrushless);
    rightShooterWheel = new CANSparkMax(ShooterRightCanID, MotorType.kBrushless);

    // brake mode
    leftShooterWheel.setIdleMode(ShooterConstants.ShooterLeftIdleMode);
    rightShooterWheel.setIdleMode(ShooterConstants.ShooterRightIdleMode);

    // burn flash
    leftShooterWheel.burnFlash();
    rightShooterWheel.burnFlash();

    // invert -> both wheels spin both ways
    leftShooterWheel.setInverted(true);
  }

  public synchronized void setShooter(double speed) {
    leftShooterWheel.set(speed);
    rightShooterWheel.set(speed);
  }
}
