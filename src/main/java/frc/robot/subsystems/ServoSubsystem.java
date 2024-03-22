// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ServoSubsystem extends SubsystemBase {
  Servo servoLeft;
  Servo servoRight;

  /** Creates a new ServoSubsystem. */
  public ServoSubsystem() {
    servoRight = new Servo(2);
    servoLeft = new Servo(3);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setServo(double angle) {
    servoRight.setAngle(angle);
    servoLeft.setAngle(angle-180);
  }

  public void setServo(double angle, boolean sync) {
      servoRight.setAngle(angle);
      servoLeft.setAngle(angle);
  }
}
