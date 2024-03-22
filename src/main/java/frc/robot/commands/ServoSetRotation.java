// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ServoSubsystem;


public class ServoSetRotation extends Command {
  private final ServoSubsystem m_servosubsystem;
  private final double m_angle;

  /** Creates a new ServoSet. */
  public ServoSetRotation(ServoSubsystem servosubsystem, double angle) {
    m_angle = angle;
    m_servosubsystem = servosubsystem;
    addRequirements(servosubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_servosubsystem.setServo(m_angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_servosubsystem.setServo(90, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
