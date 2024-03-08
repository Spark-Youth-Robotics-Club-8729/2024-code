// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeSetRotation extends Command {
  private final IntakeSubsystem m_intakesubsystem;
  private final double m_speed;

  /** Creates a new IntakeSet. */
  public IntakeSetRotation(IntakeSubsystem intakesubsystem, double speed) {
    m_speed = speed;
    m_intakesubsystem = intakesubsystem;
    addRequirements(intakesubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakesubsystem.setRotate(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakesubsystem.setRotate(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
