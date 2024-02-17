// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeSetSpin extends Command {
  IntakeSubsystem intakesubsystem;
  double speed;
  /** Creates a new IntakeSet. */
  public IntakeSetSpin(IntakeSubsystem intakesubsystem, double speed) {
    this.speed = speed;
    this.intakesubsystem = intakesubsystem;
    addRequirements(intakesubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakesubsystem.setSpin(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakesubsystem.setSpin(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
