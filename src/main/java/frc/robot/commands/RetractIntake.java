// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.IntakeConstants;

public class RetractIntake extends Command {
  private final IntakeSubsystem m_intakesubsystem;
  private final double m_rotationSpeed;

  /** Creates a new RetractIntake. */
  public RetractIntake(IntakeSubsystem intakeSubsystem, double rotationOutSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intakesubsystem = intakeSubsystem;
    m_rotationSpeed = rotationOutSpeed;
    addRequirements(m_intakesubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_intakesubsystem.getRotationEncoder() < IntakeConstants.rotationIn) {
      m_intakesubsystem.setRotate(-m_rotationSpeed);
    } else if(m_intakesubsystem.getRotationEncoder()<-0.8){
        m_intakesubsystem.setRotate(-0.8);
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakesubsystem.setRotate(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_intakesubsystem.getRotationEncoder()<-0.3){
      return false;
    }
    else{
      return true;
    }
  }
}
