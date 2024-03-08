// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.IntakeConstants;

public class IntakeAndRetract extends Command {
  private final IntakeSubsystem m_intakesubsystem;
  private final double m_rotationSpeed;
  private final double m_intakeSpeed;

  /** Creates a new IntakeAndRetract. */
  public IntakeAndRetract(IntakeSubsystem intakeSubsystem, double rotationOutSpeed, double intakeSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intakesubsystem = intakeSubsystem;
    m_rotationSpeed = rotationOutSpeed;
    m_intakeSpeed = intakeSpeed;
    addRequirements(m_intakesubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_intakesubsystem.getRotationEncoder() > IntakeConstants.rotationOut) {
      m_intakesubsystem.setRotate(m_rotationSpeed);
      // System.out.println("INTAKE OUT");
    } else {
      m_intakesubsystem.setRotate(0.0);
      m_intakesubsystem.setSpin(m_intakeSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (m_intakesubsystem.getRotationEncoder() < IntakeConstants.rotationIn) {
      m_intakesubsystem.setRotate(-m_rotationSpeed);
    } else if(m_intakesubsystem.getRotationEncoder()<-0.8){
        m_intakesubsystem.setRotate(-0.8);
      }
    else{
      m_intakesubsystem.setRotate(0.0);
    }

    m_intakesubsystem.setSpin(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
