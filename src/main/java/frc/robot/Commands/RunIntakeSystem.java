/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import frc.robot.Robot;

public class RunIntakeSystem extends CommandBase {
  private final Intake i;
  /**
   * Creates a new RunIntakeSystem.
   */
  public RunIntakeSystem(Intake subsystem) {
    i = subsystem;
    addRequirements(i);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.intake.runIntakeSystem(Robot.robotContainer.rTriggerM(), Robot.robotContainer.lTriggerM(), Robot.robotContainer.bButtonM(), Robot.robotContainer.xButtonM(), Robot.robotContainer.yButtonM());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
