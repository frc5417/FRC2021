// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import frc.robot.subsystems.*;
import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoIntake extends CommandBase {
private final Intake in;
  /** Creates a new AutoIntake. */
  public AutoIntake(Intake subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    in = subsystem;
    addRequirements(in);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Robot.intake.deployPistons(true, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.intake.runIntakeSystem(1, 0, false, false, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.intake.runIntakeSystem(0, 0, false, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Robot.autoStraight.isFinished();
  }
}
