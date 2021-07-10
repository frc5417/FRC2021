// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.*;
import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoDriveBack extends CommandBase {
  private final Drive d;
  private double time;
  /** Creates a new SmallUTurn. */
  public AutoDriveBack(Drive subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    d = subsystem;
    time = 0.0;
    addRequirements(d);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("you have reached the uturn");
    time += 20;
    d.SetPower(-.6, -.6);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    time = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return time > 2000;
  }
}