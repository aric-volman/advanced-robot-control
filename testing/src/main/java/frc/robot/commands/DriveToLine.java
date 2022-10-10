// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveTrain;

public class DriveToLine extends CommandBase {
  /** Creates a new DriveToLine. */

  DriveTrain dt;
  double displacement; // Displacement in meters
  double drivespeed;


  public DriveToLine(DriveTrain dt, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    // Use addRequirements() here to declare subsystem dependencies.
    this.dt = dt;
    displacement = 1.0 - dt.getDTLOffset();
    drivespeed = dt.getDTLDirection() * Math.abs(speed);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    displacement = 1.0 - dt.getDTLOffset();
    drivespeed = dt.getDTLDirection() * Math.abs(drivespeed);
    dt.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // The magnitude of the displacement vector is independent of its direction
    dt.tankDrive(drivespeed, drivespeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(dt.getDisplacement()) >= Math.abs(displacement);
  }
}
