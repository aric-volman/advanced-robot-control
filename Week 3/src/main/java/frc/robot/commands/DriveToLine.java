// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrain;

public class DriveToLine extends CommandBase {
  /** Creates a new DriveDistance. */
  DriveTrain dt;
  boolean finished;

  double displacement; // Displacement in meters
  double _drivespeed;

  public DriveToLine(DriveTrain driveTrain, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    dt = driveTrain;
    addRequirements(dt);
    displacement = 1.0;
    _drivespeed = (displacement >= 0 ? 1 : -1) * Math.abs(speed);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
    dt.tankDrive(0.0, 0.0);
    dt.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // The magnitude of the displacement vector is independent of direction
    if (Math.abs(dt.getDisplacement()) < Math.abs(displacement)) {
      SmartDashboard.putNumber("Average Displacement", Math.abs(dt.getDisplacement()));
      // The direction of the displacement vector is independent of magnitude
      dt.tankDrive(_drivespeed, _drivespeed);
    }
    else {
      finished = true;
      dt.tankDrive(0.0, 0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.tankDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }

}
