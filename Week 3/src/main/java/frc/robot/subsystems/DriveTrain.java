// Based upon 2021's Competition Season DriveTrain code

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.kauailabs.navx.frc.AHRS;

public class DriveTrain extends SubsystemBase 
{
  /** Creates a new ExampleSubsystem. */
  private final WPI_TalonSRX _leftDriveTalon;
  private final WPI_TalonSRX _rightDriveTalon;
  private final VictorSPX _leftDriveVictor;
  private final VictorSPX _rightDriveVictor;

  private AHRS navx = new AHRS(SPI.Port.kMXP);
 
  public DriveTrain() 
  {
    _leftDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.LeftDriveTalonPort);
    _rightDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.RightDriveTalonPort);
    _leftDriveVictor = new VictorSPX(Constants.DriveTrainPorts.LeftDriveVictorPort);
    _rightDriveVictor = new VictorSPX(Constants.DriveTrainPorts.RightDriveVictorPort);

    _leftDriveVictor.follow(_leftDriveTalon);
    _rightDriveVictor.follow(_rightDriveTalon);

    _leftDriveTalon.setInverted(true);
    _rightDriveTalon.setInverted(false);
    _leftDriveVictor.setInverted(InvertType.FollowMaster);
    _rightDriveVictor.setInverted(InvertType.FollowMaster);

    _leftDriveTalon.configFactoryDefault();
    _leftDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    _rightDriveTalon.configFactoryDefault();


    _rightDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    if (Math.abs(leftSpeed) < 0.1){
      leftSpeed = 0;
    }
    if (Math.abs(rightSpeed) < 0.1){
      rightSpeed = 0;
    } 
    _rightDriveTalon.set(ControlMode.PercentOutput, -rightSpeed);  
    _leftDriveTalon.set(ControlMode.PercentOutput, -leftSpeed);

    /*SmartDashboard.putNumber("leftPow:", leftSpeed);
    SmartDashboard.putNumber("rightPow:", rightSpeed);*/
  }

  public void resetEncoders() {
    _leftDriveTalon.setSelectedSensorPosition(0,0,10);
    _rightDriveTalon.setSelectedSensorPosition(0,0,10);
  }

  public double getDisplacement() {
    return (getTicks() * (Constants.DriveToLineConstants.ticksToMeters));  // average distance of both left and right
  }

  public double getTicks() {
    return (_leftDriveTalon.getSelectedSensorPosition(0) + _rightDriveTalon.getSelectedSensorPosition(0)) / 2;
  }

  public double getAngleAndReset(){
    double degrees = navx.getAngle();
    navx.reset();
    return degrees;
  }
 
  public double getAngle(){
    return navx.getAngle(); 
  }
 
  public void resetN(){
    navx.reset();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("NavX angle", getAngle());
    SmartDashboard.putNumber("Left Native Velocity", _leftDriveTalon.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Right Native Velocity", _leftDriveTalon.getSelectedSensorVelocity());
  }

  @Override
  public void simulationPeriodic() 
  {
    // This method will be called once per scheduler run during simulation
  }
}