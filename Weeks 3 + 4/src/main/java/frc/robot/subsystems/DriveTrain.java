// Based upon 2021's Competition Season DriveTrain code

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants;
import frc.robot.Robot;

import com.kauailabs.navx.frc.AHRS;

public class DriveTrain extends SubsystemBase 
{
  /** Creates a new ExampleSubsystem. */
  private final WPI_TalonSRX leftDriveTalon;
  private final WPI_TalonSRX rightDriveTalon;
  private final VictorSPX _leftDriveVictor;
  private final VictorSPX _rightDriveVictor;

  // private final DifferentialDrive diff;

  private AHRS navx = new AHRS(SPI.Port.kMXP);

  private double DriveToLineDirection = 1.0;
  private double DriveToLineOffset = 0.0;

  private ShuffleboardTab DTLTab = Shuffleboard.getTab("Drive To Line");
  private NetworkTableEntry SwitchDirection = DTLTab.add("Direction", 1.0).getEntry();
  private NetworkTableEntry DTLDisplacement = DTLTab.add("Displacement", 0.0).getEntry();
  private NetworkTableEntry DTLOffset = DTLTab.add("Offset", 0.0).getEntry();
  private NetworkTableEntry LeftVelocity = DTLTab.add("Left Native Velocity", 0.0).getEntry();
  private NetworkTableEntry RightVelocity = DTLTab.add("Right Native Velocity", 0.0).getEntry();

  // Week 4
  private NetworkTableEntry leftTalonkP = DTLTab.add("Left kP", 5.0).getEntry();
  private NetworkTableEntry rightTalonkP = DTLTab.add("Right kP", 5.0).getEntry();
  private NetworkTableEntry leftTalonAccel = DTLTab.add("Left MM Accel", 510.0).getEntry();
  private NetworkTableEntry rightTalonAccel = DTLTab.add("Right MM Accel", 300.0).getEntry();

  public DriveTrain() 
  {
    leftDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.LeftDriveTalonPort);
    rightDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.RightDriveTalonPort);
    _leftDriveVictor = new VictorSPX(Constants.DriveTrainPorts.LeftDriveVictorPort);
    _rightDriveVictor = new VictorSPX(Constants.DriveTrainPorts.RightDriveVictorPort);

    _leftDriveVictor.follow(leftDriveTalon);
    _rightDriveVictor.follow(rightDriveTalon);
  
    leftDriveTalon.setNeutralMode(NeutralMode.Coast);
    rightDriveTalon.setNeutralMode(NeutralMode.Coast);

    leftDriveTalon.setInverted(false);
    rightDriveTalon.setInverted(true);
    _leftDriveVictor.setInverted(InvertType.FollowMaster);
    _rightDriveVictor.setInverted(InvertType.FollowMaster);

    leftDriveTalon.setSensorPhase(true);
    rightDriveTalon.setSensorPhase(true);

    leftDriveTalon.configFactoryDefault();
    leftDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    rightDriveTalon.configFactoryDefault();
    rightDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    
    // Week 4 Motion Magic
   leftDriveTalon.config_kP(0, 3, 10);
   leftDriveTalon.config_kI(0, 0, 10);
   leftDriveTalon.config_kD(0, 0, 10);
   leftDriveTalon.configMotionAcceleration(220, 10);
   leftDriveTalon.configMotionCruiseVelocity(400, 10);
   // If Left Velocity is 200
   // then Left Accel 120

   rightDriveTalon.config_kP(0, 3, 10);
   rightDriveTalon.config_kI(0, 0, 10);
   rightDriveTalon.config_kD(0, 0, 10);
   rightDriveTalon.configMotionAcceleration(200, 10);
   rightDriveTalon.configMotionCruiseVelocity(400, 10);
   // If  Right Velocity is 200
   // then Right Accel 90

  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    //diff.tankDrive(leftSpeed, rightSpeed);
    rightDriveTalon.set(rightSpeed);
    leftDriveTalon.set(leftSpeed);
  }

  public void magicDrive(double displacement) {
    leftDriveTalon.set(ControlMode.MotionMagic, Constants.DriveToLineConstants.ticksToMeters*displacement);
    rightDriveTalon.set(ControlMode.MotionMagic, Constants.DriveToLineConstants.ticksToMeters*displacement);
  }

  public void resetEncoders() {
    leftDriveTalon.setSelectedSensorPosition(0,0,10);
    rightDriveTalon.setSelectedSensorPosition(0,0,10);
  }

  public double getDisplacement() {
    return (getTicks() / (Constants.DriveToLineConstants.ticksToMeters));
  }

  public double getDTLOffset() {
    return DriveToLineOffset;
  }

  public double getDTLDirection() {
    return DriveToLineDirection;
  }

  public double getTicks() {
    return (leftDriveTalon.getSelectedSensorPosition(0) + rightDriveTalon.getSelectedSensorPosition(0)) / 2.0;
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
    SmartDashboard.putNumber("Left Voltage", leftDriveTalon.getMotorOutputPercent());
    SmartDashboard.putNumber("Right Voltage", rightDriveTalon.getMotorOutputPercent());
    SmartDashboard.putNumber("Left Native Velocity", leftDriveTalon.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Right Native Velocity", leftDriveTalon.getSelectedSensorVelocity());

    DriveToLineDirection = SwitchDirection.getDouble(1.0);
    DriveToLineOffset = DTLOffset.getDouble(0.0);

    DTLDisplacement.setDouble(getDisplacement());

    LeftVelocity.setDouble(leftDriveTalon.getSelectedSensorVelocity());
    RightVelocity.setDouble(rightDriveTalon.getSelectedSensorVelocity());
    
    tankDrive(RobotContainer.getJoy1().getY()*-0.2, RobotContainer.getJoy2().getY()*-0.2);

    // Week 4
    leftDriveTalon.config_kP(0, leftTalonkP.getDouble(3.0), 0);
    rightDriveTalon.config_kP(0, rightTalonkP.getDouble(3.0), 0);
    leftDriveTalon.configMotionAcceleration(leftTalonAccel.getDouble(220.0), 0);
    rightDriveTalon.configMotionAcceleration(rightTalonAccel.getDouble(200.0), 0);
  }

  @Override
  public void simulationPeriodic() 
  {
    // This method will be called once per scheduler run during simulation
  }
}