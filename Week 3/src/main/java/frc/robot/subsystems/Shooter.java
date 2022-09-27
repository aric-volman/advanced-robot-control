// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;


import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

// Week 2 - import these below
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTableEntry;

public class Shooter extends SubsystemBase {

  private double flywheelTolerance = 0.05; // Tolerance of PID controller
  private boolean override = false; // Helps us switch from manual to auto
  private Timer overrideTimer = new Timer(); // We want a toggle button of some sorts
  private double overrideTime = 1.0;

  private double angle = 90.0; // Angle at which to start at - 90 above the horizontal
  private double encoderConst = 1.0; // Flips the sign of the angle if needed
  private double pivotTolerance = 1.0; // PID tolerance
  // This is a positive offset that we need to add
  // meaning that the angle returned from the pivot is in fact negative
  private double offset = Constants.angularOffset;

  private final WPI_TalonSRX leftFlywheel = new WPI_TalonSRX(Constants.ShooterPorts.LeftFlywheelPort);
  private final WPI_TalonSRX rightFlywheel = new WPI_TalonSRX(Constants.ShooterPorts.RightFlywheelPort);
  private final WPI_TalonSRX pivot = new WPI_TalonSRX(Constants.ShooterPorts.pivotPort);
  private final WPI_VictorSPX roller = new WPI_VictorSPX(Constants.ShooterPorts.rollerPort);

  private final PIDController leftFlywheelPID = new PIDController(Constants.leftFlywheelPIDConsts.pidP, Constants.leftFlywheelPIDConsts.pidI, Constants.leftFlywheelPIDConsts.pidD);
  private final PIDController rightFlywheelPID = new PIDController(Constants.rightFlywheelPIDConsts.pidP, Constants.rightFlywheelPIDConsts.pidI, Constants.rightFlywheelPIDConsts.pidD);
  private final PIDController pivotPID = new PIDController(Constants.pivotPIDConsts.pidP, Constants.pivotPIDConsts.pidI, Constants.pivotPIDConsts.pidD);

  private SimpleMotorFeedforward leftFlywheelFF = new SimpleMotorFeedforward(Constants.leftFlywheelFF.kS, Constants.leftFlywheelFF.kV, Constants.leftFlywheelFF.kA);
  private SimpleMotorFeedforward rightFlywheelFF = new SimpleMotorFeedforward(Constants.rightFlywheelFF.kS, Constants.rightFlywheelFF.kV, Constants.rightFlywheelFF.kA);
  private ArmFeedforward pivotFF = new ArmFeedforward(Constants.pivotFF.kS, Constants.pivotFF.kG, Constants.pivotFF.kV, Constants.pivotFF.kA);
  
  // Start of Week 2 code section //
  private ShuffleboardTab pidTab = Shuffleboard.getTab("Flywheel PID");
  private NetworkTableEntry leftFlywheelPIDP = pidTab.add("Left Flywheel PID P", Constants.leftFlywheelPIDConsts.pidP).getEntry();
  private NetworkTableEntry leftFlywheelPIDI = pidTab.add("Left Flywheel PID I", Constants.leftFlywheelPIDConsts.pidI).getEntry();
  private NetworkTableEntry leftFlywheelPIDD = pidTab.add("Left Flywheel PID D", Constants.leftFlywheelPIDConsts.pidD).getEntry();

  private NetworkTableEntry rightFlywheelPIDP = pidTab.add("Right Flywheel PID P", Constants.rightFlywheelPIDConsts.pidP).getEntry();
  private NetworkTableEntry rightFlywheelPIDI = pidTab.add("Right Flywheel PID I", Constants.rightFlywheelPIDConsts.pidI).getEntry();
  private NetworkTableEntry rightFlywheelPIDD = pidTab.add("Right Flywheel PID D", Constants.rightFlywheelPIDConsts.pidD).getEntry();

  private NetworkTableEntry pivotPIDP = pidTab.add("Pivot PID P", Constants.pivotPIDConsts.pidP).getEntry();
  private NetworkTableEntry pivotPIDI = pidTab.add("Pivot PID I", Constants.pivotPIDConsts.pidI).getEntry();
  private NetworkTableEntry pivotPIDD = pidTab.add("Pivot PID D", Constants.pivotPIDConsts.pidD).getEntry();  

  /** Creates a new Shooter. */
  public Shooter() {
    // Config talons

    pivot.configFactoryDefault();
    pivot.setInverted(false);
    pivot.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    pivot.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    pivot.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    pivot.configPeakCurrentLimit(20, 5);

    roller.configFactoryDefault();
    roller.setInverted(false);
    
    // Note: Do not use NeutralMode brake with feedforward control!
    // When characterizing, the characterization code doesn't use it!

    leftFlywheel.configFactoryDefault();
    // Inversion affects the output of the motor, but not its sensor's phase
    leftFlywheel.setInverted(true);
    leftFlywheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    rightFlywheel.configFactoryDefault();
    rightFlywheel.setInverted(false);    
    rightFlywheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    // Config PID tolerance
    leftFlywheelPID.setTolerance(flywheelTolerance);
    rightFlywheelPID.setTolerance(flywheelTolerance);
    pivotPID.setTolerance(pivotTolerance);

    // Start and reset the manual override button timer
    // Sort of like the debounce time on your keyboard
    overrideTimer.start(); // Start timer
    overrideTimer.reset(); // Reset timer

  }

  // FLYWHEEL METHODS

  // From ticks per 100 ms, to ticks per s, then to rotations per s, and finally RPM
  public double getLeftRPM() {
    return ((leftFlywheel.getSelectedSensorVelocity() * 10)/4096.0)*60.0;
  }

  public double getRightRPM() {
    return ((rightFlywheel.getSelectedSensorVelocity() * 10)/4096.0)*60.0;
  }

  // Gets percent voltages of flywheels
  public double getLeftFlywheelPower() {
    return leftFlywheel.get();
  }

  public double getRightFlywheelPower() {
    return rightFlywheel.get();
  }

  // Sets percent voltages of flywheels
  public void setFlywheelPower(double speed) {
    leftFlywheel.set(speed);
    rightFlywheel.set(speed);
  }

  public boolean flywheelWithinErrorMargin() {
    return (leftFlywheelPID.atSetpoint() && rightFlywheelPID.atSetpoint());
  }

  // The actual method for PIDF control
  public void setFlywheelConstantVelocity(double RPM) {
    leftFlywheel.setVoltage((leftFlywheelFF.calculate(RPM/60.0)*60.0) + leftFlywheelPID.calculate(getLeftRPM(), RPM));
    rightFlywheel.setVoltage((rightFlywheelFF.calculate(RPM/60.0)*60.0) + rightFlywheelPID.calculate(getRightRPM(), RPM));
  }
  
  public double getAverageRPM() {
    return ((getLeftRPM() + getRightRPM())/2.0);
  }

  public double getFlywheelCurrent() {
    return (leftFlywheel.getStatorCurrent() + rightFlywheel.getStatorCurrent())/2.0;
  }

  public void resetFlywheelEncoders() {
    leftFlywheel.setSelectedSensorPosition(0, 0, 10);
    rightFlywheel.setSelectedSensorPosition(0, 0, 10);
  }

  // PIVOT METHODS

  public void resetPivotEncoders() {
    pivot.setSelectedSensorPosition(0, 0, 10);
  }

  public void setPivotAngle(double angle) {
    this.angle = angle;
  }

  public void updatePivotAnglePIDF() {
    pivotPID.setSetpoint(this.angle);
    // Calculates both PID and Feedforward - the FF test doesn't have an offset,
    // but we can tune the PID to an offsetted angle to make our lives easier
    pivot.setVoltage(pivotPID.calculate(this.angle) + pivotFF.calculate(Math.toRadians(this.angle - offset), Constants.pivotVelocityInRadiansPerSecond));
  }

  public boolean atSetpoint() {
    return pivotPID.atSetpoint();
    // return (this.angle == getPivotPosition()); // We might want to revisit this one
  }

  public boolean isForwardLimitClosed() {
    if (pivot.isFwdLimitSwitchClosed() == 1) {
      return true;
    }
    else {
      return false;
    }
  }

  public boolean isReverseLimitClosed() {
    if (pivot.isRevLimitSwitchClosed() == 1) {
      return true;
    }
    else {
      return false;
    }
  }

  public double getPivotVelocityInRad() {
    // Converts rotations per second to radians per second
    // 2PI is one rotation, so multiplication makes sense
    return ((encoderConst*pivot.getSelectedSensorPosition(0) * 10)/4096.0) * 2 * Math.PI;
  }

  public double getPivotPosition(){
    return (((encoderConst*pivot.getSelectedSensorPosition(0)/4096.0)*360.0) + offset); // OFFSETTED is angle here. The negative const might not be needed
  }

  public double getPivotPositionNotOffset() {
    return (((encoderConst*pivot.getSelectedSensorPosition(0)/4096.0)*360.0));
  }

  public double getRawPivotPosition() {
    return ((encoderConst*pivot.getSelectedSensorPosition(0)) - ((offset/360.0)*4096.0));
  }

  public double getRawPivotPositionNotOffset() {
    return ((encoderConst*pivot.getSelectedSensorPosition(0)));
  }

  public void setPivotPower(double power) {
    pivot.set(power); // This is what was tested March 1st
  }

  public double getPivotPower() {
    return pivot.get();
  }

  public void setPivotBrake() {
    pivot.setNeutralMode(NeutralMode.Brake);
  }

  public void setPivotCoast() {
    pivot.setNeutralMode(NeutralMode.Coast);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Printouts for important numbers
    SmartDashboard.putNumber("Average RPM", getAverageRPM());
    SmartDashboard.putNumber("Average Current", getFlywheelCurrent());
    SmartDashboard.putNumber("Left Flywheel RPM", getLeftRPM());
    SmartDashboard.putNumber("Left Flywheel Power", getLeftFlywheelPower());
    SmartDashboard.putNumber("Right Flywheel RPM", getRightRPM());
    SmartDashboard.putNumber("Right Flywheel Power", getRightFlywheelPower());

    SmartDashboard.putNumber("Pivot Angle", getPivotPosition());
    SmartDashboard.putNumber("Pivot Angle with No Offset", getPivotPositionNotOffset()); // Use this to find the offset angle at horizontal power
    SmartDashboard.putNumber("Raw Pivot Angle with No Offset", getRawPivotPositionNotOffset()); 
    SmartDashboard.putNumber("Pivot Power", getPivotPower());

    SmartDashboard.putNumber("Pivot Velocity In Radians", getPivotVelocityInRad());

    // If the button is pressed, and the timer is over 1.0 s, toggle
    // the manual-to-auto override, and reset the timer.
    if (RobotContainer.getJoy1().getRawButton(2) && overrideTimer.get() >= overrideTime) {
      override = !override;
      overrideTimer.reset();
    }

    // Logic that switches from manual to auto
    // 1000 RPM is a nice, whole number to target

    if (override) { // Auto code

      if (RobotContainer.getJoy1().getRawButton(1)) {
        setFlywheelConstantVelocity(1000.0); // Sets it to 1000 RPM
      } else {
        setFlywheelConstantVelocity(0.0);
        setFlywheelPower(0.0);
      }

      if (RobotContainer.getJoy1().getRawButton(3)) {
        setPivotAngle(90.0);
      } else if (RobotContainer.getJoy1().getRawButton(4)) {
        setPivotAngle(45.0);
      } else if (RobotContainer.getJoy1().getRawButton(11)) {
        setPivotAngle(30.0);
      }

      if (RobotContainer.getJoy1().getRawButton(5)) {
        roller.set(1.0);
      } else if (RobotContainer.getJoy1().getRawButton(6)) {
        roller.set(-1.0);
      } else {
        roller.set(0.0);
      }

      updatePivotAnglePIDF(); // Constantly holds the angle and re-calculates PIDF

  } else if (!override) { // Default manual override
      setFlywheelPower(-1.0*RobotContainer.getJoy1().getX());
      setPivotPower(-1.0*RobotContainer.getJoy1().getY());
      roller.set(0.0);
  }

    // Week 2 - set PID using Shuffleboard
    leftFlywheelPID.setPID(leftFlywheelPIDP.getDouble(Constants.leftFlywheelPIDConsts.pidP), leftFlywheelPIDI.getDouble(Constants.leftFlywheelPIDConsts.pidI), leftFlywheelPIDD.getDouble(Constants.leftFlywheelPIDConsts.pidD));
    rightFlywheelPID.setPID(rightFlywheelPIDP.getDouble(Constants.rightFlywheelPIDConsts.pidP), rightFlywheelPIDI.getDouble(Constants.rightFlywheelPIDConsts.pidI), rightFlywheelPIDD.getDouble(Constants.rightFlywheelPIDConsts.pidD));
    
    pivotPID.setPID(pivotPIDP.getDouble(Constants.pivotPIDConsts.pidP), pivotPIDI.getDouble(Constants.pivotPIDConsts.pidI), pivotPIDD.getDouble(Constants.pivotPIDConsts.pidD));
      
  }
}
