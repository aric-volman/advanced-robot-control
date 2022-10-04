// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// How to calculate kF:
// (Percent Motor Output% X 1023) / Native velocity
// This formula aligns with Team 6377's whitepaper on Motion Magic
// https://howdybots.org/wp-content/uploads/2019/12/Dont_Break_Your_Bot_Whitepaper-V1.pdf

// CTRE Docs on use of IntegralZone:
// Hint: it resets the Integral after a certain value
// https://docs.ctre-phoenix.com/en/latest/ch16_ClosedLoop.html?highlight=Zone#closed-loop-configs-per-slot-four-slots-available

// WARNING - Still in testing phase!

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class magicpivot extends SubsystemBase {
  /** Creates a new magicpivot. */
  private boolean override = false; // Helps us switch from manual to auto
  private Timer overrideTimer = new Timer(); // We want a toggle button of some sorts
  private double overrideTime = 1.0;

  // private double ticks = 90.0; // Angle at which to start at - 90 above the horizontal
  private double encoderConst = 1.0; // Flips the sign of the angle if needed
  // private double pivotTolerance = 1.0; // PID tolerance
  // This is a positive offset that we need to add
  // meaning that the angle returned from the pivot is in fact negative
  private double offset = Constants.angularOffset;

  // All in ticks -- to do later
  private double targetPositionAtRest = 0.0;
  private double targetPostionALittleAway = 0.0;
  private double targetPositionNear45 = 0.0;

  private double targetPosition = targetPositionAtRest;

  private final WPI_TalonSRX pivot = new WPI_TalonSRX(Constants.ShooterPorts.pivotPort);

  public magicpivot() {

    pivot.configFactoryDefault();
    pivot.setInverted(false);
    pivot.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    pivot.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    pivot.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    pivot.configPeakCurrentLimit(20, 5);

    pivot.config_kP(0, Constants.pivotMMconsts.kP, Constants.pivotMMconsts.kTimeoutMs);
    pivot.config_kI(0, Constants.pivotMMconsts.kI, Constants.pivotMMconsts.kTimeoutMs);
    pivot.config_kD(0, Constants.pivotMMconsts.kD, Constants.pivotMMconsts.kTimeoutMs);
    pivot.config_kF(0, Constants.pivotMMconsts.kF, Constants.pivotMMconsts.kTimeoutMs);
    pivot.config_IntegralZone(0, Constants.pivotMMconsts.kIZone, Constants.pivotMMconsts.kTimeoutMs);
    pivot.configClosedLoopPeakOutput(0, Constants.pivotMMconsts.kPeakOutput, Constants.pivotMMconsts.kTimeoutMs);

    pivot.configMotionAcceleration(Constants.pivotMMconsts.kAcceleration, Constants.pivotMMconsts.kTimeoutMs);
    pivot.configMotionCruiseVelocity(Constants.pivotMMconsts.kVelocity, Constants.pivotMMconsts.kTimeoutMs);
  
  }


  public void resetPivotEncoders() {
    pivot.setSelectedSensorPosition(0, 0, 10);
  }

  /* public void setPivotAngle(double ticks) {
    this.ticks = ticks;
  } */


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

  public double getPivotVelocity() {
    return encoderConst*pivot.getSelectedSensorVelocity(0);
  }

  public double getPivotPosition(){
    return (((encoderConst*pivot.getSelectedSensorPosition(0)/4096.0)*360.0) + offset); // OFFSETTED is angle here. The negative const might not be needed
  }

  public double getFF() {
    // Has to be offsetted in order to fit the range of cos
    double angleInRadians = Math.toRadians(getPivotPosition());
    double calculatedFF = Constants.maxHorizontalVoltage*Math.cos(angleInRadians);
    return calculatedFF;
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
    // pivot.set(ControlMode.MotionMagic, targetPositionInTicks, DemandType.ArbitraryFeedForward, getFF());
    // Ideas: tunable on the fly constants?
    SmartDashboard.putNumber("Pivot Angle", getPivotPosition());
    SmartDashboard.putNumber("Pivot Angle with No Offset", getPivotPositionNotOffset()); // Use this to find the offset angle at horizontal power
    SmartDashboard.putNumber("Raw Pivot Angle with No Offset", getRawPivotPositionNotOffset()); 
    SmartDashboard.putNumber("Pivot Power", getPivotPower());

    SmartDashboard.putNumber("Pivot Velocity", getPivotVelocity());


    if (RobotContainer.getJoy1().getRawButton(2) && overrideTimer.get() >= overrideTime) {
      override = !override;
      overrideTimer.reset();
    }

    if (!override) {
      setPivotPower(-1.0*RobotContainer.getJoy1().getY());
    } /* else if (override) {
      pivot.set(ControlMode.MotionMagic, targetPosition, DemandType.ArbitraryFeedForward, getFF());
      if (RobotContainer.getJoy1().getRawButton(3)) {
        this.targetPosition = targetPositionAtRest;
      } else if (RobotContainer.getJoy1().getRawButton(4)) {
        this.targetPosition = targetPostionALittleAway;
      } else if (RobotContainer.getJoy1().getRawButton(11)) {
        this.targetPosition = targetPositionNear45;
      }
    }*/ else {
      pivot.set(0.0); // For safe manual testing
    }

  }
}
