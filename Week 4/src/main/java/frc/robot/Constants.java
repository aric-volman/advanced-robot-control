// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class ShooterPorts {
        public static final int LeftFlywheelPort = 16;
        public static final int RightFlywheelPort = 17;
        public static final int pivotPort = 14;
        public static final int rollerPort = 15;
    }

    public static final class USBOrder {
        public static final int Zero = 0;
        public static final int One = 1;
    }
    
    public static final class pivotMMconsts {
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kF = 0.0;
        public static final double kIZone = 0.0;
        public static final int kTimeoutMs = 10;
        public static final double kPeakOutput = 0.8;
        public static final double kVelocity = 0.0;
        public static final double kAcceleration = 0.0;
    }

    // In degrees, added, not subtracted
    public static final double angularOffset = 90.0;
    public static final double maxHorizontalVoltage = 0.2;
}
