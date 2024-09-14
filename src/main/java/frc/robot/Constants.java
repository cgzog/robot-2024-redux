// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.*;
import static edu.wpi.first.units.Units.*;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class RobotConstants {

    public static final double kRobotMassKg  = 100.0 / 2.2;  // 100# robot

    public static final double kRobotInertia = 5.5;   // not measured for this robot - right in the middle of the typical 3-8 jKgM^2 range
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DrivetrainConstants {

    public static final int kNumOfMotorsPerSide = 2;  // 4 motor mirrors drivetrain

    // if the motor will be controlled via a CANbus connected controller, we need to know the
    // CANbus ID to use for the various constructors
    //
    // IMPORTANT:  This not only needs to be set in the code but the actual motor controller
    // will need to be configured to the correct CANbus address so it will respond to the code
    // as expected - this is done outside of the robot code and is typically done by a
    // vendor supplied tool specific to the brand of controller being used (REV Hardware Client
    // for REV Robotics SparkMax controllers for example)
    
    public static final int kDriveLeftFront_CANID  = 10;
    public static final int kDriveLeftRear_CANID   = 11;
    public static final int kDriveRightFront_CANID = 12;
    public static final int kDriveRightRear_CANID  = 13;

    public static final int kDriveArcade       = 0;
    public static final int kDriveTank         = 1;
    public static final int kDriveCurvature    = 2;

    public static final int kDriveType         = kDriveArcade;

    public static final boolean kDriveSqInputs = true;

    // physical drivetrain constants

    public static final double kDrivetrainTrackMeters       = 0.75;     // close to the width of the robot - just under 30" here
    public static final double kDrivetrainWheelbaseMeters   = 0.60;     // right around 24" - wheel base tends to be smaller than track
    public static final double kDrivetrainWheelRadiusMeters = 0.0762;   // 6in wheel diameter - this is precise
    public static final double kDrivetrainGearRatio         = 9.13;     // middle of the road Toughbox Mini S gearbox ratio
  }

  public static class IntakeConstants {

  }

  public static class ShooterConstants {
 
  }

}
