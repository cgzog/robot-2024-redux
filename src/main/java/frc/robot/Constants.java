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

    public static final Measure<Mass> kRobotMass = Pounds.of(100.0);

    public static final double kRobotInertia = 7.0;   // not measured for this robot - typical 3-8 jKgM^2 range
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

    public static final int kLeftEncoderADio  = 0;      // DIO channels on RoboRIO (should be able to do this using SparkMAX encoder but having simulation trouble)
    public static final int kLeftEncoderBDio  = 1;
    public static final int kRightEncoderADio = 2;
    public static final int kRightEncoderBDio = 3;

    public static final int kDriveArcade       = 0;
    public static final int kDriveTank         = 1;
    public static final int kDriveCurvature    = 2;

    public static final int kDriveType         = kDriveArcade;

    public static final boolean kDriveSqInputs = true;

    // physical drivetrain constants

    public static final Measure<Distance> kDrivetrainTrack         = Inches.of(29);   // close to the width of the robot - using a typical 30" width
    public static final Measure<Distance> kDrivetrainWheelbase     = Inches.of(24);   // wheel base tends to be smaller than track
    public static final Measure<Distance> kDrivetrainWheelDiameter = Inches.of(6);    // 6in wheel diameter
    public static final double kDrivetrainGearRatio                = 10.71;                     // typical kitbot gear ratio

    public static final int    kDrivetrainPulsesPerWheelRotation = 4096; // typical quadrature encoder - used for simulation
  }

  public static class IntakeConstants {

  }

  public static class ShooterConstants {
 
  }

}
