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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveConstants {
    // if the motor will be controlled via a CANbus connected controller, we need to know the
    // CANbus ID to use for the various constructors
    //
    // IMPORTANT:  This not only needs to be set in the code but the actual motor controller
    // will need to be configured to the correct CANbus address so it will respond to the code
    // as expected - this is done outsiode of the robot code and is typically done by a
    // vendor supplied tool specific to the brand of controller being used (REV Hardware Client
    // for REV Robotics SparkMax controllers for example)
    
    public static final int kDriveLeftFront_CANID  = 10;
    public static final int kDriveLeftRear_CANID   = 11;
    public static final int kDriveRightFront_CANID = 12;
    public static final int kDriveRightRear_CANID  = 13;
  }

  public static class IntakeConstants {

  }

  public static class ShooterConstants {
 
  }

}
