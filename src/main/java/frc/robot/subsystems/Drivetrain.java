package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

// general FRC imports

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


// SparkMax imports - these come from REV Robotics

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


// pickup constants we need for the drivetrain

import frc.robot.Constants.DrivetrainConstants;


// the first comment in the definition is picked up and displayed as information about this objecty when you
// hover over it in the editor

public class Drivetrain extends SubsystemBase {

/**
 * The Drivetrain subsystem provides all of the methods for driving the robot
 */
    private DifferentialDrive m_diffDrive;

    private CANSparkMax m_leftFrontMotor;
    private CANSparkMax m_leftRearMotor;
    private CANSparkMax m_rightFrontMotor;
    private CANSparkMax m_rightRearMotor;

    private boolean m_driveTypeErrorPrinted = false;



    public Drivetrain() {

        // initialize the controllers

        m_leftFrontMotor  = new CANSparkMax(DrivetrainConstants.kDriveLeftFront_CANID, MotorType.kBrushless);
        m_leftRearMotor   = new CANSparkMax(DrivetrainConstants.kDriveLeftRear_CANID, MotorType.kBrushless);
        m_rightFrontMotor = new CANSparkMax(DrivetrainConstants.kDriveRightFront_CANID, MotorType.kBrushless);
        m_rightRearMotor  = new CANSparkMax(DrivetrainConstants.kDriveRightRear_CANID, MotorType.kBrushless);

        // in general, it's a good idea to set the controller options to WHAT WE WANT
        //
        // like every time
        //
        // it's OK to use the various configuration tools to come up with the settings we want but one we have
        // those settings, we want make those settings are in use every time
        //
        // to get there, we'll reset the controllers to factory defaults and then explicitly set what we need

        m_leftFrontMotor.restoreFactoryDefaults();
        m_leftRearMotor.restoreFactoryDefaults();
        m_rightFrontMotor.restoreFactoryDefaults();
        m_rightRearMotor.restoreFactoryDefaults();


        // since the motors are facing in the opposite direction, one side needs to be reversed
        //
        // if we consider the "left" side as we look from the rear of the robot (like we were driving it),
        // then when the left is turning "forward", the right side needs to turn "backwards" on a differential
        // drivetrain
        //
        // we only need to set the right front motor as the rear motors will all be set to follow their
        // the front motors on their respective side

        m_rightFrontMotor.setInverted(true);


        // since we have two motors on each side, we'll make the rear motors follow the fronts
        //
        // that way we only send speed comnmands to the front motors and the rears will do the same
        //
        // this makes multiple motors useable with the differential drive class since it excpects only
        // a single motor controller for each "side"

        m_leftRearMotor.follow(m_leftFrontMotor);
        m_rightRearMotor.follow(m_rightFrontMotor);
        

        // create the differential drive using the leading motors - the fronts on each side

        m_diffDrive = new DifferentialDrive(m_leftFrontMotor, m_rightFrontMotor);


        m_leftFrontMotor.stopMotor();       // just a safety thing - they should be stopped on instantiation
        m_rightFrontMotor.stopMotor();
    }



    public void drive(double val1, double val2, int driveType) {

        switch(driveType) {
          
            case DrivetrainConstants.kDriveArcade:

                // invert the side to side joystick
                m_diffDrive.arcadeDrive(val1, -val2, DrivetrainConstants.kDriveSqInputs);
                break;

            case DrivetrainConstants.kDriveTank:

                m_diffDrive.tankDrive(val1, val2, DrivetrainConstants.kDriveSqInputs);
                break;

            case DrivetrainConstants.kDriveCurvature:

                // curvatureDrive controls the rate of turning related to the robot speed
                //
                // it does not have a square input flag but I still think it's valuable so we'll manually apply it here

                if (val1 < 0.0) {
                  val1 *= val1 * -1.0;    // maintain the sign as negative
                } else {
                  val1 *= val1;
                }

                if (val2 < 0.0) {
                  val2 *= val2 * -1.0;
                } else {
                  val2 *= val2;
                }

                // enable turn in place
                m_diffDrive.curvatureDrive(val1, val2, true);
                break;

            default:

              // only need to print the message once when we encounter this error
              //
              // in the event of an invalid drive type, we will basically ignore this call
              if (m_driveTypeErrorPrinted != true) {
                  String errMsg;
              
                  errMsg = "Drivetrain: Drive: invalid drive type " + driveType;
              
                  System.err.println(errMsg);

                  m_driveTypeErrorPrinted = true;
              }
              break;
          }
    }



  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  // be able to get the current speeds for logging and dashboard use

  public double getLeftSpeed() {

    return m_leftFrontMotor.get();
  }

  public double getRightSpeed() {

    return m_rightFrontMotor.get();
  }
}
