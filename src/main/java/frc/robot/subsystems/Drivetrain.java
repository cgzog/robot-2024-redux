package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.units.*;
import static edu.wpi.first.units.Units.*;


// simulation-related

import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
//import edu.wpi.first.wpilibj.simulation.GyroSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N7;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


// SparkMax imports - these come from REV Robotics

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;


// our robot constants

import frc.robot.Robot;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.DrivetrainConstants;



// the first comment in the definition is picked up and displayed as information about this objecty when you
// hover over it in the editor

public class Drivetrain extends SubsystemBase {

/**
 * The Drivetrain subsystem provides all of the methods for driving the robot
 */
    

    private final CANSparkMax m_leftFrontMotor  = new CANSparkMax(DrivetrainConstants.kDriveLeftFront_CANID,  MotorType.kBrushless);
    private final CANSparkMax m_leftRearMotor   = new CANSparkMax(DrivetrainConstants.kDriveLeftRear_CANID,   MotorType.kBrushless);
    private final CANSparkMax m_rightFrontMotor = new CANSparkMax(DrivetrainConstants.kDriveRightFront_CANID, MotorType.kBrushless);
    private final CANSparkMax m_rightRearMotor  = new CANSparkMax(DrivetrainConstants.kDriveRightRear_CANID, MotorType.kBrushless);

    private final RelativeEncoder m_leftEncoder  = m_leftFrontMotor.getEncoder();
    private final RelativeEncoder m_rightEncoder = m_leftFrontMotor.getEncoder();

    private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftFrontMotor, m_rightFrontMotor);    // setup following later

    /*
    // for now, we'll just make everything close to "perfect" - would need to measure this in real life using 'sysinfo'

    private final Matrix<N7, N1> m_measurementStdDevs = VecBuilder.fill(0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01);

    private final DifferentialDrivetrainSim m_diffDriveSim = DifferentialDrivetrainSim(DCMotor.getNEO(DrivetrainConstants.kNumOfMotorsPerSide),
                                                                                       DrivetrainConstants.kDrivetrainGearRatio,
                                                                                       RobotConstants.kRobotInertia,
                                                                                       RobotConstants.kRobotMass.in(Kilograms),
                                                                                       DrivetrainConstants.kDrivetrainWheelDiameter.in(Meters) / 2,
                                                                                       DrivetrainConstants.kDrivetrainTrack.in(Meters),
                                                                                       m_measurementStdDevs); // measurement std devs;
    */

    private final DifferentialDrivetrainSim m_diffDriveSim = DifferentialDrivetrainSim.createKitbotSim(KitbotMotor.kDoubleNEOPerSide,
                                                                                                       DifferentialDrivetrainSim.KitbotGearing.k10p71,
                                                                                                       DifferentialDrivetrainSim.KitbotWheelSize.kSixInch,
                                                                                                       null);

    // dummy encoders to instantiate the simulated encoders - can't seem to do that from the SparkMax built-in encoder

    private final Encoder m_leftDummyEncoder  = new Encoder(DrivetrainConstants.kLeftEncoderADio,  DrivetrainConstants.kLeftEncoderBDio);
    private final Encoder m_rightDummyEncoder = new Encoder(DrivetrainConstants.kRightEncoderADio, DrivetrainConstants.kRightEncoderBDio);

    private final EncoderSim m_leftEncoderSim  = new EncoderSim(m_leftDummyEncoder);
    private final EncoderSim m_rightEncoderSim = new EncoderSim(m_rightDummyEncoder);

    private boolean m_driveTypeErrorPrinted = false;



    public Drivetrain() {

        // in general, it's a good idea to set the controller options to WHAT WE WANT
        //
        // like every time
        //
        // it's OK to use the various configuration tools to come up with the settings we want but once we have
        // those settings, we want make sure those settings are in use every time
        //
        // to get there, we'll reset the controllers to factory defaults and then explicitly set what we need

        m_leftFrontMotor.restoreFactoryDefaults();
        m_leftRearMotor.restoreFactoryDefaults();
        m_rightFrontMotor.restoreFactoryDefaults();
        m_rightRearMotor.restoreFactoryDefaults();

        m_leftEncoder.setPosition(0.0);   // make sure the encoders are zeroed out to start (they should be)
        m_rightEncoder.setPosition(0.0);


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
        
        m_leftFrontMotor.stopMotor();       // just a safety thing - they should be stopped on instantiation
        m_rightFrontMotor.stopMotor();

        if ( ! Robot.isReal()) {            // setup things for the simulation as needed
        
          // we'll just tell it we have a specific pulse count per rev of the wheel and calculate a distance for each pulse
          // calculate the wheel circuference and divide but the number of pulses per rotation

          m_leftEncoderSim.setDistancePerPulse(DrivetrainConstants.kDrivetrainWheelDiameter.in(Meters) * 3.1415 / DrivetrainConstants.kDrivetrainPulsesPerWheelRotation);
          m_rightEncoderSim.setDistancePerPulse(m_leftEncoderSim.getDistancePerPulse());    // get what we set; they always should be the same

          m_leftEncoderSim.setCount(0);
          m_rightEncoderSim.setCount(0);
        }
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
