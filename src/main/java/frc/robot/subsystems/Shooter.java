package frc.robot.subsystems;

import edu.wpi.first.math.*;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.units.*;
import static edu.wpi.first.units.Units.*;

import javax.naming.directory.InvalidSearchFilterException;

// simulation-related

import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



// SparkMax imports - these come from REV Robotics

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;


// our robot constants

import frc.robot.Robot;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.ShooterConstants;



// the first comment in the definition is picked up and displayed as information about this objecty when you
// hover over it in the editor

public class Shooter extends SubsystemBase {

/**
 * The Shooter subsystem provides all of the methods for shooting the note one loaded
 */
    
    private final CANSparkMax m_shooterMotor = new CANSparkMax(ShooterConstants.kShooterMotor_CANID, MotorType.kBrushless);
    private final CANSparkMax m_kickerMotor  = new CANSparkMax(ShooterConstants.kKickerMotor_CANID,  MotorType.kBrushless);

    private SparkPIDController m_shooterPidController;
    private double             m_pidP;
    private double             m_pidI;
    private double             m_pidD;
    private double             m_pidIzone;
    private double             m_pidFF;
    private double             m_pidOutputMax;
    private double             m_pidOutputMin;
    private double             m_pidSetPoint = 0;
    

    private RelativeEncoder m_shooterEncoder;



    public Shooter() {

        m_shooterMotor.restoreFactoryDefaults();
        m_kickerMotor.restoreFactoryDefaults();

        m_shooterMotor.stopMotor();                     // just a safety thing - they should be stopped on instantiation
        m_kickerMotor.stopMotor();

        m_shooterPidController = m_shooterMotor.getPIDController();

        m_shooterPidController.setP(ShooterConstants.kPidP);
        m_shooterPidController.setI(ShooterConstants.kPidI);
        m_shooterPidController.setD(ShooterConstants.kPidD);
        m_shooterPidController.setIZone(ShooterConstants.kPidIzone);
        m_shooterPidController.setFF(ShooterConstants.kPidFF);
        m_shooterPidController.setOutputRange(ShooterConstants.kPidOutputMin, ShooterConstants.kPidOutputMax);

        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("Shooter Set Point", m_pidSetPoint);
        SmartDashboard.putNumber("Shooter P Gain", ShooterConstants.kPidP);
        SmartDashboard.putNumber("Shooter I Gain", ShooterConstants.kPidI);
        SmartDashboard.putNumber("Shooter D Gain", ShooterConstants.kPidD);
        SmartDashboard.putNumber("Shooter I Zone", ShooterConstants.kPidIzone);
        SmartDashboard.putNumber("Shooter Feed Forward", ShooterConstants.kPidFF);
        SmartDashboard.putNumber("Shooter Max Output", ShooterConstants.kPidOutputMax);
        SmartDashboard.putNumber("Shooter Min Output", ShooterConstants.kPidOutputMin);

        m_shooterEncoder = m_shooterMotor.getEncoder();

        if ( ! Robot.isReal()) {                        // setup things for the simulation as needed
        
        }
    }



    // set the shooter speed in RPM
    //
    // positive RPM means create a PID controller on the SparkMAX to set and maintain the target speed
    //
    // 0 means stop the shooter motor

    public void setShooterSpeed(double speed) {

     if (speed == 0) {
        m_shooterMotor.stopMotor();
        return;
     }

     // if we're not stopping, we're going so set the target speed

    }



    public double getShooterSpeed() {

      return m_shooterEncoder.getVelocity();      // return the RPM by default
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

    // read PID coefficients from SmartDashboard
    m_pidSetPoint  = SmartDashboard.getNumber("Shooter Set Point", 0);

    double p  = SmartDashboard.getNumber("Shooter P Gain", 0);
    double i  = SmartDashboard.getNumber("Shooter I Gain", 0);
    double d  = SmartDashboard.getNumber("Shooter D Gain", 0);
    double iz  = SmartDashboard.getNumber("Shooter I Zone", 0);
    double ff  = SmartDashboard.getNumber("Shooter Feed Forward", 0);
    double max = SmartDashboard.getNumber("Shooter Max Output", 0);
    double min = SmartDashboard.getNumber("Shooter Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != m_pidP)) { m_shooterPidController.setP(p); m_pidP = p; }
    if((i != m_pidIzone)) { m_shooterPidController.setI(i); m_pidI = i; }
    if((d != m_pidD)) { m_shooterPidController.setD(d); m_pidD = d; }
    if((iz != m_pidIzone)) { m_shooterPidController.setIZone(iz); m_pidIzone = iz; }
    if((ff != m_pidFF)) { m_shooterPidController.setFF(ff); m_pidFF = ff; }
    if((max != m_pidOutputMax) || (min != m_pidOutputMin)) { 
      m_shooterPidController.setOutputRange(min, max); 
      m_pidOutputMin = min;
      m_pidOutputMax = max; 
    }

    SmartDashboard.putNumber("Shooter Target RPM", m_pidSetPoint);
    SmartDashboard.putNumber("Shooter Actual RPM", getShooterSpeed());

  }

  @Override
  public void simulationPeriodic() {

  }
}