package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;


public class ShooterCommand extends Command {
  private final DriveTrain m_driveTrain;

  public DriveForward(DriveTrain driveTrain) {

  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    //Code to drive forward
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}