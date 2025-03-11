package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vision.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.VisionSubsystem;

class PIDControllerConfigurable extends PIDController {
  public PIDControllerConfigurable(double kP, double kI, double kD) {
    super(kP, kI, kD);
  }

  public PIDControllerConfigurable(double kP, double kI, double kD, double tolerance) {
    super(kP, kI, kD);
    this.setTolerance(tolerance);
  }
}

public class AlignCommand extends Command {

  private final VisionSubsystem m_limelight;
  private int m_tagId;

  public double rotationalRate = 0;
  public double velocityX = 0;

  public AlignCommand(VisionSubsystem limelight, int tagId) {
    this.m_limelight = limelight;
    this.m_tagId = tagId;
    addRequirements(m_limelight);
  }

  @Override
  public void initialize() {
    System.out.println(getName() + " AlignCommand");
  }

  @Override
  public void execute() {

    RawFiducial fiducial;
    SmartDashboard.putNumber("TagID", m_tagId);
    try {
      if (m_tagId == 0) {
        fiducial = m_limelight.getClosestFiducial();
      } else {
        fiducial = m_limelight.getFiducialWithId(m_tagId);
      }

      SmartDashboard.putNumber("txnc", fiducial.txnc);
      SmartDashboard.putNumber("distToRobot", fiducial.distToRobot);
      SmartDashboard.putNumber("rotationalPidController", rotationalRate);
      SmartDashboard.putNumber("xPidController", velocityX);

      System.out.println(getName() + " AlignCommand" + String.valueOf(fiducial.distToRobot));

    } catch (VisionSubsystem.NoSuchTargetException nste) {
      System.out.println("No apriltag found");
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
