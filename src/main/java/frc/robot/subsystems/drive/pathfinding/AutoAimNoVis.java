package frc.robot.subsystems.drive.pathfinding;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class AutoAimNoVis extends Command {
  private final Drive drive;
  private final PIDController xPID;
  private final PIDController yPID;
  private final PIDController thetaPID;
  private Pose2d target;
  private CommandXboxController controller;

  public AutoAimNoVis(Drive drive, CommandXboxController controller, Pose2d target) {
    this.drive = drive;
    this.target = target;
    this.xPID = new PIDController(7.1, 0.0, 0.5);
    this.yPID = new PIDController(5.0, 0.0, 0.03);
    this.thetaPID = new PIDController(6.0, 0.0, 0.0);
    this.controller = controller;

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    // Reset PID controllers to prevent accumulated error
    xPID.reset();
    yPID.reset();
    thetaPID.reset();

    xPID.setTolerance(0.05);
    yPID.setTolerance(0.05);
    thetaPID.setTolerance(0.15);
  }

  @Override
  public void execute() {
    Pose2d robotPose = drive.getPose(); // Get updated robot pose

    // Cal PID outputs
    ChassisSpeeds zoom =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xPID.calculate(robotPose.getX(), target.getX()),
            yPID.calculate(robotPose.getY(), target.getY()),
            thetaPID.calculate(
                robotPose.getRotation().getRadians(), target.getRotation().getRadians()),
            drive.getRotation());

    // zoom zoom
    drive.runVelocity(ChassisSpeeds.discretize(zoom, 0.02));

    // Log data for debugging
    Logger.recordOutput("Drive/PID/Target", target);
    Logger.recordOutput("Drive/PID/RobotPose", robotPose);
    Logger.recordOutput("Drive/PID/XSetpoint", xPID.getSetpoint());
    Logger.recordOutput("Drive/PID/YSetpoint", yPID.getSetpoint());
    Logger.recordOutput("Drive/PID/ThetaSetpoint", thetaPID.getSetpoint());
  }

  @Override
  public boolean isFinished() {
    boolean setpoint = xPID.atSetpoint() && yPID.atSetpoint() && thetaPID.atSetpoint();
    boolean joystickOveride =
        Math.abs(controller.getLeftX()) > 0.5
            || Math.abs(controller.getLeftY()) > 0.5
            || Math.abs(controller.getRightX()) > 0.5;
    return setpoint || joystickOveride;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stopWithX();
  }
}
