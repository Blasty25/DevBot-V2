package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private SparkMax feeder;
  private SparkMax shooter;

  private SparkMaxConfig config = new SparkMaxConfig();

  public Shooter(int feederID, int shooterID) {
    feeder = new SparkMax(feederID, MotorType.kBrushless);
    shooter = new SparkMax(shooterID, MotorType.kBrushless);

    config.idleMode(IdleMode.kCoast);
    config.inverted(true);
    config.voltageCompensation(12.0);
    config.smartCurrentLimit(80);

    feeder.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shooter.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Shooter/ShooterVolts", shooter.getAppliedOutput());
    Logger.recordOutput("Shooter/FeederVolts", feeder.getAppliedOutput());
  }

  public void runShooter(double set) {
    shooter.set(set);
  }

  public void runFeeder(double set) {
    feeder.set(set);
  }

  public void stop() {
    feeder.set(0);
    shooter.set(0);
  }
}
