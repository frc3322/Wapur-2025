package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ShooterIOReal implements ShooterIO {
  private final SparkMax shooterMotor;
  private final SparkMax feederMotor;

  public ShooterIOReal() {
    shooterMotor = new SparkMax(ShooterConstants.shooterMotorCanId, MotorType.kBrushless);
    feederMotor = new SparkMax(ShooterConstants.feederMotorCanId, MotorType.kBrushless);

    SparkMaxConfig shooterConfig = new SparkMaxConfig();
    shooterConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ShooterConstants.shooterMotorCurrentLimit);

    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ShooterConstants.feederMotorCurrentLimit);

    shooterMotor.configure(
        shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    feederMotor.configure(
        feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.shooterMotorSpeed = shooterMotor.getEncoder().getVelocity();
    inputs.feederMotorSpeed = feederMotor.getEncoder().getVelocity();
  }

  @Override
  public void setShooterMotorSpeed(double speed) {
    shooterMotor.set(speed);
  }

  @Override
  public void setFeederMotorSpeed(double speed) {
    feederMotor.set(speed);
  }
}
