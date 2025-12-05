package frc.robot.subsystems.crateIntake;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

/** Real hardware implementation for the crate intake using REV SparkMax motors. */
public class CrateIntakeIOReal implements CrateIntakeIO {
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;

  /** Initializes the crate intake motors with proper configuration. */
  public CrateIntakeIOReal() {
    leftMotor = new SparkMax(CrateIntakeConstants.leftMotorCanId, MotorType.kBrushless);
    rightMotor = new SparkMax(CrateIntakeConstants.rightMotorCanId, MotorType.kBrushless);

    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(CrateIntakeConstants.motorCurrentLimit);

    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(CrateIntakeConstants.motorCurrentLimit);

    leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(CrateIntakeIOInputs inputs) {
    inputs.leftMotorSpeed = leftMotor.getEncoder().getVelocity();
    inputs.rightMotorSpeed = rightMotor.getEncoder().getVelocity();
  }

  /** Sets motor speeds - left motor runs in reverse for proper intake direction. */
  @Override
  public void setMotorSpeed(double speed) {
    rightMotor.set(speed);
    leftMotor.set(-speed);
  }
}
