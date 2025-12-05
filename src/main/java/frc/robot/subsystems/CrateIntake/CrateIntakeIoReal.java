package frc.robot.subsystems.CrateIntake;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class CrateIntakeIOReal implements CrateIntakeIO {
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;

  public CrateIntakeIOReal() {
    leftMotor = new SparkMax(CrateIntakeConstants.leftIntakeCanId, MotorType.kBrushless);
    rightMotor = new SparkMax(CrateIntakeConstants.rightIntakeCanId, MotorType.kBrushless);

    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(CrateIntakeConstants.motorLimit);

    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(CrateIntakeConstants.motorLimit);

    leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(CrateIntakeIOInputs inputs) {
    inputs.leftMotorSpeed = leftMotor.getEncoder().getVelocity();
    inputs.rightMotorSpeed = rightMotor.getEncoder().getVelocity();
  }

  @Override
  public void setMotorspeed(double speed) {
    rightMotor.set(speed);
    leftMotor.set(speed);
  }
}
