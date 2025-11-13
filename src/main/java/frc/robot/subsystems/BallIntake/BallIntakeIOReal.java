package frc.robot.subsystems.ballIntake;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public class BallIntakeIOReal implements BallIntakeIO {
  private final SparkFlex motor;

  public BallIntakeIOReal() {
    motor = new SparkFlex(BallIntakeConstants.motorCanId, MotorType.kBrushless);

    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kCoast).smartCurrentLimit(BallIntakeConstants.motorCurrentLimit);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(BallIntakeIOInputs inputs) {
    inputs.motorSpeed = motor.getEncoder().getVelocity();
  }

  @Override
  public void setMotorSpeed(double speed) {
    motor.set(speed);
  }
}
