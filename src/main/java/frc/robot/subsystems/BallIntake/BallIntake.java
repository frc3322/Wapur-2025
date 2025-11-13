// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ballIntake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ballIntake.BallIntakeConstants.BallIntakeState;
import org.littletonrobotics.junction.Logger;

public class BallIntake extends SubsystemBase {

  private final BallIntakeIO ballIntakeIO;
  private final BallIntakeIOInputsAutoLogged inputs = new BallIntakeIOInputsAutoLogged();
  private static BallIntake instance;

  public static BallIntake getInstance() {
    if (instance == null) {
      instance = new BallIntake(new BallIntakeIOReal());
    }
    return instance;
  }

  /** Creates a new BallIntake. */
  public BallIntake(BallIntakeIO ballIntakeIO) {
    this.ballIntakeIO = ballIntakeIO;
  }

  public void updateInputs() {
    ballIntakeIO.updateInputs(inputs);
    Logger.processInputs("BallIntake", inputs);
  }

  @Override
  public void periodic() {
    updateInputs();
  }

  public void setState(BallIntakeState state) {
    setMotorSpeed(state.motorSpeed);
  }

  public void setMotorSpeed(double speed) {
    ballIntakeIO.setMotorSpeed(speed);
  }

  public Command goToStateCommand(BallIntakeState state) {
    return new InstantCommand(() -> setState(state), this);
  }
}
