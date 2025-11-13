// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterState;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private static Shooter instance;

  private ShooterState currentState = ShooterState.OFF;

  public static Shooter initialize(ShooterIO shooterIO) {
    if (instance == null) {
      instance = new Shooter(shooterIO);
    }
    return instance;
  }

  public static Shooter getInstance() {
    return instance;
  }

  /** Creates a new shooter. */
  public Shooter(ShooterIO shooterIO) {
    this.shooterIO = shooterIO;
  }

  public void updateInputs() {
    shooterIO.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  @Override
  public void periodic() {
    updateInputs();
  }

  public void setState(ShooterState state) {
    currentState = state;

    setShooterMotorSpeed(state.shooterMotorSpeed);
    setFeederMotorSpeed(state.feederMotorSpeed);
  }

  public void setShooterMotorSpeed(double speed) {
    shooterIO.setShooterMotorSpeed(speed);
  }

  public void setFeederMotorSpeed(double speed) {
    shooterIO.setFeederMotorSpeed(speed);
  }

  public Command goToStateCommand(ShooterState state) {
    return new InstantCommand(() -> setState(state), this);
  }
}
