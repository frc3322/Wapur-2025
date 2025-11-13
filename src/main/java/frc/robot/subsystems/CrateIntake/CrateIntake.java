// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.crateIntake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.crateIntake.CrateIntakeConstants.CrateIntakeState;
import org.littletonrobotics.junction.Logger;

public class CrateIntake extends SubsystemBase {

  private final CrateIntakeIO crateIntakeIO;
  private final CrateIntakeIOInputsAutoLogged inputs = new CrateIntakeIOInputsAutoLogged();
  private static CrateIntake instance;

  public static CrateIntake getInstance() {
    if (instance == null) {
      instance = new CrateIntake(new CrateIntakeIOReal());
    }
    return instance;
  }

  /** Creates a new CrateIntake. */
  public CrateIntake(CrateIntakeIO crateIntakeIO) {
    this.crateIntakeIO = crateIntakeIO;
  }

  public void updateInputs() {
    crateIntakeIO.updateInputs(inputs);
    Logger.processInputs("CrateIntake", inputs);
  }

  @Override
  public void periodic() {
    updateInputs();
  }

  public void setState(CrateIntakeState state) {
    setMotorSpeed(state.motorSpeed);
  }

  public void setMotorSpeed(double speed) {
    crateIntakeIO.setMotorSpeed(speed);
  }

  public Command goToStateCommand(CrateIntakeState state) {
    return new InstantCommand(() -> setState(state), this);
  }
}
