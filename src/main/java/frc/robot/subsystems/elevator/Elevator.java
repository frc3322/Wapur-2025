// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorState;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private final ElevatorIO elevatorIO;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private static Elevator instance;

  private ElevatorState elevatorState = ElevatorState.STOW;
  private boolean atGoal = false;
  private double elevatorHeight = 0;

  public static Elevator getInstance() {
    if (instance == null) {
      instance = new Elevator(new ElevatorIOReal());
    }
    return instance;
  }

  /** Creates a new Elevator. */
  public Elevator(ElevatorIO elevatorIO) {
    this.elevatorIO = elevatorIO;
  }

  public void updateInputs() {
    elevatorIO.updateInputs(inputs);
    atGoal = inputs.atGoal;

    Logger.processInputs("Elevator", inputs);

    elevatorHeight = inputs.position;
  }

  @Override
  public void periodic() {
    updateInputs();
  }

  public boolean isAtGoal() {
    return atGoal;
  }

  public double getElevatorHeightMeters() {
    return elevatorHeight;
  }

  public ElevatorState getElevatorState() {
    return elevatorState;
  }

  public void setState(ElevatorState state) {
    elevatorIO.presetSetpoint(elevatorHeight);
    this.elevatorState = state;
  }

  public Command goToStateCommand(Supplier<ElevatorState> elevatorStateSupplier) {
    return new RunCommand(
        () -> {
          ElevatorState elevatorSetpoint = elevatorStateSupplier.get();
          elevatorIO.goToPosition(
              elevatorSetpoint.elevatorSetpoint, elevatorSetpoint.elevatorVelocity);
        },
        this);
  }

  public Command setStateCommand(ElevatorState elevatorState) {
    return new InstantCommand(
        () -> {
          setState(elevatorState);
          elevatorIO.goToPosition(elevatorState.elevatorSetpoint, elevatorState.elevatorVelocity);
        },
        this);
  }

  public Command lowerElevatorCommand() {
    return new RunCommand(() -> elevatorIO.setMotorSpeeds(-0.1), this);
  }

  public Command stopElevatorCommand() {
    return new InstantCommand(() -> elevatorIO.setMotorSpeeds(0), this);
  }

  public Command zeroElevatorCommand() {
    return new InstantCommand(() -> elevatorIO.zeroEncoder(), this);
  }
}
