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

/**
 * Elevator subsystem controlling vertical positioning with PID and feedforward
 * control.
 */
public class Elevator extends SubsystemBase {

    private final ElevatorIO elevatorIO;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private static Elevator instance;

    private ElevatorState elevatorState = ElevatorState.LEVEL0;
    private boolean atGoal = false;
    private double elevatorHeight = 0;

    /** Gets singleton instance of the elevator subsystem. */
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

    /** Returns true if elevator is at its target position. */
    public boolean isAtGoal() {
        return atGoal;
    }

    /** Gets current elevator height in meters. */
    public double getElevatorHeightMeters() {
        return elevatorHeight;
    }

    /** Gets current elevator state (INTAKE, LEVEL0, etc.). */
    public ElevatorState getElevatorState() {
        return elevatorState;
    }

    /** Sets elevator to a predefined state without moving. */
    public void setState(ElevatorState state) {
        elevatorIO.presetSetpoint(state.elevatorSetpoint);
        this.elevatorState = state;
    }

    /** Creates command to continuously move to dynamically supplied state. */
    public Command goToStateCommand(Supplier<ElevatorState> elevatorStateSupplier) {
        return new RunCommand(
                () -> {
                    ElevatorState elevatorSetpoint = elevatorStateSupplier.get();
                    elevatorIO.goToPosition(elevatorSetpoint.elevatorSetpoint);
                },
                this);
    }

    /** Creates command to set state and move to position immediately. */
    public Command setStateCommand(ElevatorState elevatorState) {
        return new InstantCommand(
                () -> {
                    setState(elevatorState);
                    elevatorIO.goToPosition(elevatorState.elevatorSetpoint);
                },
                this);
    }

    /** Creates command to slowly lower elevator for manual control. */
    public Command lowerElevatorCommand() {
        return new RunCommand(() -> elevatorIO.setMotorSpeeds(-0.1), this);
    }

    /** Creates command to stop elevator motion. */
    public Command stopElevatorCommand() {
        return new InstantCommand(() -> elevatorIO.setMotorSpeeds(0), this);
    }

    /** Creates command to zero the elevator encoder. */
    public Command zeroElevatorCommand() {
        return new InstantCommand(() -> elevatorIO.zeroEncoder(), this);
    }
}
