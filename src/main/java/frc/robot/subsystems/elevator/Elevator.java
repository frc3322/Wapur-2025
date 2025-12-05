package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
    private final ElevatorIO elevatorIO;

    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private static Elevator instance;

    private ElevatorStates elevatorState = ElevatorStates.STOW;

    private boolean atGoal = false;
    private double elevatorHeight = 0;

    // Sets Elevator
    public static Elevator initialize(ElevatorIO elevatorIO) {
        if (instance == null) { // If there is no instance then
            instance = new Elevator(elevatorIO); // Sets the new instance
        }
        return instance; // Returns our object instance
    }

    // Get Instance
    public static Elevator getInstance() {
        return instance;
    }

    public Elevator(ElevatorIO elevatorIO) { // / The constructor for the class
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

    // Some getters that return values
    public boolean isHighEnough() {
        return elevatorHeight > 1.5;
    }

    public boolean isAtGoal() {
        return atGoal;
    }

    public double getElevatorHeightInMeters() {
        return elevatorHeight;
    }

    public ElevatorStates getElevatorstate() {
        return elevatorState;
    }

    // Setters
    public void setElevatorState(ElevatorStates elevatorStates) {
        elevatorIO.presetSetpoint(elevatorHeight);
        this.elevatorState = elevatorStates;
    }

    public Command goToStateCommand(Supplier<ElevatorStates> elevatorStateSupplier) {
        return new RunCommand(
                () -> {
                    ElevatorStates elevatorSetpoint = elevatorStateSupplier.get();
                    elevatorIO.goToPosition(elevatorSetpoint.elevatorSetpoint);
                },
                this);
    }

    public Command setStateCommand(ElevatorStates elevatorState) {
        return new InstantCommand(
                () -> {
                    setElevatorState(elevatorState);
                    elevatorIO.goToPosition(elevatorState.elevatorSetpoint);
                },
                this);
    }

    public Command lowerElevatorCommand() {
        return new RunCommand(() -> elevatorIO.setMotorSpeeds(-.1), this);
    }

    public Command stopElevatorCommand() {
        return new InstantCommand(() -> elevatorIO.setMotorSpeeds(0), this);
    }

    public Command zeroElevatorCommand() {
        return new InstantCommand(() -> elevatorIO.zeroEncoder(), this);
    }
}
