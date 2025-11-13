package frc.robot.subsystems.elevator;

public class Elevator {
    private final ElevatorIO elevatorIO; // Elevator Object
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    
    private static Elevator instance;

    private boolean goalMet = false;
    private double elevatorHeight = 0; 
    
    // Sets Elevator
    public static Elevator initialize(ElevatorIO elevatorIO) { 
        if (instance == null) { // If no instance MAKE ONE
            instance = new Elevator(elevatorIO); // Sets instance 
        }
        return instance;
    } 

    // Get Instance
    public static Elevator getInstance() {
        return instance;
    }

    public Elevator(ElevatorIO elevatorIO) {
        this.elevatorIO = elevatorIO;
    }

    
}
