package frc.robot.subsystems.CrateIntake;

public class CrateIntakeConstants {
    public static final int leftIntakeCanId = 6;
    public static final int rightIntakeCanId = 7;
    public static final int motorLimit = 40;

    public enum CrateIntakeState {
        INTAKE(1.0),
        OUTTAKE(-1.0),
        STOP(0.0);

    
    

        public final double motorSpeed;

        CrateIntakeState(double motorSpeed) {
            this.motorSpeed = motorSpeed;
        }
    }
}
    