package frc.robot.subsystems.dropper;

public class DropperConstants {
    public static final int droppercanID = 99;

    public enum DropperState {
        OUTTAKE(-1),
        STOP(0.0);

        public final double motorSpeed;

        DropperState(double motorSpeed) {
            this.motorSpeed = motorSpeed;
        }
    }
}