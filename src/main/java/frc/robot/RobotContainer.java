// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.CrateIntake.CrateIntake;
import frc.robot.subsystems.CrateIntake.CrateIntakeConstants.CrateIntakeState;
import frc.robot.subsystems.CrateIntake.CrateIntakeIOReal;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.dropper.Dropper;
import frc.robot.subsystems.dropper.DropperConstants.DropperState;
import frc.robot.subsystems.dropper.DropperIOReal;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Elevator elevator;
  private final CrateIntake crateIntake;
  private final Dropper dropper;

  // Controller
  private final CommandXboxController primaryController = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // Modules 0 and 2 use SparkFlex (NeoVortex), modules 1 and 3 use SparkMax (NEO)
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0, false), // Front Left - SparkFlex
                new ModuleIOSpark(1, true), // Front Right - SparkMax (NEO)
                new ModuleIOSpark(2, false), // Back Left - SparkFlex
                new ModuleIOSpark(3, true)); // Back Right - SparkMax (NEO)

        elevator = Elevator.initialize(new ElevatorIOReal());
        crateIntake = CrateIntake.initialize(new CrateIntakeIOReal());
        dropper = Dropper.initialize(new DropperIOReal());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());

        elevator = Elevator.initialize(new ElevatorIOReal());
        crateIntake = CrateIntake.initialize(new CrateIntakeIOReal());
        dropper = Dropper.initialize(new DropperIOReal());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        elevator = Elevator.initialize(new ElevatorIOReal());
        crateIntake = CrateIntake.initialize(new CrateIntakeIOReal());
        dropper = Dropper.initialize(new DropperIOReal());

        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Simple forward auton
    autoChooser.addOption(
        "Drive Forward",
        Commands.run(() -> drive.runVelocity(new ChassisSpeeds(1.0, 0.0, 0.0)), drive)
            .withTimeout(2.0));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -primaryController.getLeftY(),
            () -> -primaryController.getLeftX(),
            () -> -primaryController.getRightX()));

    // Lock to 0° when A button is held
    primaryController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -primaryController.getLeftY(),
                () -> -primaryController.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    primaryController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    primaryController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    primaryController.a().onTrue(elevator.setStateCommand(ElevatorStates.L1));
    primaryController.x().onTrue(elevator.setStateCommand(ElevatorStates.L2));
    primaryController.b().onTrue(elevator.setStateCommand(ElevatorStates.L3));
    primaryController.y().onTrue(elevator.setStateCommand(ElevatorStates.L4));

    primaryController
        .leftTrigger()
        .whileTrue(crateIntake.setCrateIntakeStateCommand(CrateIntakeState.OUTTAKE))
        .onFalse(crateIntake.setCrateIntakeStateCommand(CrateIntakeState.STOP));
    primaryController
        .rightTrigger()
        .whileTrue(crateIntake.setCrateIntakeStateCommand(CrateIntakeState.INTAKE))
        .onFalse(crateIntake.setCrateIntakeStateCommand(CrateIntakeState.STOP));

    primaryController
        .rightBumper()
        .whileTrue(dropper.setDropperStateCommand(DropperState.OUTTAKE))
        .onFalse(dropper.setDropperStateCommand(DropperState.STOP));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
