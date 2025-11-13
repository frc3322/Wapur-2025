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
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.ballIntake.BallIntake;
import frc.robot.subsystems.ballIntake.BallIntakeConstants.BallIntakeState;
import frc.robot.subsystems.ballIntake.BallIntakeIO;
import frc.robot.subsystems.ballIntake.BallIntakeIOReal;
import frc.robot.subsystems.crateIntake.CrateIntake;
import frc.robot.subsystems.crateIntake.CrateIntakeConstants.CrateIntakeState;
import frc.robot.subsystems.crateIntake.CrateIntakeIO;
import frc.robot.subsystems.crateIntake.CrateIntakeIOReal;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorState;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterState;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private static final double TRIGGER_THRESHOLD = 0.1;

  // Subsystems
  private final Drive drive;
  private final Elevator elevator;
  private final BallIntake ballIntake;
  private final CrateIntake crateIntake;
  private final Shooter shooter;
  private final Vision vision = new Vision();

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        elevator = new Elevator(new ElevatorIOReal());
        ballIntake = new BallIntake(new BallIntakeIOReal());
        crateIntake = new CrateIntake(new CrateIntakeIOReal());
        shooter = new Shooter(new ShooterIOReal());
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
        elevator = new Elevator(new ElevatorIO() {});
        ballIntake = new BallIntake(new BallIntakeIO() {});
        crateIntake = new CrateIntake(new CrateIntakeIO() {});
        shooter = new Shooter(new ShooterIO() {});
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
        elevator = new Elevator(new ElevatorIO() {});
        ballIntake = new BallIntake(new BallIntakeIO() {});
        crateIntake = new CrateIntake(new CrateIntakeIO() {});
        shooter = new Shooter(new ShooterIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

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
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    controller
        .leftBumper()
        .whileTrue(
            Commands.startEnd(
                () -> ballIntake.setState(BallIntakeState.INTAKE),
                () -> ballIntake.setState(BallIntakeState.STOP),
                ballIntake));

    controller
        .leftTrigger(TRIGGER_THRESHOLD)
        .whileTrue(
            Commands.startEnd(
                () -> ballIntake.setState(BallIntakeState.OUTTAKE),
                () -> ballIntake.setState(BallIntakeState.STOP),
                ballIntake));

    controller
        .rightBumper()
        .whileTrue(
            Commands.startEnd(
                () -> crateIntake.setState(CrateIntakeState.INTAKE),
                () -> crateIntake.setState(CrateIntakeState.STOP),
                crateIntake));

    controller
        .rightTrigger(TRIGGER_THRESHOLD)
        .whileTrue(
            Commands.startEnd(
                () -> crateIntake.setState(CrateIntakeState.OUTTAKE),
                () -> crateIntake.setState(CrateIntakeState.STOP),
                crateIntake));

    controller.povUp().onTrue(elevator.setStateCommand(ElevatorState.UP));
    controller.povDown().onTrue(elevator.setStateCommand(ElevatorState.DOWN));
    controller.povRight().onTrue(elevator.setStateCommand(ElevatorState.STOW));

    controller
        .y()
        .whileTrue(
            Commands.startEnd(
                () -> shooter.setState(ShooterState.SHOOTING),
                () -> shooter.setState(ShooterState.OFF),
                shooter));

    controller
        .start()
        .whileTrue(
            Commands.startEnd(
                () -> shooter.setState(ShooterState.FEEDING),
                () -> shooter.setState(ShooterState.OFF),
                shooter));
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
