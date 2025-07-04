package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera1;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.drive.commands.DriveCommands;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

    // Subsystems
    private final Drive drive;
    private final Vision vision;
    private Shooter shooter;

    // Controller
    private final CommandXboxController driver = new CommandXboxController(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOSpark(0),
                        new ModuleIOSpark(1),
                        new ModuleIOSpark(2),
                        new ModuleIOSpark(3));

                vision = new Vision(
                        drive::addVisionMeasurement,
                        // new VisionIOPhotonVision("BackCamera", robotToCamera0),
                        new VisionIOPhotonVision("BackCamera", robotToCamera1));

                shooter = new Shooter(12, 11);
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                drive = new Drive(
                        new GyroIO() {
                        },
                        new ModuleIOSim(),
                        new ModuleIOSim(),
                        new ModuleIOSim(),
                        new ModuleIOSim());

                vision = new Vision(
                        drive::addVisionMeasurement,
                        new VisionIOPhotonVisionSim("camFront", robotToCamera0, drive::getPose),
                        new VisionIOPhotonVisionSim("camBack", robotToCamera1, drive::getPose));
                break;
            default:
                // Replayed robot, disable IO implementations
                drive = new Drive(
                        new GyroIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        });

                vision = new Vision(drive::addVisionMeasurement, new VisionIO() {
                }, new VisionIO() {
                });
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
        autoChooser.addOption("Middle", new PathPlannerAuto("middle"));

        // Configure the button bindings
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive, () -> driver.getLeftY(), () -> driver.getLeftX(), () -> -driver.getRightX()));

        // Lock to 0° when A button is held
        driver
            .a()
                .whileTrue(
                        DriveCommands.joystickDriveAtAngle(
                                drive,
                                () -> -driver.getLeftY(),
                                () -> -driver.getLeftX(),
                                () -> Rotation2d.fromDegrees(-54.55)));

        // Switch to X pattern when X button is pressed
        driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

        // Slows speeds down by half
        driver
            .leftBumper()
                .whileTrue(
                        DriveCommands.joystickDrive(
                                drive,
                                () -> (driver.getLeftY() * 0.3),
                                () -> (driver.getLeftX() * 0.3),
                                () -> (driver.getRightX() * 0.3)));

        // Reset gyro to 0° when B button is pressed
        driver
                .b()
                .onTrue(
                        Commands.runOnce(
                                () -> drive.setPose(
                                        new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                                drive)
                                .ignoringDisable(true));

        driver
                .y()
                .whileTrue(
                        DriveCommands.autoTarget(
                                drive,
                                () -> (driver.getLeftY()),
                                () -> (driver.getLeftX()),
                                () -> vision.getTargetX(0, 14),
                                driver));

        if (Robot.isReal()) {
            driver
                    .rightTrigger(0.2)
                    .whileTrue(
                            Commands.run(
                                    () -> shooter.runFeeder(0.3),
                                    shooter)
                                    .alongWith(
                                            Commands.sequence(
                                                    Commands.waitSeconds(1.0),
                                                    Commands.run(
                                                            () -> shooter.runShooter(driver.getRightTriggerAxis()),
                                                            shooter))));

            driver.leftTrigger(0.2).whileTrue(
                    Commands.parallel(
                            Commands.run(() -> shooter.runShooter(-1.0), shooter),
                            Commands.run(() -> shooter.runFeeder(-1.0), shooter)));
        }
    }

    public Command getAutonomousCommand() {
        return Commands.print("No auto");
    }
    // gg bro auto aim is a pain :sob:
}
