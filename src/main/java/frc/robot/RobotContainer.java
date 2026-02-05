// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.Intake;
import frc.robot.commands.autos.AutoAim;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    // private final SwerveRequest.SwerveDriveBrake swerveDriveBrake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Telemetry logger = new Telemetry(MaxSpeed);

    //controller
    private final CommandXboxController driverCtrl = new CommandXboxController(0);
    private final XboxController opController = new XboxController(0);

    //drivetrain
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    //Subsystem
    public final Intake intake = new Intake();
    // public final autoAlignmentDrive m_aAutoAlignmentDrive = new autoAlignmentDrive.alignDrive(driverCtrl, () -> driveConstants.getHubPose().toPose2d());

    //Command
    public final AutoAim mAutoAim = new AutoAim(drivetrain, driverCtrl);

    //Path followers
    // private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // NamedCommands.registerCommand("autoHubAlign", mAutoAim);

        // autoChooser = AutoBuilder.buildAutoChooser();
        // SmartDashboard.putData("AutoMode", autoChooser);

        configureBindings();
        
        //run through a full path following command to warm up the library.
        // FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {
        driverCtrl.a().whileTrue(mAutoAim);

        // driverCtrl.b().whileTrue(new InstantCommand(intake::intakeReverseRotate)).onFalse(new InstantCommand(intake::intakeStop, intake));
        new JoystickButton(opController, 5).whileTrue(new InstantCommand(intake::intakeRotate, intake)).onFalse(new InstantCommand(intake::intakeStop, intake));
        new JoystickButton(opController, 6).whileTrue(new InstantCommand(intake::intakeReverseRotate, intake)).onFalse(new InstantCommand(intake::intakeStop, intake));

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverCtrl.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverCtrl.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverCtrl.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // // Idle while the robot is disabled. This ensures the configured
        // // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // driverCtrl.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverCtrl.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverCtrl.getLeftY(), -driverCtrl.getLeftX()))
        ));

        // Reset the field-centric heading on left bumper press.
        // driverCtrl.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }  

    // public Command getAutonomousCommand() {
    //     /* Run the path selected from the auto chooser */
    //     return autoChooser.getSelected();
    // }

    // public Command getAutonomousCommand() {
    //     // Simple drive forward auton
    //     final var idle = new SwerveRequest.Idle();
    //     return Commands.sequence(
    //         // Reset our field centric heading to match the robot
    //         // facing away from our alliance station wall (0 deg).
    //         drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
    //         // Then slowly drive forward (away from us) for 5 seconds.
    //         drivetrain.applyRequest(() ->
    //             drive.withVelocityX(0.5)
    //                 .withVelocityY(0)
    //                 .withRotationalRate(0)
    //         )
    //         .withTimeout(5.0),
    //         // Finally idle for the rest of auton
    //         drivetrain.applyRequest(() -> idle)
    //     );
    // }
}
