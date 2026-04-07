// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.ejml.ops.ConvertMatrixData;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.util.ISO8601DateFormat;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.util.datalog.IntegerArrayLogEntry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.Auto_CMD.AutoIntake;
import frc.robot.commands.Auto_CMD.AutoShoot;
import frc.robot.commands.Group_CMD.BackupShoot;
import frc.robot.commands.Group_CMD.FlywheelShoot;
import frc.robot.commands.Group_CMD.FromNeutralZone;
import frc.robot.commands.Group_CMD.aimAuto;
import frc.robot.commands.Single_CMD.SmartIntake;
import frc.robot.commands.Single_CMD.SmartTransport;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake; 

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    // Subsystem
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Intake intake = new Intake();
    public final Conveyor conveyor = new Conveyor();
    public final Flywheel flywheel = new Flywheel();
    public final Hood hood = new Hood();

    // Single cmd
    public final SmartIntake smartIntake = new SmartIntake(intake, conveyor);
    public final SmartTransport smartTransport = new SmartTransport(intake);

    // Group cmd
    public final aimAuto aimAuto = new aimAuto(drivetrain, joystick, hood);
    public final FlywheelShoot flywheelShoot = new FlywheelShoot(flywheel, drivetrain, conveyor, intake);
    public final BackupShoot backupShoot = new BackupShoot(hood, flywheel, conveyor, intake);
    public final FromNeutralZone fromNeutralZone = new FromNeutralZone(hood, flywheel, conveyor, intake);

    // Auto cmd
    public final AutoShoot autoShoot = new AutoShoot(flywheel, conveyor, intake, hood, drivetrain);
    public final AutoIntake autoIntake = new AutoIntake(intake, conveyor);

    private SendableChooser<Command> autoChooser;

    public RobotContainer() {
        configureBindings();

        NamedCommands.registerCommand("shoot", autoShoot);
        NamedCommands.registerCommand("intake", autoIntake);
        NamedCommands.registerCommand("intake back", new InstantCommand(intake::intakeInhale, intake));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("AutoModeChooser", autoChooser);

        //run through a full path following command to warm up the library
        FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {
        // auto aim
        joystick.a().whileTrue(aimAuto).onFalse(new InstantCommand(hood::setPosLow, hood));
        // intake
        joystick.button(5).onTrue(smartIntake).onFalse(new InstantCommand(intake::intakeStop, intake).alongWith(new InstantCommand(conveyor::converyStop, conveyor)));
        joystick.leftTrigger(0.1).onTrue(new InstantCommand(conveyor::conveyorIntake, conveyor).alongWith(smartTransport)).onFalse(new InstantCommand(conveyor::converyStop, conveyor).alongWith(new InstantCommand(intake::intakeStop, intake)));
        // flywheel shoot
        joystick.button(6).whileTrue(flywheelShoot).onFalse(new InstantCommand(flywheel::stop, flywheel).alongWith(new InstantCommand(conveyor::converyStop, conveyor)).alongWith(new InstantCommand(intake::intakeStop, intake)));
        // shoot from neutral zone
        joystick.b().onTrue(fromNeutralZone).onFalse(new InstantCommand(flywheel::stop, flywheel).alongWith(new InstantCommand(conveyor::converyStop, conveyor)).alongWith(new InstantCommand(intake::intakeStop, intake)));
        // backup shoot 
        joystick.y().whileTrue(new InstantCommand(intake::intakeReverse, intake)).onFalse(new InstantCommand(intake::intakeStop, intake));
        // conveyor reverse
        joystick.x().whileTrue(new InstantCommand(conveyor::converyorReverse, conveyor)).onFalse(new InstantCommand(conveyor::converyStop, conveyor));
        // field centric
        joystick.button(8).onTrue(new InstantCommand(drivetrain::seedFieldCentric, drivetrain));

        // test use: set intake default pos
        // joystick.button(10).onTrue(new InstantCommand(intake::intakeDefault, intake));

        // Note that X is defined as forward according to WPILib convention, 
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * 2.5 /*3 */ * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
