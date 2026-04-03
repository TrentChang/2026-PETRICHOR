// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import org.ejml.ops.ConvertMatrixData;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
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

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        joystick.a().whileTrue(drivetrain.autoAlignCommand(joystick)
                                .alongWith(hood.hoodSetPos((drivetrain::getDistanceToTarget))))
                    .onFalse(new InstantCommand(hood::setPosLow, hood));

        joystick.b().whileTrue(new InstantCommand(conveyor::conveyorTransmiss, conveyor)
                                .alongWith(flywheel.dashboardSpinUpCommand(drivetrain.getDistanceToTarget())))
                    .onFalse(new InstantCommand(conveyor::converyStop, conveyor)
                                .alongWith(new InstantCommand(flywheel::stop, flywheel)));
        // joystick.x().onTrue(new InstantCommand(intake::intakeExtend, intake));
        // joystick.y().onTrue(new InstantCommand(intake::intakeSystole, intake));
        // joystick.a().whileTrue(new InstantCommand(intake::intakeInhale, intake))
        //             .onFalse(new InstantCommand(intake::intakeStop, intake));

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
}
