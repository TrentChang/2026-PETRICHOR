// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.driveConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    public final RobotContainer m_robotContainer;
    public String targetLimelight = "";
    public boolean doGetTarget = false;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    private void getHasTarget(){
        if (LimelightHelpers.getFiducialID("limelight-two") != -1) {
            targetLimelight = "limelight-two";
            doGetTarget = true;
        }
        else if (LimelightHelpers.getFiducialID("limelight-one") != -1) {
            targetLimelight = "limelight-one";
            doGetTarget = true;
        }
        else {
            doGetTarget = false;
        }
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 
        Pose2d drivePose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-one").pose;
        Pose2d targetPose = Constants.driveConstants.blueHubPose.toPose2d();
        Rotation2d desiredAngle = drivePose.relativeTo(targetPose).getTranslation().getAngle();
        Rotation2d currentAngle = drivePose.getRotation();
        Rotation2d deltaAngle = currentAngle.minus(desiredAngle);
        double wrappedAngleDeg = MathUtil.inputModulus(deltaAngle.getDegrees(), -180, 180); 
        SmartDashboard.putNumber("wrapp", wrappedAngleDeg);
        double rotationalRate = driveConstants.rotationController.calculate(currentAngle.getRadians(), desiredAngle.getRadians());
        SmartDashboard.putNumber("rr", rotationalRate);

        getHasTarget();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {
        // Use external Gyro to "seed" internal IMU
        LimelightHelpers.SetIMUMode("limelight-one", 1);
        LimelightHelpers.SetIMUMode("limelight-two", 1);
    }

    @Override
    public void disabledExit() {
        // Switch to internal IMU + MT1
        LimelightHelpers.SetIMUMode("limelight-one", 1);
        LimelightHelpers.SetIMUMode("limelight-two", 1);
    }

    // @Override
    // public void autonomousInit() {
    //     m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    //     if (m_autonomousCommand != null) {
    //         CommandScheduler.getInstance().schedule(m_autonomousCommand);
    //     }
    // }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}

