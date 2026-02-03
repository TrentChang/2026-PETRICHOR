package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.driveConstants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.LimelightHelpers;

public class AutoAlign extends Command{
    private final CommandSwerveDrivetrain drivetrain;
    private final CommandXboxController driverCtrl;
    private String targetLimelight = "";
    public boolean doGetTarget = false;

    public AutoAlign(CommandXboxController driverCtrl, CommandSwerveDrivetrain drivetrain){
        this.driverCtrl = driverCtrl;
        this.drivetrain = drivetrain;
    }

    // private Boolean LL1Fid(){
    //     return LimelightHelpers.getFiducialID(LL1) == -1;
    // }

    // private Boolean LL2Fid(){
    //     return LimelightHelpers.getFiducialID(LL2) == -1;
    // }
    
    public void getHasTarget(){
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
    public void execute(){
        getHasTarget();

        drivetrain.applyRequest(()-> {
            double controllerVelX = -driverCtrl.getLeftX();
            double controllerVelY = -driverCtrl.getLeftY();
            Pose2d drivePose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(targetLimelight).pose;
            Pose2d targetPose = Constants.driveConstants.blueHubPose.toPose2d();
            Rotation2d desiredAngle = drivePose.relativeTo(targetPose).getTranslation().getAngle();
            Rotation2d currentAngle = drivePose.getRotation();
            Rotation2d deltaAngle = currentAngle.minus(desiredAngle);
            double wrappedAngleDeg = MathUtil.inputModulus(deltaAngle.getDegrees(), -180, 180); 

            if ((Math.abs(wrappedAngleDeg) < driveConstants.epsilonAngleToGoal.in(Degrees)) // if facing goal already
                && Math.hypot(controllerVelX, controllerVelY) < 0.1 || doGetTarget != false) {
                return new SwerveRequest.SwerveDriveBrake();
            }     
            else {
                double rotationalRate = driveConstants.rotationController.calculate(currentAngle.getRadians(), desiredAngle.getRadians());
                return drivetrain.alignDrive.withVelocityX(controllerVelX * driveConstants.maxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(-controllerVelY * driveConstants.maxSpeed) // Drive left with negative X (left)
                .withRotationalRate(rotationalRate * driveConstants.maxAngularRate); // Use angular rate for rotation             
            }
        });
    }
}
