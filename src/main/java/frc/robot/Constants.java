package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;

public class Constants {
    // public static class fieldConstants {
    //     public static final double hubCentricXInMeters = 4.625594;
    //     public static final double hubCentricYInMeters = 4.034663;
    // }
    
    // public static class controlBoardConstants {
    //     public static final double triggerThreshold = 0.2;
    //     public static final double stickDeadband = 0.1;
    // }

    // public static class driveConstants {
    //     public static final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    //     public static final double maxAngularRate = Units.RotationsPerSecond.of(0.75).in(Units.RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
        
    //     public static final Pose3d redHubPose = new Pose3d(Units.Inches.of(468.56), Units.Inches.of(158.32), Units.Inches.of(72.0), new Rotation3d());
    //     public static final Pose3d blueHubPose = new Pose3d(Units.Inches.of(152.56), Units.Inches.of(158.32),  Units.Inches.of(72.0), new Rotation3d());

    //     // public static final Pose3d redFerryPoseDepot = new Pose3d(14.3, 6, 0, Rotation3d.kZero);
    //     // public static final Pose3d redFerryPoseOutpost = new Pose3d(14.3, 2, 0, Rotation3d.kZero);
    //     // public static final Pose3d blueFerryPoseDepot = new Pose3d(2.1, 2, 0, Rotation3d.kZero);
    //     // public static final Pose3d blueFerryPoseOutpost = new Pose3d(2.1, 6, 0, Rotation3d.kZero);

    //     public static final Angle epsilonAngleToGoal = Degree.of(1.0);

    //     public static final PIDController rotationController = getRotationController();

    //     private static final PIDController getRotationController() {
    //         PIDController controller = new PIDController(1.5, 0.0, 0.0);
    //         controller.enableContinuousInput(-Math.PI, Math.PI);
    //         return controller;
    //     }

    //     public static final Pose3d getHubPose() {
    //         Pose3d pose = DriverStation.getAlliance().equals(Optional.of(Alliance.Blue)) ? blueHubPose : redHubPose;
    //         return pose;
    //     }
    // }

    public static class swerveDriveConstants {
        public static final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        public static final double maxAngularRate = Units.RotationsPerSecond.of(0.75).in(Units.RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    }

    public static class visionConstants {
        public static final String NAME1 = "limelight1";
        public static final Transform3d ll1_to_ROBOT = new Transform3d();
        public static final String NAME2 = "limelight2";
        public static final Transform3d ll2_to_ROBOT = new Transform3d();
    }
}
