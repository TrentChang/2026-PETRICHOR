package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.generated.TunerConstants;

public class Constant {
    public static class canBUS {
        public static final String rio = "rio";
        public static final String canivore = "CANivore";
    }

    public static class pigeon2Constant{
        public static final int pigeon2 = 13;
    }

    public static class intakeConstant {
        // intake motor id
        public static final int extend = 31;
        public static final int roller = 32;
        public static final int encoder = 33;
        
        // PID
        public static final double kP = 10;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0; 

        // intake Angle
        public static final double extendAngle = 0.0;
        public static final double systoleAngle = 0.0;
        public static final double defaultAngle = 0.0;
    }

    public static class conveyorConstant {
        //conveyor motor id
        public static final int indexer = 41;
        public static final int roller = 42;

        public static final double transmiss = 0.95;
        public static final double reverse = -0.95;
        public static final double stop = 0.0;  
    }

    public static class flyWheelConstant {
        //shooter motor id
        public static final int FR = 21;
        public static final int BR = 22;
        public static final int FL = 23;
        public static final int BL = 24;

        public static final double kP = 6.8;
        public static final double kI = 0.0;
        public static final double kD = 0.0068;

        public static final double speedLimit = 2500;  // RPM
    }

    public static class KrakenX60 {
        public static final AngularVelocity kFreeSpeed = RPM.of(6000);
    }

    public static class hoodConstant {
        public static final int angle = 25;
        public static final int encoder = 26;

        public static final double minAngle = -0.03;
        public static final double maxAngle = -1.24;

        public static final double kP = 80;
        public static final double kI = 0;
        public static final double kD = 0.05;
        public static final double kS = 0.08;
    }

    public static class limelightConstant {
        public static final String FM = "limelight-fm";
        public static final String FL = "limelight-fl";
    }

    public static class autoAimConstant {
        public static final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        public static final double maxAngularRate = Units.RotationsPerSecond.of(0.75).in(Units.RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

        public static final Angle epsilonAngleToGoal = Degree.of(0.1); //robot will stop if it's in range of 5deg

        public static final PIDController rotationController = getRotationController();

        private static final PIDController getRotationController() {
            PIDController controller = new PIDController(2.5, 0.0, 0.00514); 
            controller.enableContinuousInput(-Math.PI, Math.PI);
            return controller;
        }
    }

    public static class hubConstants {
        //wpiblue view
        public static final Pose3d redHubPose = new Pose3d(Units.Meters.of(11.915394), Units.Meters.of(4.0345), Units.Meters.of(1.83), new Rotation3d());
        public static final Pose3d blueHubPose = new Pose3d(Units.Meters.of(4.625594), Units.Meters.of(4.0345),  Units.Meter.of(1.83), new Rotation3d());

        //decide blue or red hub pose by driverstation
        public static final Pose3d getHubPose() {
            Pose3d hubPose = DriverStation.getAlliance().equals(Optional.of(Alliance.Blue)) ? blueHubPose : redHubPose;
            return hubPose;
        }
    }
}
