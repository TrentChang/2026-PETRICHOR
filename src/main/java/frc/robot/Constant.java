package frc.robot;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;

public class Constant {
    public static class intakeConstant {
        // intake motor id
        public static final int extend = 31;
        public static final int roller = 32;
        public static final int encoder = 33;
        
        // PID extend
        public static final double extendP = 10;
        public static final double extendI = 0;
        public static final double extendD = 0;
        public static final double extendF = 0; 

        // PID systole
        public static final double systoleP = 10;
        public static final double systoleI = 0;
        public static final double systoleD = 0;
        public static final double systoleF = 0;
        
        // intake extend Angle
        public static final double extendAngle = 0.0;
        public static final double systoleAngle = 0.0;
        public static final double defaultAngle = 0.0;
    }

    public static class conveyorConstant {
        //conveyor motor id
        public static final int indexer = 41;
        public static final int roller = 42;
    }

    public static class shooterConstant {
        //shooter motor id
        public static final int FR = 21;
        public static final int BR = 22;
        public static final int FL = 23;
        public static final int BL = 24;
    }

    public static class KrakenX60 {
        public static final AngularVelocity kFreeSpeed = RPM.of(6000);
    }

    public static class limelightConstant {
        public static final String FR = "limelight-FR";
        public static final String FL = "limelight-FL";
    }
}
