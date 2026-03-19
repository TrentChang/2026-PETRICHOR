package frc.robot;

import edu.wpi.first.networktables.PubSub;

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
}
