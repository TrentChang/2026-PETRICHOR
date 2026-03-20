package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constant.limelightConstant;

public class Limelight extends SubsystemBase{
    public static boolean badTagData = false;
    public static String limelightUsing = "";
    
    @Override
    public void periodic() { 
        badTagData = false;
        
        if (LimelightHelpers.getFiducialID(limelightConstant.FR) != -1) {
            limelightUsing = limelightConstant.FR;
        }
        else if (LimelightHelpers.getFiducialID(limelightConstant.FL) != -1) {
            limelightUsing = limelightConstant.FL;
        }
        else {
            badTagData = true;
        }
    }
}
