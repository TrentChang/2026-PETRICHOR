package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constant.limelightConstant;

public class Limelight extends SubsystemBase{
    public static boolean badTagData = false;
    public static String limelightUsing = "";

    public static String limelightUsing() {
        if (LimelightHelpers.getFiducialID(limelightConstant.FM) != -1) {
            limelightUsing = limelightConstant.FM;
        }
        else if (LimelightHelpers.getFiducialID(limelightConstant.FL) != -1) {
            limelightUsing = limelightConstant.FL;
        }

        return limelightUsing;
    }

    public static Boolean badTagData() {
        badTagData = false;

        if (LimelightHelpers.getFiducialID(limelightConstant.FM) != -1) {
            badTagData = false;
        }
        else if (LimelightHelpers.getFiducialID(limelightConstant.FL) != -1) {
            badTagData = false;
        }
        else {
            badTagData = true;
        }

        return badTagData;
    }
}
