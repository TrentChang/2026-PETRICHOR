package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constant.limelightConstant;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Limelight extends SubsystemBase{
    public static boolean doRejectUpdate = false;
    public static String limelightUsing = "";
    public static LimelightHelpers.PoseEstimate mt2;
    private PoseEstimate m_mt2;
    private Pigeon2 m_pigeon2 = new Pigeon2(13, "rio");

    private PoseEstimate megaTag2FR(String limelightName) {
        m_mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        return m_mt2;
    }
    
    @Override
    public void periodic() {        
        if (LimelightHelpers.getFiducialID(limelightConstant.FR) != -1) {
            limelightUsing = limelightConstant.FR;
        }
        else if (LimelightHelpers.getFiducialID(limelightConstant.FR) != -1) {
            limelightUsing = limelightConstant.FL;
        }
        else {
            if (Math.abs(m_pigeon2.getAngularVelocityZWorld().getValueAsDouble()) > 720){
                doRejectUpdate = true;
            }
            if (mt2.tagCount == 0){
                doRejectUpdate = true;
            }
        }

        mt2 = megaTag2FR(limelightUsing);
    }
}
