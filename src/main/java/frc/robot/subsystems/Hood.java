package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import java.security.PublicKey;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constant.canBUS;
import frc.robot.Constant.hoodConstant;

public class Hood extends SubsystemBase {
    //  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private  double desireAngleToShoot;
    private static final double minAngle = Units.degreesToRadians(hoodConstant.minAngel);
    private static final double maxAngle = Units.degreesToRadians(hoodConstant.maxAngle);

    public TalonFX hoodMotor = new TalonFX(hoodConstant.angle, canBUS.canivore);
    public CANcoder hoodEncoder = new CANcoder(hoodConstant.encoder, canBUS.canivore);

    private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);
    // Keep a brake request so we can disable the motor
    private final NeutralOut m_brake = new NeutralOut();

    public Hood() {
        TalonFXConfiguration configs = new TalonFXConfiguration()
            .withFeedback(new FeedbackConfigs()
            .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
            .withFeedbackRemoteSensorID(hoodConstant.encoder)
            .withSensorToMechanismRatio(1));
        configs.Slot0.kP = hoodConstant.kP;
        configs.Slot0.kI = hoodConstant.kI;
        configs.Slot0.kD = hoodConstant.kD;
        configs.Slot0.kS = hoodConstant.kS;

        //hoodMotor.setNeutralMode(NeutralModeValue.Brake);
        configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        configs.Voltage.withPeakForwardVoltage(Volts.of(12))
                        .withPeakReverseVoltage(Volts.of(-12));

        hoodMotor.getConfigurator().apply(configs);

        // motor start at 0.0
        hoodMotor.setPosition(0.0);
    }

    // private final double distanceToHub() {
    //     Pose2d currentPose2d = drivetrain.getState().Pose;
    //     Pose2d targetPosed2d = hubConstants.getHubPose().toPose2d();
            
    //     double distanceToHub = targetPosed2d.relativeTo(currentPose2d).getTranslation().getDistance();    
    //     return distanceToHub;
    // }
    
    public void hoodSetPos() {
        hoodMotor.setControl(m_positionVoltage.withPosition(desireAngleToShoot));
    }

    public void setPosLow() {
        hoodMotor.setControl(m_positionVoltage.withPosition(-0.2));
    }

    public void setPosHigh() {
        hoodMotor.setControl(m_positionVoltage.withPosition(-1.0));
    }

    @Override
    public void periodic() {
        // desireDistanceToHub = distanceToHub();
        // desireAngleToShoot = angleToShoot();
        // SmartDashboard.putNumber("Absolute distance of HUB", desireDistanceToHub);
        SmartDashboard.putNumber("Hood angle", hoodEncoder.getAbsolutePosition().getValueAsDouble());
        
        desireAngleToShoot = SmartDashboard.getNumber("set hood angle", maxAngle);
    }
}