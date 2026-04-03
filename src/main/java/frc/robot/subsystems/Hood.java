package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constant.canBUS;
import frc.robot.Constant.hoodConstant;

public class Hood extends SubsystemBase {
    private static final double minAngle = hoodConstant.minAngle;
    private static final double maxAngle = hoodConstant.maxAngle;

    public TalonFX hoodMotor = new TalonFX(hoodConstant.angle, canBUS.canivore);
    public CANcoder hoodEncoder = new CANcoder(hoodConstant.encoder, canBUS.canivore);

    private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);

    public Hood() {
        TalonFXConfiguration configs = new TalonFXConfiguration()
            .withFeedback(new FeedbackConfigs()
            .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
            .withFeedbackRemoteSensorID(hoodConstant.encoder)
            .withSensorToMechanismRatio(1));
        configs.Slot0.kP = hoodConstant.kP;
        configs.Slot0.kI = hoodConstant.kI;
        configs.Slot0.kD = hoodConstant.kD;
        configs.Slot0.kS = hoodConstant.kS; // voltage support to overcome friction

        // set motor brake
        hoodMotor.setNeutralMode(NeutralModeValue.Brake);   

        configs.Voltage.withPeakForwardVoltage(Volts.of(12))
                        .withPeakReverseVoltage(Volts.of(-12));

        configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        hoodMotor.getConfigurator().apply(configs);
    }

    public double hoodAngle(double distanceToHub) {
        double angle = (-0.0433 * Math.pow(distanceToHub, 2)) + (0.08294 * (distanceToHub)) + (-0.04094);
        if (angle < hoodConstant.maxAngle) {angle = maxAngle;}
        else if (angle > hoodConstant.minAngle) {angle = minAngle;} 
        SmartDashboard.putNumber("hood angle ctrl", angle);
        return angle;
    }
    
    public Command hoodSetPos(Supplier<Double> distance) {
        //hoodMotor.setControl(m_positionVoltage.withPosition(hoodAngle(distance)));
        return run(()-> hoodMotor.setControl(
            m_positionVoltage.withPosition(
                hoodAngle(distance.get())
            )
        ));
    }

    public void setPosLow() {
        hoodMotor.setControl(m_positionVoltage.withPosition(minAngle));
    }
    
    public void setPosHigh() {

        hoodMotor.setControl(m_positionVoltage.withPosition(-0.8)); 
    }

    @Override   
    public void periodic() {
        SmartDashboard.putNumber("Hood angle", hoodEncoder.getPosition().getValueAsDouble());
    }
}