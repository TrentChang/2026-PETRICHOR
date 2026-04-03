package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constant.canBUS;
import frc.robot.Constant.intakeConstant;

public class Intake extends SubsystemBase{
    private final TalonFX intakeExtend = new TalonFX(intakeConstant.extend, canBUS.canivore);
    private final TalonFX intakeRoller = new TalonFX(intakeConstant.roller, canBUS.canivore);
    private final CANcoder intakeENcoder = new CANcoder(intakeConstant.encoder, canBUS.canivore);

    private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);

    public Intake(){
       TalonFXConfiguration configs = new TalonFXConfiguration()
            .withFeedback(new FeedbackConfigs()
            .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
            .withFeedbackRemoteSensorID(intakeConstant.encoder)
            .withSensorToMechanismRatio(1));
        configs.Slot0.kP = intakeConstant.kP;
        configs.Slot0.kI = intakeConstant.kI;
        configs.Slot0.kD = intakeConstant.kD;
        configs.Slot0.kS = intakeConstant.kS;
        
        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        intakeExtend.setNeutralMode(NeutralModeValue.Coast);

        configs.Voltage.withPeakForwardVoltage(Volts.of(12))
                        .withPeakReverseVoltage(Volts.of(-12));
                        
        intakeExtend.getConfigurator().apply(configs);
    }

    // intake extend
    public void intakeExtend() {
        intakeExtend.setControl(m_positionVoltage.withPosition(4.0));
    }

    // intake systole
    public void intakeSystole() {
        intakeExtend.setControl(m_positionVoltage.withPosition(0.5));
    }

    // intake setzero
    // public void intakeSetPosDefault() {
    //     intakeExtend.setControl(new MotionMagicDutyCycle(null).withSlot(0));
    // }

    // intaking
    public void intakeInhale() {
        intakeRoller.set(0.25);
    }

    // stop
    public void intakeStop() {
        intakeRoller.set(0.0);
    }

    @Override
    public void periodic() {
        // Output encoder angle value
        SmartDashboard.putNumber("Intake Encoder", intakeENcoder.getPosition().getValueAsDouble());
    }
}