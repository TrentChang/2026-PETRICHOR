package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constant.intakeConstant;

public class Intake extends SubsystemBase{
    private final TalonFX intakeExtend = new TalonFX(intakeConstant.extend, "CANivore");
    private final TalonFX intakeRoller = new TalonFX(intakeConstant.roller, "CANivore");
    private final CANcoder intakeENcoder = new CANcoder(intakeConstant.encoder, "CANivore");

    public Intake(){
        var intakeCtrlConfig = intakeExtend.getConfigurator();
        // intakeMotorConfig.Inverted  = InvertedValue.CounterClockwise_Positive;

        // set feedback sensor as integrated sensor
        intakeCtrlConfig.apply(new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                .withFeedbackRemoteSensorID(intakeConstant.encoder));

        // set maximum acceleration and velocity        
        intakeCtrlConfig.apply(new MotionMagicConfigs()
                .withMotionMagicAcceleration(500)
                .withMotionMagicCruiseVelocity(200));

        intakeRoller.setNeutralMode(NeutralModeValue.Brake);// intakeExtend.getConfigurator().apply(intakeMotorConfig);
        
        // Pivot PIDConfig
        // TODO:adjust PID value
        Slot0Configs intakeExtendPIDConfig = new Slot0Configs();
        intakeExtendPIDConfig.kP = intakeConstant.extendP;
        intakeExtendPIDConfig.kI = intakeConstant.extendI;
        intakeExtendPIDConfig.kD = intakeConstant.extendD;
        intakeExtendPIDConfig.kV = intakeConstant.extendF;
        intakeCtrlConfig.apply(intakeExtendPIDConfig);

        // Pivot PIDConfig
        // TODO:adjust PID value
        Slot1Configs intakeSystolePIDConfig = new Slot1Configs();
        intakeSystolePIDConfig.kP = intakeConstant.systoleP;
        intakeSystolePIDConfig.kI = intakeConstant.systoleI;
        intakeSystolePIDConfig.kD = intakeConstant.systoleD;
        intakeSystolePIDConfig.kV = intakeConstant.systoleF;
        intakeCtrlConfig.apply(intakeSystolePIDConfig);

        intakeExtend.setPosition(0);
    }

    // intake extend
    public void intakeExtend() {
        intakeExtend.setControl(new MotionMagicDutyCycle(0).withSlot(0));
    }

    // intake systole
    public void intakeSystole() {
        intakeExtend.setControl(new MotionMagicDutyCycle(0).withSlot(1));
    }

    // intake setzero
    public void intakeSetDefault() {
        intakeExtend.setControl(new MotionMagicDutyCycle(null).withSlot(0));
    }

    @Override
    public void periodic() {
        // Output encoder angle value
        SmartDashboard.putNumber("Intake Encoder", intakeENcoder.getAbsolutePosition().getValueAsDouble());
    }
}