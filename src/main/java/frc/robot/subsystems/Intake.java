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
import frc.robot.Constant.canBUS;
import frc.robot.Constant.intakeConstant;

public class Intake extends SubsystemBase{
    private final TalonFX intakeExtend = new TalonFX(intakeConstant.extend, canBUS.canivore);
    private final TalonFX intakeRoller = new TalonFX(intakeConstant.roller, canBUS.canivore);
    private final CANcoder intakeENcoder = new CANcoder(intakeConstant.encoder, canBUS.canivore);

    public Intake(){
        var intakeCtrlConfig = intakeExtend.getConfigurator();
        intakeExtend.setNeutralMode(NeutralModeValue.Brake);
        // set feedback sensor as integrated sensor
        intakeCtrlConfig.apply(new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder) // do not use remote sensor
                .withFeedbackRemoteSensorID(intakeConstant.encoder)
                .withSensorToMechanismRatio(48.0/32.0));

        // set maximum acceleration and velocity        
        intakeCtrlConfig.apply(new MotionMagicConfigs()
                .withMotionMagicAcceleration(100)
                .withMotionMagicCruiseVelocity(50));
        
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

        intakeENcoder.setPosition(0.0);
    }

    // intake extend
    public void intakeExtend() {
        intakeExtend.setControl(new MotionMagicDutyCycle(0.3).withSlot(0));
    }

    // intake systole
    public void intakeSystole() {
        intakeExtend.setControl(new MotionMagicDutyCycle(-0.1).withSlot(1));
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
        SmartDashboard.putNumber("Intake Encoder", intakeENcoder.getAbsolutePosition().getValueAsDouble());
    }
}