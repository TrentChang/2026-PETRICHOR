package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constant.flyWheelConstant;
import frc.robot.Constant.KrakenX60;
import frc.robot.Constant.canBUS;

public class Flywheel extends SubsystemBase{
    private static final AngularVelocity kVelocityTolerance = RPM.of(100);

    private final TalonFX FRMotor, BRMotor, FLMotor, BLMotor;
    private final List<TalonFX> motors;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    public Flywheel() { 
        FRMotor = new TalonFX(flyWheelConstant.FR, canBUS.canivore);
        BRMotor = new TalonFX(flyWheelConstant.BR, canBUS.canivore);
        FLMotor = new TalonFX(flyWheelConstant.FL, canBUS.canivore);
        BLMotor = new TalonFX(flyWheelConstant.BL, canBUS.canivore);
        motors = List.of(FRMotor, BRMotor, FLMotor, BLMotor);

        configureMotor(FRMotor, InvertedValue.Clockwise_Positive);
        configureMotor(BRMotor, InvertedValue.Clockwise_Positive);
        configureMotor(FLMotor, InvertedValue.CounterClockwise_Positive);
        configureMotor(BLMotor, InvertedValue.CounterClockwise_Positive);
    }

    private void configureMotor(TalonFX motor, InvertedValue invertedDirection) {
         final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(invertedDirection)
                    .withNeutralMode(NeutralModeValue.Coast)
            )
            .withVoltage(
                new VoltageConfigs()
                    .withPeakReverseVoltage(Volts.of(0))
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(70))
                    .withSupplyCurrentLimitEnable(true)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(flyWheelConstant.kP)
                    .withKI(flyWheelConstant.kI)
                    .withKD(flyWheelConstant.kD)
                    .withKV(12.0 / KrakenX60.kFreeSpeed.in(RotationsPerSecond)) // 12 volts when requesting max RPS
            );
        
        motor.getConfigurator().apply(config);
    }

    private double getTargetRPM(double distanceToHub) {
        double setRPM = ((-17.04 * Math.pow(distanceToHub, 3)) + (145.7 * Math.pow(distanceToHub, 2)) + (-197.6  * distanceToHub) + 1719) * 0.93;
        return setRPM;
    }

    public void setRPM(double rpm) { 
        if (Math.abs(rpm) > flyWheelConstant.speedLimit) {
            rpm = flyWheelConstant.speedLimit * Math.signum(rpm);
        }
        for (final TalonFX motor : motors) { // read and set every motor from list "motors" using ":"
            motor.setControl(
                velocityRequest
                .withVelocity(RPM.of(rpm))
            );
        }
    }

    public void setPercentOutput(double percentOutput) {
        for (final TalonFX motor : motors) {
            motor.setControl(
                voltageRequest
                    .withOutput(Volts.of(percentOutput * 12.0))
            );
        }
    }
    public void stop() {
        setPercentOutput(0.0);
    }

    // spin up to rpm
    public Command spinUpCommand(double rpm) {
        return runOnce(() -> setRPM(rpm))
            .andThen(Commands.waitUntil(this::isVelocityWithinTolerance));  
    }

    public Command dashboardSpinUpCommand(double distance) {
        return defer(() -> spinUpCommand(getTargetRPM(distance)));
    }

    public boolean isVelocityWithinTolerance() {
        return motors.stream().allMatch(motor -> {
            final boolean isInVelocityMode = motor.getAppliedControl().equals(velocityRequest);
            final AngularVelocity currentVelocity = motor.getVelocity().getValue();
            final AngularVelocity targetVelocity = velocityRequest.getVelocityMeasure();
            return isInVelocityMode && currentVelocity.isNear(targetVelocity, kVelocityTolerance);
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("rpm", FRMotor.getVelocity().getValueAsDouble()*60);
        // SmartDashboard.putNumber("static RPM", rpm);
    }
}
