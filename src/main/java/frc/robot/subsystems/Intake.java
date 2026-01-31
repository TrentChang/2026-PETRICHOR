package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final TalonFX intakeMotor = new TalonFX(32);

    public void intakeRotate(){
        intakeMotor.set(0.4);
    }

    public void intakeReverseRotate(){
        intakeMotor.set(-0.4);
    }

    public void intakeStop(){
        intakeMotor.set(0);
    }
}
