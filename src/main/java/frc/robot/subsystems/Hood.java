package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constant.canBUS;
import frc.robot.Constant.hoodConstant;

public class Hood extends SubsystemBase {
    private final TalonFX hood = new TalonFX(hoodConstant.angle, canBUS.canivore);
    private boolean hoodAtPosition = false;

    MotionMagicExpoVoltage hoodMotionRequest = new MotionMagicExpoVoltage(0);
    Angle lastDesiredHoodAngle = Degrees.zero();

    public Hood() {
    }
}