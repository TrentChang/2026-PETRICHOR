package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constant.canBUS;
import frc.robot.Constant.conveyorConstant;

public class Conveyor extends SubsystemBase{
    private final TalonFX conveyorIndexer = new TalonFX(conveyorConstant.indexer, canBUS.canivore);
    private final TalonFX conveyorRoller = new TalonFX(conveyorConstant.roller, canBUS.canivore);

    // only rollers spin
    public void conveyorIntake(){
        conveyorRoller.set(conveyorConstant.transport);
    }

    // rollers and indexer spin
    public void converyorTransport(){
          conveyorIndexer.set(-conveyorConstant.transport);
        conveyorRoller.set(conveyorConstant.transport);
    }

    // avoid fuel stuck
    public void converyorReverse() {
        conveyorIndexer.set(-conveyorConstant.reverse);
        conveyorRoller.set(conveyorConstant.reverse);
    }

    // set speed 0
    public void converyStop() {
        conveyorIndexer.set(conveyorConstant.stop);
        conveyorRoller.set(conveyorConstant.stop);
    }
}
