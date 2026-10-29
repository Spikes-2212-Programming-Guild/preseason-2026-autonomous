package frc.robot;

import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.SmartMotorController;

public class Test extends SmartMotorControllerGenericSubsystem {

    public Test(SmartMotorController motorController) {
        super("test", motorController);
    }
}
