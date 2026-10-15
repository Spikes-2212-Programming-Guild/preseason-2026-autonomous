package frc.robot;

import com.spikes2212.util.PlaystationControllerWrapper;
import frc.robot.swerve.commands.Drive;

public class OI {

    private static final PlaystationControllerWrapper driver = new PlaystationControllerWrapper(0);

    public OI() {
        configureDriver();
    }

    private void configureDriver(){
        driver.getRightStickButton().onTrue(new Drive(driver::getRightX, driver::getRightY, () -> 0.0,
                true, true));
        driver.getLeftStickButton().onTrue(new Drive(() -> 0.0, () -> 0.0, driver::getLeftX, true,
                true));
    }
}
