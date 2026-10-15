package frc.robot;

import com.spikes2212.util.PlaystationControllerWrapper;
import frc.robot.swerve.commands.Drive;

public class OI {

    private static final PlaystationControllerWrapper driver = new PlaystationControllerWrapper(0);

    public OI() {

    }

    public double getRightX(){
        return driver.getRightX();
    }

    public double getRightY(){
        return driver.getRightY();
    }

    public double getLeftX(){
        return driver.getLeftX();
    }

    public double getLeftY(){
        return driver.getLeftY();
    }
}
