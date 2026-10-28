package frc.robot;

import com.spikes2212.util.XboxControllerWrapper;

public class OI {

    private final XboxControllerWrapper controller = new XboxControllerWrapper(0);

    public OI() {

    }

    public double getLeftX() {
        return controller.getLeftX();
    }

    public double getLeftY() {
        return controller.getLeftY();
    }

    public double getRightX() {
        return controller.getRightX();
    }
}
