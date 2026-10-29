package frc.robot;

import com.spikes2212.util.PlaystationControllerWrapper;

public class OI {

    private final PlaystationControllerWrapper controller = new PlaystationControllerWrapper(0);

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
