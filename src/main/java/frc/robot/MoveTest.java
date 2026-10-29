package frc.robot;

import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.UnifiedControlMode;

import java.util.function.Supplier;

public class MoveTest extends MoveSmartMotorControllerGenericSubsystem {

    private static final RootNamespace namespace = new RootNamespace("move test");
    private static final PIDSettings pidSettings = namespace.addPIDNamespace("pid",
            new PIDSettings(0, 0, 0));
    private static final FeedForwardSettings ffSetting = namespace.addFeedForwardNamespace("ff",
            new FeedForwardSettings(0, 0, 0, FeedForwardController.ControlMode.LINEAR_POSITION));

    public MoveTest(Test test, Supplier<Double> setpoint, boolean updatePeriodically) {
        super(test, pidSettings, ffSetting, UnifiedControlMode.POSITION, setpoint, updatePeriodically);
    }

    public static void updateNamespace() {
        namespace.update();
    }
}
