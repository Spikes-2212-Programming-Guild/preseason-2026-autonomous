package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.spikes2212.command.drivetrains.swerve.SwerveModule;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.smartmotorcontrollers.SparkWrapper;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SwerveModuleRebuilt extends SwerveModule {

    private static final RootNamespace MODULE_TUNER = new RootNamespace("module tuner");

    private static final boolean UPDATE_TURN_PERIODICALLY = false;
    private static final boolean UPDATE_DRIVE_PERIODICALLY = false;

    private static final PIDSettings DRIVE_PID_SETTINGS = MODULE_TUNER.addPIDNamespace("drive",
            new PIDSettings(0, 0, 0));

    private static final FeedForwardSettings DRIVE_FF_SETTINGS = MODULE_TUNER.addFeedForwardNamespace("drive",
            new FeedForwardSettings(0.195, 0, FeedForwardController.ControlMode.LINEAR_VELOCITY));

    private static final PIDSettings TURN_PID_SETTINGS = MODULE_TUNER.addPIDNamespace("turn",
            new PIDSettings(0.0005, 0, 0.00015, 0, 0, 10));

    private static final FeedForwardSettings TURN_FF_SETTINGS = MODULE_TUNER.addFeedForwardNamespace("turn",
            new FeedForwardSettings(FeedForwardController.ControlMode.LINEAR_POSITION));

    private static final double MIN_SPEED_LIMIT = 0.1;
    private static final double DRIVE_GEAR_RATIO = 1 / 6.12;
    private static final double TURN_GEAR_RATIO = (double) 7 / 150;
    private static final double WHEEL_DIAMETER_METERS = 0.1016;
    private static final int DEGREES_IN_ROTATION = 360;
    private static final int SECONDS_IN_MINUTE = 60;
    private static final int ABSOLUTE_POSITION_DISCONTINUITY_POINT = 1;

    private static final double DRIVE_MOTOR_ROTATION_TO_WHEEL_ROTATIONS =
            DRIVE_GEAR_RATIO * WHEEL_DIAMETER_METERS * Math.PI;
    private static final double TURN_VELOCITY_IN_ROTATION =
            (TURN_GEAR_RATIO * DEGREES_IN_ROTATION) / SECONDS_IN_MINUTE;
    private static final double TURN_POSITION_IN_ROTATION =
            TURN_GEAR_RATIO * DEGREES_IN_ROTATION;

    private static final boolean IS_SIMULATION = true;

    private final TalonFXWrapper driveMotor;
    private final SparkWrapper turnMotor;
    private final CANcoder cancoder;

    private DCMotorSim driveMotorSim;
    private TalonFXSimState talonFXSim;
    private CANcoderSimState cancoderSim;

    public SwerveModuleRebuilt(String namespaceName, TalonFXWrapper driveMotor, SparkWrapper turnMotor,
                               boolean driveMotorInverted, boolean turnMotorInverted,
                               double absoluteEncoderOffsetDegrees, CANcoder cancoder) {
        super(namespaceName, driveMotor, turnMotor, driveMotorInverted, turnMotorInverted, absoluteEncoderOffsetDegrees,
                DRIVE_PID_SETTINGS, TURN_PID_SETTINGS, DRIVE_FF_SETTINGS, TURN_FF_SETTINGS, MIN_SPEED_LIMIT,
                UPDATE_DRIVE_PERIODICALLY, UPDATE_TURN_PERIODICALLY);
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.cancoder = cancoder;
        driveMotor.restoreFactoryDefaults();
        turnMotor.restoreFactoryDefaults();

        driveMotor.setInverted(driveMotorInverted);
        turnMotor.setInverted(turnMotorInverted);

        if (IS_SIMULATION) {
            this.absoluteEncoderOffsetDegrees = 0;
            turnMotor.configureSimulation(DCMotor.getNEO(1), 1.0, 0.001);
            cancoderSim = cancoder.getSimState();
            talonFXSim = driveMotor.getSim();
            driveMotorSim = new DCMotorSim(
                    LinearSystemId.createDCMotorSystem(
                            DCMotor.getKrakenX60(1),
                            0.025,
                            6.12
                    ),
                    DCMotor.getKrakenX60(1)
            );
        }

        configureTurnController();
        configureDriveController();
        configureAbsoluteEncoder();
        configureDashboard();
    }

    public static void updateModuleTuner() {
        MODULE_TUNER.update();
    }

    @Override
    protected void configureDriveController() {
        driveMotor.setIdleMode(NeutralModeValue.Brake);
        driveMotor.setEncoderConversionFactor(DRIVE_MOTOR_ROTATION_TO_WHEEL_ROTATIONS);
        driveMotor.configureLoop(driveMotorPIDSettings, driveMotorFeedForwardSettings,
                TrapezoidProfileSettings.EMPTY_TRAPEZOID_PROFILE_SETTINGS);
    }

    @Override
    protected void configureTurnController() {
        turnMotor.setIdleMode(SparkBaseConfig.IdleMode.kBrake);
        turnMotor.setPositionConversionFactor(TURN_POSITION_IN_ROTATION);
        turnMotor.setVelocityConversionFactor(TURN_VELOCITY_IN_ROTATION);
        turnMotor.configureLoop(turnMotorPIDSettings, turnMotorFeedForwardSettings,
                TrapezoidProfileSettings.EMPTY_TRAPEZOID_PROFILE_SETTINGS);
    }

    @Override
    protected void configureAbsoluteEncoder() {
        MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs().
                withAbsoluteSensorDiscontinuityPoint(ABSOLUTE_POSITION_DISCONTINUITY_POINT)
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                .withMagnetOffset(absoluteEncoderOffsetDegrees / 360.0);
        cancoder.getConfigurator().apply(magnetSensorConfigs);
    }

    @Override
    protected Rotation2d getAbsoluteModuleAngle() {
        return Rotation2d.fromDegrees(cancoder.getAbsolutePosition().getValue().in(Units.Degree));
    }

    @Override
    public void simulationPeriodic() {

        talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        driveMotorSim.setInputVoltage(talonFXSim.getMotorVoltage());

        driveMotorSim.update(0.02);

        double rotorPosition = driveMotorSim.getAngularPositionRotations() * 6.12;
        double rotorVelocity = driveMotorSim.getAngularVelocityRPM() / 60.0 * 6.12;

        talonFXSim.setRawRotorPosition(rotorPosition);
        talonFXSim.setRotorVelocity(rotorVelocity);

        turnMotor.updateSimulation(0.02);
        double moduleAngleDegrees = turnMotor.getPosition();
        double moduleRotations = moduleAngleDegrees / 360.0;
        double offsetRotations = absoluteEncoderOffsetDegrees / 360.0;
        double rawPosition = moduleRotations - offsetRotations;
        cancoderSim.setRawPosition(rawPosition);
    }

    @Override
    public void configureDashboard() {
//        namespace.putNumber("drive velocity", this::getDrivingVelocity);
//        namespace.putNumber("turn velocity", () -> getTurnMotor().getVelocity());
//        namespace.putNumber("absolute encoder", () -> this.getAbsoluteModuleAngle().getDegrees());
//        namespace.putNumber("relative encoder", this::getRelativeModuleAngle);

//        namespace.putCommand("turn at 0.1", Commands.runEnd(
//                () -> turnMotor.set(0.1),
//                turnMotor::stopMotor,
//                this
//        ));

//        namespace.putCommand("drive at 0.1", Commands.runEnd(
//                () -> driveMotor.set(0.1),
//                driveMotor::stopMotor,
//                this
//        ));

//        Supplier<Double> targetAngle = namespace.addConstantDouble("target angle", 0);
//        namespace.putCommand("turn to angle", Commands.runEnd(() -> setTargetAngle(
//                        Rotation2d.fromDegrees(targetAngle.get()), true),
//                turnMotor::stopMotor,
//                this
//        ));

//        Supplier<Double> targetVelocity = namespace.addConstantDouble("target velocity", 0);
//        namespace.putCommand("drive at velocity", Commands.runEnd(() -> setTargetVelocity(
//                        targetVelocity.get(), Drivetrain.MAX_POSSIBLE_VELOCITY, true),
//                driveMotor::stopMotor,
//                this
//        ));
    }
}
