package frc.robot.swerve;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.spikes2212.command.drivetrains.swerve.SwerveModule;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.util.smartmotorcontrollers.SparkWrapper;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;

public class SwerveModuleRebuilt extends SwerveModule {

    private static final double MIN_SPEED_LIMIT = 0.1;
    private static final double DRIVE_GEAR_RATIO = 1 / 6.12;
    private static final double TURN_GEAR_RATIO = (double) 7 / 150;
    private static final double WHEEL_DIAMETER_METERS = 0.1016;
    private static final int DEGREES_IN_ROTATION = 360;
    private static final int SECONDS_IN_MINUTE = 60;
    private static final int ABSOLUTE_POSITION_DISCONTINUITY_POINT = 1;

    private static final int DRIVE_SUPPLY_CURRENT_LIMIT = 40;
    private static final int DRIVE_STATOR_CURRENT_LIMIT = 80;

    private static final int TURN_SMART_CURRENT_LIMIT = 60;

    private static final double DRIVE_MOTOR_ROTATION_TO_WHEEL_ROTATIONS =
            DRIVE_GEAR_RATIO * WHEEL_DIAMETER_METERS * Math.PI;
    private static final double TURN_VELOCITY_IN_ROTATION =
            (TURN_GEAR_RATIO * DEGREES_IN_ROTATION) / SECONDS_IN_MINUTE;
    private static final double TURN_POSITION_IN_ROTATION =
            TURN_GEAR_RATIO * DEGREES_IN_ROTATION;

    private final TalonFXWrapper driveMotor;
    private final SparkWrapper turnMotor;
    private final CANcoder cancoder;

    public SwerveModuleRebuilt(String namespaceName, TalonFXWrapper driveMotor, SparkWrapper turnMotor,
                               boolean driveMotorInverted, boolean turnMotorInverted,
                               double absoluteEncoderOffset, PIDSettings driveMotorPIDSettings,
                               PIDSettings turnMotorPIDSettings, FeedForwardSettings driveMotorFeedForwardSettings,
                               FeedForwardSettings turnMotorFeedForwardSettings, CANcoder cancoder) {
        super(namespaceName, driveMotor, turnMotor, driveMotorInverted, turnMotorInverted, absoluteEncoderOffset,
                driveMotorPIDSettings, turnMotorPIDSettings, driveMotorFeedForwardSettings,
                turnMotorFeedForwardSettings, MIN_SPEED_LIMIT);
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.cancoder = cancoder;
        driveMotor.restoreFactoryDefaults();
        turnMotor.restoreFactoryDefaults();

        driveMotor.setInverted(driveMotorInverted);
        turnMotor.setInverted(!turnMotorInverted);

        configureTurnController();
        configureDriveController();
        configureAbsoluteEncoder();
        setCurrents();
        configureDashboard();
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
                withAbsoluteSensorDiscontinuityPoint(ABSOLUTE_POSITION_DISCONTINUITY_POINT).
                withSensorDirection(turnMotorInverted ? SensorDirectionValue.Clockwise_Positive :
                        SensorDirectionValue.CounterClockwise_Positive).withMagnetOffset(absoluteEncoderOffset);
        cancoder.getConfigurator().apply(magnetSensorConfigs);
    }

    @Override
    protected Rotation2d getAbsoluteModuleAngle() {
        return Rotation2d.fromDegrees(cancoder.getAbsolutePosition().getValue().in(Units.Degree));
    }

    public void setCurrents() {
        driveMotor.getConfigurator().apply(new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(DRIVE_SUPPLY_CURRENT_LIMIT)
                .withStatorCurrentLimit(DRIVE_STATOR_CURRENT_LIMIT)
        );
        turnMotor.applyConfiguration(turnMotor.getSparkConfiguration()
                .smartCurrentLimit(TURN_SMART_CURRENT_LIMIT)
                .voltageCompensation(12)
        );
    }

    @Override
    public void configureDashboard() {

//        namespace.putNumber("absolute encoder", () -> this.getAbsoluteModuleAngle().getDegrees());
//        namespace.putNumber("relative angle", this::getRelativeModuleAngle);
//        namespace.putNumber("current drive velocity", driveMotor::getVelocity);
//        namespace.putNumber("current turn position", turnMotor::getPosition);
//        namespace.putNumber("current turn current", turnMotor::getCurrent);
//        namespace.putNumber("voltage drive", driveMotor::getVoltage);

//        namespace.putCommand("turn pid", new FunctionalCommand(() -> {},() -> turnMotor.pidSet(UnifiedControlMode.POSITION,
//                0, namespace.addPIDNamespace("try"), namespace.addFeedForwardNamespace(
//                        "try", FeedForwardController.ControlMode.LINEAR_POSITION), true),
//                b -> stop(), () -> false));

//        namespace.putCommand("set angle", new FunctionalCommand(() -> {
//        },
//                () -> setTargetState(
//                        new SwerveModuleState(0, Rotation2d.fromDegrees(
//                                namespace.addConstantDouble("target angle", 0).get())),
//                        Drivetrain.MAX_POSSIBLE_VELOCITY, false
//                ), b -> stop(), () -> false));
//
//        namespace.putCommand("set velocity", new FunctionalCommand(() -> {
//        },
//                () -> setTargetState(
//                        new SwerveModuleState(namespace.addConstantDouble("target velocity", 1).get(),
//                                Rotation2d.fromDegrees(0)), Drivetrain.MAX_POSSIBLE_VELOCITY, true),
//                b -> stop(), () -> false));

//        namespace.putCommand("set drive speed", new FunctionalCommand(() -> {
//        },
//                () -> driveMotor.pidSet(UnifiedControlMode.VELOCITY,
//                        namespace.addConstantDouble("target drive speed", 1).get(), driveMotorPIDSettings,
//                        driveMotorFeedForwardSettings, true),
//                b -> stop(), () -> false));
//
//        Supplier<Double> driveSpeed = namespace.addConstantDouble("drive speed", 0);
//        namespace.putCommand("drive at speed", new RunCommand(() -> driveMotor.set(driveSpeed.get())) {
//            @Override
//            public void end(boolean interrupted) {
//                driveMotor.stopMotor();
//            }
//        });
//
//        Supplier<Double> turnSpeed = namespace.addConstantDouble("turn speed", 0);
//        namespace.putCommand("turn at speed", new RunCommand(() -> turnMotor.set(turnSpeed.get())) {
//            @Override
//            public void end(boolean interrupted) {
//                turnMotor.stopMotor();
//            }
//        });
//
//        Supplier<Double> t = namespace.addConstantDouble("target angle", 0);
//        namespace.putCommand("turn pid", new FunctionalCommand(() -> {},
//                () -> setTargetAngle(Rotation2d.fromDegrees(t.get())), b -> stop(), () -> false));

//        Supplier<Double> k = namespace.addConstantDouble("target speed", 0);
//        namespace.putCommand("drive pid", new FunctionalCommand(() -> {},
//                () -> setTargetState(new SwerveModuleState(k.get(), Rotation2d.fromDegrees(0)),
//                        Drivetrain.MAX_POSSIBLE_VELOCITY, true),
//                b -> stop(), () -> false));

    }
}
