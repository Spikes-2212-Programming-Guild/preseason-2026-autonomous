package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.spikes2212.command.drivetrains.swerve.SwerveDrivetrain;
import com.spikes2212.command.drivetrains.swerve.SwerveModule;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.RobotMap;

public class Drivetrain extends SwerveDrivetrain implements Sendable {

    public static final double MAX_POSSIBLE_VELOCITY = 5.1;

    private static final String NAMESPACE_NAME = "swerve drivetrain";

    private static final CANBus CANIVORE = new CANBus("canivore");

    private static final double TRACK_WIDTH = 0.55;
    private static final double TRACK_LENGTH = 0.55;
    private static final int GYRO_OFFSET = 0;

    private static final boolean IS_SIMULATION = true;

    private final Pigeon2 gyro;

    private Pigeon2SimState gyroSim;
    private double simulatedYawDegrees;

    private static Drivetrain instance;

    private final Field2d field2d;

    private final SwerveDriveOdometry odometry;
    private ChassisSpeeds commandedSpeeds = new ChassisSpeeds();
    private Pose2d simPose = new Pose2d();

    public static Drivetrain getInstance() {
        if (instance == null) {
            instance = new Drivetrain(NAMESPACE_NAME, SwerveModuleHolder.getFrontLeft(),
                    SwerveModuleHolder.getFrontRight(), SwerveModuleHolder.getBackLeft(),
                    SwerveModuleHolder.getBackRight(), TRACK_WIDTH, TRACK_LENGTH, MAX_POSSIBLE_VELOCITY,
                    new Pigeon2(RobotMap.CAN.SWERVE_GYRO_PIGEON_2_ID, CANIVORE));
        }
        return instance;
    }

    private Drivetrain(String namespaceName, SwerveModule frontLeftModule, SwerveModule frontRightModule,
                       SwerveModule backLeftModule, SwerveModule backRightModule,
                       double drivetrainTrackWidth, double drivetrainTrackLength,
                       double maxPossibleVelocity, Pigeon2 gyro) {
        super(namespaceName, frontLeftModule, frontRightModule, backLeftModule, backRightModule,
                drivetrainTrackWidth, drivetrainTrackLength, maxPossibleVelocity);
        this.gyro = gyro;
        this.field2d = new Field2d();

        if (IS_SIMULATION) {
            gyroSim = gyro.getSimState();
            simulatedYawDegrees = gyro.getYaw().getValueAsDouble();
        }

        odometry = new SwerveDriveOdometry(getKinematics(), getAngle(), getSwerveModulePositions(), new Pose2d());
        configureDashboard();
    }

    @Override
    protected void setTargetModuleStates(SwerveModuleState[] targetModuleStates, boolean useVelocityPID) {
        commandedSpeeds = getKinematics().toChassisSpeeds(targetModuleStates);
        super.setTargetModuleStates(targetModuleStates, useVelocityPID);
    }

    @Override
    public void stop() {
        super.stop();
        commandedSpeeds = new ChassisSpeeds();
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    @Override
    public void resetAngleSensor() {
        gyro.setYaw(GYRO_OFFSET);
        if (IS_SIMULATION) simulatedYawDegrees = GYRO_OFFSET;
    }

    @Override
    public void periodic() {
        super.periodic();
        odometry.update(getAngle(), getSwerveModulePositions());
        field2d.setRobotPose(odometry.getPoseMeters());
    }

    @Override
    public void simulationPeriodic() {
        double dt = 0.02;
        simulatedYawDegrees += Math.toDegrees(commandedSpeeds.omegaRadiansPerSecond * dt);
        gyroSim.setRawYaw(simulatedYawDegrees);

        Rotation2d heading = Rotation2d.fromDegrees(simulatedYawDegrees);
        Translation2d delta = new Translation2d(
                commandedSpeeds.vxMetersPerSecond * dt,
                commandedSpeeds.vyMetersPerSecond * dt).
                rotateBy(heading);
        simPose = new Pose2d(simPose.getTranslation().plus(delta), heading);
        field2d.setRobotPose(simPose);
    }

    @Override
    public void drive(double xSpeed, double ySpeed, double rotationSpeed, boolean isFieldRelative,
                      double timeStep, boolean useVelocityPID) {
        super.drive(xSpeed, ySpeed, rotationSpeed, isFieldRelative,
                timeStep, useVelocityPID);
    }

    public void setGyro(double angle) {
        gyro.setYaw(angle);
        if (IS_SIMULATION) simulatedYawDegrees = angle;
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("gyro angle", () -> getAngle().getDegrees());
        namespace.putData("swerve drive", this);
        namespace.putData("field", field2d);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");
        addModuleSendable(builder, getBackLeftModule(), "Back Left");
        addModuleSendable(builder, getBackRightModule(), "Back Right");
        addModuleSendable(builder, getFrontLeftModule(), "Front Left");
        addModuleSendable(builder, getFrontRightModule(), "Front Right");
        builder.addDoubleProperty("Robot Angle", () -> getAngle().getDegrees(), null);
    }

    private void addModuleSendable(SendableBuilder builder, SwerveModule module, String name) {
        builder.addDoubleProperty(name + " Angle",
                () -> module.getModuleState().angle.getDegrees(), null);
        builder.addDoubleProperty(name + " Velocity",
                () -> module.getModuleState().speedMetersPerSecond, null);
    }
}
