package frc.robot.swerve;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.spikes2212.command.drivetrains.swerve.SwerveDrivetrain;
import com.spikes2212.command.drivetrains.swerve.SwerveModule;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import frc.robot.RobotMap;

public class Drivetrain extends SwerveDrivetrain {

    public static final double MAX_POSSIBLE_VELOCITY = 5.1;

    private static final String NAMESPACE_NAME = "swerve drivetrain";

    private static final CANBus CANIVORE = new CANBus("canivore");

    private static final double TRACK_WIDTH = 0.55;
    private static final double TRACK_LENGTH = 0.55;
    private static final int GYRO_OFFSET = 180;

    public static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.625, 4.03);
    public static final Translation2d RED_HUB_CENTER = new Translation2d(11.915, 4.03);

    private final StructArrayPublisher<SwerveModuleState> currentStates = NetworkTableInstance.getDefault()
            .getStructArrayTopic("current states", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> desiredStates = NetworkTableInstance.getDefault()
            .getStructArrayTopic("desired states", SwerveModuleState.struct).publish();

    private final Pigeon2 gyro;

    private final static double TRANSLATION_POSE_TOLERANCE = -1;
    private final static double TRANSLATION_VELOCITY_TOLERANCE = -1;
    private final static double ROTATION_TOLERANCE_IN_DEGREES = -1;
    private static final double ROTATION_VELOCITY_TOLERANCE = -1;

    private final SwerveDrivePoseEstimator odometry;

    private static Drivetrain instance;

    public static Drivetrain getInstance() {
        if (instance == null) {
            instance = new Drivetrain(NAMESPACE_NAME, SwerveModuleHolder.getFrontLeft(),
                    SwerveModuleHolder.getFrontRight(), SwerveModuleHolder.getBackLeft(),
                    SwerveModuleHolder.getBackRight(), TRACK_WIDTH, TRACK_LENGTH, MAX_POSSIBLE_VELOCITY,
                    new Pigeon2(RobotMap.CAN.SWERVE_GYRO_PIGEON_2_ID));
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

        setStructArrayStates(currentStates,
                new SwerveModuleState[]{
                        new SwerveModuleState(),
                        new SwerveModuleState(),
                        new SwerveModuleState(),
                        new SwerveModuleState()
                });
        setStructArrayStates(desiredStates,
                new SwerveModuleState[]{
                        new SwerveModuleState(),
                        new SwerveModuleState(),
                        new SwerveModuleState(),
                        new SwerveModuleState()
                });
        odometry = new SwerveDrivePoseEstimator(getKinematics(), getAngle(), getSwerveModulePositions(), new Pose2d());
        configureDashboard();
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    @Override
    public void resetAngleSensor() {
        gyro.setYaw(GYRO_OFFSET);
    }

    @Override
    public void drive(double xSpeed, double ySpeed, double rotationSpeed, boolean isFieldRelative,
                      double timeStep, boolean useVelocityPID) {
        super.drive(xSpeed, ySpeed, rotationSpeed, isFieldRelative,
                timeStep, useVelocityPID);

        setStructArrayStates(currentStates,
                new SwerveModuleState[]{
                        frontLeftModule.getModuleState(),
                        frontRightModule.getModuleState(),
                        backLeftModule.getModuleState(),
                        backRightModule.getModuleState()
                });

        setStructArrayStates(desiredStates, getDesiredStates(xSpeed, ySpeed, rotationSpeed, isFieldRelative,
                timeStep));
    }

    public void setStructArrayStates(StructArrayPublisher<SwerveModuleState> states,
                                     SwerveModuleState[] desiredStatesToSet) {
        states.set(desiredStatesToSet);
    }

    public void setGyro(double angle) {
        gyro.setYaw(angle);
    }

    public Pose2d getEstimatedPose() {
        return odometry.getEstimatedPosition();
    }

    public void resetPose(Pose2d newPose) {
        odometry.resetPose(newPose);
    }

    public void updateOdometry() {
        odometry.update(getAngle(), getSwerveModulePositions());
    }

    private boolean atAxis(double currentAxisPose, double targetAxisPose, double currentVelocity) {
        boolean atPose = MathUtil.isNear(targetAxisPose, currentAxisPose, TRANSLATION_POSE_TOLERANCE);
        boolean robotStill = Math.abs(currentVelocity) <= TRANSLATION_VELOCITY_TOLERANCE;
        return atPose && robotStill;
    }

    private boolean atRotation(Rotation2d rotation2d) {
        double error = rotation2d.minus(getAngle()).getDegrees();
        boolean isAtRotation = Math.abs(error) <= ROTATION_TOLERANCE_IN_DEGREES;
        boolean isRotationStill = Math.abs(getSpeeds().omegaRadiansPerSecond)
                <= ROTATION_VELOCITY_TOLERANCE;
        return isAtRotation && isRotationStill;
    }

    public boolean atPose(Pose2d pose2d) {
        boolean atXAxis = atAxis(getEstimatedPose().getX(), pose2d.getX(),
                getSpeeds().vxMetersPerSecond);
        boolean atYAxis = atAxis(getEstimatedPose().getY(), pose2d.getY(),
                getSpeeds().vyMetersPerSecond);
        return atXAxis && atYAxis && atRotation(pose2d.getRotation());
    }

    public Pose2d getEstimatedPoseByLatency(ChassisSpeeds relativeSpeeds, double latencySeconds) {
        double predictedXSpeed = relativeSpeeds.vxMetersPerSecond * latencySeconds;
        double predictedYSpeed = relativeSpeeds.vyMetersPerSecond * latencySeconds;
        Rotation2d predictedRotationSpeed =
                Rotation2d.fromRadians(relativeSpeeds.omegaRadiansPerSecond * latencySeconds);
        return getEstimatedPose().
                transformBy(new Transform2d(predictedXSpeed, predictedYSpeed, predictedRotationSpeed));
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("gyro", () -> this.getAngle().getDegrees());
        namespace.putNumber("x", () -> getEstimatedPose().getX());
        namespace.putNumber("y", () -> getEstimatedPose().getY());
        namespace.putRunnable("reset odometry", () ->
                odometry.resetPosition(getAngle(), getSwerveModulePositions(), new Pose2d()));
        namespace.putRunnable("reset gyro to 0", () -> gyro.setYaw(0));
    }

//    public void updateVision() {
//        LimelightHelpers.SetRobotOrientation(
//                "limelight", getAngle().getDegrees(), 0, 0, 0, 0, 0);
//
//        LimelightHelpers.PoseEstimate limelightMeasurement =
//                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
//
//        Pose2d pose = limelightMeasurement.pose;
//        double timestamp = limelightMeasurement.timestampSeconds;
//
//        if (pose == null) return;
//        addVisionMeasurement(pose, timestamp);
//    }

//    public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
//        odometry.addVisionMeasurement(visionPose, timestamp);
//    }
//
//    public double getDistanceFromHub() {
//        Pose2d robotPose = getEstimatedPose();
//
//        Translation2d hub = BLUE_HUB_CENTER;
//        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Red).equals(DriverStation.Alliance.Red)) {
//            hub = RED_HUB_CENTER;
//        }
//
//        Translation2d delta = hub.minus(robotPose.getTranslation());
//        return delta.getNorm();
//    }

//    public double getAngleFromHub() {
//        Pose2d robotPose = getEstimatedPose();
//
//        Translation2d hub = BLUE_HUB_CENTER;
//        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Red).equals(DriverStation.Alliance.Red)) {
//            hub = RED_HUB_CENTER;
//        }
//
//        Translation2d delta = hub.minus(robotPose.getTranslation());
//        return delta.getAngle().getDegrees();
//    }

    @Override
    public void periodic() {
        super.periodic();
        updateOdometry();
//        updateVision();
    }
}
