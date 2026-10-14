package frc.robot.swerve.commands;

import com.spikes2212.command.drivetrains.swerve.SwerveDrivetrain;
import com.spikes2212.command.drivetrains.swerve.commands.RotateModulesWithPID;
import edu.wpi.first.math.geometry.Rotation2d;

public class ModuleRotateWithPID extends RotateModulesWithPID {

    public ModuleRotateWithPID(SwerveDrivetrain drivetrain, Rotation2d frontLeftDesiredAngle,
                               Rotation2d frontRightDesiredAngle, Rotation2d backLeftDesiredAngle,
                               Rotation2d backRightDesiredAngle) {
        super(drivetrain, frontLeftDesiredAngle, frontRightDesiredAngle, backLeftDesiredAngle, backRightDesiredAngle);
    }

    public ModuleRotateWithPID(SwerveDrivetrain drivetrain, double frontLeftDesiredAngle,
                               double frontRightDesiredAngle, double backLeftDesiredAngle,
                               double backRightDesiredAngle) {
        super(drivetrain, frontLeftDesiredAngle, frontRightDesiredAngle, backLeftDesiredAngle, backRightDesiredAngle);
    }
}
