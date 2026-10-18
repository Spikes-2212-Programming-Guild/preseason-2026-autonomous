// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.swerve.Drivetrain;
import frc.robot.swerve.commands.Drive;

public class Robot extends TimedRobot {

    Drivetrain drivetrain = Drivetrain.getInstance();
    RootNamespace namespace = new RootNamespace("robot");

    @Override
    public void robotInit() {
        namespace.putNumber("speed motors y swerve",() -> Drivetrain.getInstance().getSpeeds().vxMetersPerSecond);
        namespace.putNumber("speed motors x swerve",() -> Drivetrain.getInstance().getSpeeds().vyMetersPerSecond);
        namespace.putNumber("speed motors r swerve",() -> Drivetrain.getInstance().getSpeeds().omegaRadiansPerSecond);
//        namespace.putNumber("voltage", () -> );
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        namespace.update();
    }

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void disabledPeriodic() {

    }

    @Override

    public void autonomousInit() {

    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        OI oi = new OI();

        CommandScheduler.getInstance().setDefaultCommand(drivetrain, new Drive(oi::getLeftX, oi::getLeftY, oi::getRightX,
                true, false));
    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void simulationInit() {

    }

    @Override
    public void simulationPeriodic() {

    }
}
