// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.swerve.Drive;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.subsystems.swerve.SwerveModuleRebuilt;

public class Robot extends TimedRobot {

    private Drivetrain drivetrain;

    @Override
    public void robotInit() {
        CommandScheduler.getInstance().cancelAll();
        drivetrain = Drivetrain.getInstance();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SwerveModuleRebuilt.updateModuleTuner();
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
        drivetrain.resetFieldRelativity();
        drivetrain.resetRelativeEncoders();

        OI oi = new OI();
        double ySpeed = 5;
        double xSpeed = 5;
        double rotationSpeed = 5;
        drivetrain.setDefaultCommand(new Drive(
                () -> squareInputs(oi.getLeftX()) * xSpeed,
                () -> squareInputs(oi.getLeftY()) * -ySpeed,
                () -> squareInputs(oi.getRightX()) * -rotationSpeed,
                true, true));
    }

    private static double squareInputs(double input) {
        if (Math.abs(input) < 0.1) return 0;
        return Math.signum(input) * (input * input);
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
