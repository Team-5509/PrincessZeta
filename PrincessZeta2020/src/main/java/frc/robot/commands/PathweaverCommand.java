package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;

class PathweaverCommand extends RamseteCommand {

    Path trajectoryPath;
    public PathweaverCommand(String pathFileName) throws IOException {
        super(TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve(pathFileName)),
        (Supplier<Pose2d>)Robot.driveTrain::getPose,
        new RamseteController(Robot.kRamseteB, Robot.kRamseteZeta),
        new SimpleMotorFeedforward(Robot.ksVolts,
                                   Robot.kvVoltsSecondsPerMeter,
                                   Robot.kAVoltsSecondsSquaredPerMeter),
        Robot.kDriveKinematics,
        (Supplier<DifferentialDriveWheelSpeeds>)Robot.driveTrain::getWheelSpeeds,
        new PIDController(Robot.kPDriveVel, 0, 0),
        new PIDController(Robot.kPDriveVel, 0, 0),
        (BiConsumer<Double, Double>)Robot.driveTrain::tankDriveVolts
        );
        /*
        super​(trajectoryPath,
        Robot.driveTrain.getPose(),
        new RamseteController(Robot.kRamseteB, Robot.kRamseteZeta),
        new SimpleMotorFeedforward(Robot.ksVolts,
                                   Robot.kvVoltsSecondsPerMeter,
                                   Robot.kAVoltsSecondsSquaredPerMeter),
        Robot.kDriveKinematics,
        Robot.driveTrain.getWheelSpeeds(),
        new PIDController(Robot.kPDriveVel, 0, 0),
        new PIDController(Robot.kPDriveVel, 0, 0),
        Robot.driveTrain::tankDriveVolts,
        Robot.driveTrain
            );
            */
            /*
        final DriveTrain driveTrain = Robot.driveTrain;
        final Supplier<Pose2d> pose = driveTrain::getPose;
        final Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds = driveTrain::getWheelSpeeds;
        final BiConsumer<Double, Double> outputVolts = driveTrain::tankDriveVolts;

        final Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(pathFileName);
        final Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        */
/*
        super​(trajectory,
        pose,
        new RamseteController(Robot.kRamseteB, Robot.kRamseteZeta),
        new SimpleMotorFeedforward(Robot.ksVolts,
        Robot.kvVoltsSecondsPerMeter,
        Robot.kAVoltsSecondsSquaredPerMeter),
        Robot.kDriveKinematics,
        wheelSpeeds,
        new PIDController(Robot.kPDriveVel, 0, 0),
        new PIDController(Robot.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        outputVolts, null);
        */

        /*
        super(trajectory,
        pose,
        controller,
        feedforward,
        kinematics,
        wheelSpeeds,
        leftController,
        rightController,
        outputVolts,
        null);
            */
        
        /*
        trajectory,
                Robot.driveTrain.getPose(),
                new RamseteController(Robot.kRamseteB, Robot.kRamseteZeta),
                new SimpleMotorFeedforward(Robot.ksVolts,
                                           Robot.kvVoltsSecondsPerMeter,
                                           Robot.kAVoltsSecondsSquaredPerMeter),
                Robot.kDriveKinematics,
                Robot.driveTrain.getWheelSpeeds(),
                new PIDController(Robot.kPDriveVel, 0, 0),
                new PIDController(Robot.kPDriveVel, 0, 0),
                Robot.driveTrain::tankDriveVolts,
                Robot.driveTrain
        );
        */


    }
}
