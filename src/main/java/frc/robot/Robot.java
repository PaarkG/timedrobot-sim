// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class Robot extends TimedRobot {
  private final XboxController controller = new XboxController(0);

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  private final SlewRateLimiter speedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

  private final Drivetrain drive = new Drivetrain();
  private final RamseteController ramsete = new RamseteController();
  private final Timer timer = new Timer();
  private Trajectory trajectory;
  private ArrayList<Trajectory> trajectories = new ArrayList<Trajectory>();

  private final String path = "pathplanner/generatedJSON/";
  private int currentTrajectoryIndex = 0;

  @Override
  public void robotInit() {
    // m_trajectory =
    //     TrajectoryGenerator.generateTrajectory(
    //         new Pose2d(2, 2, new Rotation2d()),
    //         List.of(),
    //         new Pose2d(8, 4, new Rotation2d(180)),
    //         new TrajectoryConfig(2, 2));

    try {
      trajectory = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("pathplanner/generatedJSON/ToCube.wpilib.json"));
      trajectories.add(trajectory);
      trajectory = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("pathplanner/generatedJSON/ScoreMidCube.wpilib.json"));
      trajectories.add(trajectory);
      trajectory = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("pathplanner/generatedJSON/ToSecondCube.wpilib.json"));
      trajectories.add(trajectory);
      trajectory = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("pathplanner/generatedJSON/ScoreLowCube.wpilib.json"));
      trajectories.add(trajectory);
      trajectory = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("pathplanner/generatedJSON/Balance.wpilib.json"));
      trajectories.add(trajectory);
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  @Override
  public void robotPeriodic() {
    drive.periodic();
  }

  @Override
  public void autonomousInit() {
    timer.reset();
    timer.start();
    drive.resetOdometry(trajectories.get(0).getInitialPose());
  }

  @Override
  public void autonomousPeriodic() {
    double elapsed = timer.get();
    if(elapsed < trajectories.get(currentTrajectoryIndex).getTotalTimeSeconds()) {
      Trajectory.State reference = trajectories.get(currentTrajectoryIndex).sample(elapsed);
      ChassisSpeeds speeds = ramsete.calculate(drive.getPose(), reference);
      drive.drive(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
    } else if(currentTrajectoryIndex < trajectories.size() - 1) {
      currentTrajectoryIndex++;
      timer.reset();
      timer.start();
    }
  }

  @Override
  public void teleopPeriodic() {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    double xSpeed = -speedLimiter.calculate(controller.getLeftY()) * Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    double rot = -rotLimiter.calculate(controller.getRightX()) * Drivetrain.kMaxAngularSpeed;
    drive.drive(xSpeed, rot);
  }

  @Override
  public void simulationPeriodic() {
    drive.simulationPeriodic();
  }
}
