// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.align.alignUtils.Target;
import frc.robot.align.preciseAligning.ClimbAlign;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final Core m_core;

  private final Target testTarget = new Target(31, new Transform3d(1.575, 0.0, 0, new Rotation3d(0, 0, 0)));


  public Robot() {
    m_core = new Core();
  }

  @Override
  public void robotPeriodic() {
     CommandScheduler.getInstance().run(); 

    m_core.drivetrain.passGlobalEstimates(m_core.vision.getGlobalFieldPoses());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_core.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      //CommandScheduler.getInstance().schedule(new SequentialCommandGroup(m_autonomousCommand, climbAlign));
      m_core.drivetrain.resetPose((((PathPlannerAuto) m_core.getAutonomousCommand()).getStartingPose()));
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }

  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    CommandScheduler.getInstance().schedule(new InstantCommand(() -> m_core.intake.stop()));
    CommandScheduler.getInstance().schedule(new InstantCommand(() -> m_core.shooter.stopAll()));
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}