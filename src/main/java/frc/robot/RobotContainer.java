/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import static edu.wpi.first.wpilibj.XboxController.Axis.*;
import static edu.wpi.first.wpilibj.XboxController.Button.*;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.shooter.ChangePosition;
import frc.robot.shooter.Conveyor;
import frc.robot.shooter.Plucker;
import frc.robot.vision.AimTarget;
import frc.robot.vision.Limelight;
import frc.robot.climber.Lift;
import frc.robot.drive.Gears;
import frc.robot.drive.RevDrivetrain;
import frc.robot.shooter.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.Arrays;

import static frc.robot.Constants.*;
import static frc.robot.Gains.Ramsete.*;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Drive Controller
  private XboxController xbox = new XboxController(kXboxPort);

  // Drive Subsystem
  private final RevDrivetrain rDrive = new RevDrivetrain();

  // Limelight Subsystem
  private final Limelight limelight = new Limelight();

  private ChangePosition goalMover = new ChangePosition();

  private final Lift lift = new Lift();

  private final Shooter shooter = new Shooter(goalMover, limelight);

  private final Conveyor conveyor = new Conveyor(goalMover);

  private final Plucker plucker = new Plucker(goalMover);

  private final Gears gears = new Gears();

  // Update PID values
  private final Update update = new Update(shooter, plucker);

  //  --- Default Commands ---

  // Drive with Controller 
  private Command manualDrive = new RunCommand(
    () -> rDrive.getDifferentialDrive().tankDrive(
      rDrive.deadband(drivePercentLimit * xbox.getRawAxis(kLeftY.value), percentDeadband), 
      rDrive.deadband(drivePercentLimit * xbox.getRawAxis(kRightY.value), percentDeadband),
      false
      ),
    rDrive
  );

  private Command moveArmOneAxis = new RunCommand(
    () -> lift.moveOneAxis(xbox.getRawAxis(kLeftTrigger.value)), lift);
  
  private Command moveArm = new RunCommand(
    () -> lift.move(xbox.getRawAxis(kRightTrigger.value) - xbox.getRawAxis(kLeftTrigger.value)), lift);

  // --- Command Groups ---

  private SequentialCommandGroup waitAndFeed = new SequentialCommandGroup(
    // When in collecting pose, the time delay is not needed, so it is interrupted 
    new WaitCommand(shooterRampUpTime).withInterrupt(goalMover::isCollectingPose),
    new InstantCommand(() -> plucker.setSpeed(), plucker), 
    new InstantCommand(() -> conveyor.setSpeed(), conveyor));

  private SequentialCommandGroup waitUntilVelocity = new SequentialCommandGroup(
    new WaitUntilCommand(() -> shooter.atSpeed()),
    new InstantCommand(() -> plucker.setSpeed(), plucker), 
    new InstantCommand(() -> conveyor.setSpeed(), conveyor)
  );

  private SequentialCommandGroup stopFeeders = new SequentialCommandGroup(
    new InstantCommand(() -> plucker.stop(), plucker),
    new InstantCommand(() -> conveyor.stop(), conveyor));

  // Autonomous 
  private SequentialCommandGroup shootThenGo = new SequentialCommandGroup(
    new InstantCommand(() -> goalMover.collectPose(), goalMover),
    new WaitCommand(.75),
    new InstantCommand(() -> goalMover.shootPose(), goalMover),
    new InstantCommand(() -> shooter.setSpeedSpark(), shooter),
    new WaitCommand(shooterRampUpTime).withInterrupt(goalMover::isCollectingPose),
    new InstantCommand(() -> plucker.setSpeed(), plucker), 
    new InstantCommand(() -> conveyor.setSpeed(), conveyor),
    new WaitCommand(2 + shooterRampUpTime),
    new InstantCommand(() -> plucker.stop(), plucker),
    new InstantCommand(() -> conveyor.stop(), conveyor),
    new InstantCommand(() -> shooter.stop(), shooter),
    new RunCommand(() -> rDrive.getDifferentialDrive().tankDrive(0.4, 0.4), rDrive).withTimeout(2)
  );

  /*private RamseteCommand rBase = new RamseteCommand(
    getMovingTrajectory(), 
    rDrive::getPose, 
    new RamseteController(kBeta, kZeta), 
    rDrive.getFeedforward(), 
    rDrive.getKinematics(), 
    rDrive::getSpeeds, 
    rDrive.getLeftDrivePID(), 
    rDrive.getRightDrivePID(), 
    rDrive::setOutputVolts, 
    rDrive);*/

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() { 
    // Configure the button bindings
    configureButtonBindings();

    rDrive.setDefaultCommand(manualDrive);
    lift.setDefaultCommand(moveArm);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
    // Switch position between shooting and intake
    new JoystickButton(xbox, kA.value)
    .whenPressed(new InstantCommand(() -> goalMover.swapHeight(), goalMover));

    // Shoot or intake with voltage, aiming for low goal
    new JoystickButton(xbox, kBumperLeft.value)
    .whenPressed(new InstantCommand(() -> shooter.toggleSpeedVolts(), shooter))
    .whenPressed(new InstantCommand(() -> conveyor.toggleSpeed(), shooter))
    .whenPressed(new InstantCommand(() -> plucker.toggleSpeed(), plucker));
    
    // Shoot or intake with set velocity, specifically for high goal
    new JoystickButton(xbox, kB.value)
    .whenPressed(new InstantCommand(() -> plucker.toggleSpeed(), plucker));
    
    // Toggles high shooting
    new JoystickButton(xbox, kY.value)
    .whenPressed(new InstantCommand(() -> shooter.toggleSpeedSpark()))
    .whenPressed(new ConditionalCommand(waitUntilVelocity, stopFeeders, shooter::isEngaged));

    // Vision correction
    new JoystickButton(xbox, kX.value)
    .whileHeld(new AimTarget(limelight, rDrive));

    // Switch Gears
    new JoystickButton(xbox, kBumperRight.value)
    .whenPressed(() -> gears.switchGears(), gears);

    new JoystickButton(xbox, kStart.value)
    .whenPressed(new InstantCommand(() -> shooter.zoneSwitch(), shooter))
    .whenPressed(new PrintCommand("Pressed"));
    
  }

  public void init(){
    limelight.driverMode();
    limelight.lightOff();
    limelight.PiPSecondaryStream();

    shooter.stop();
    plucker.stop();
    conveyor.stop();
  } 

  public void periodic() {

    SmartDashboard.putString("Shooter Postion:", shooter.selector.positionName);
    SmartDashboard.putNumber("Set RPM", shooter.shooterRPM);

    update.periodic();
  }

  /*
  private Trajectory getMovingTrajectory() {
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      Arrays.asList(Update.getStartingPose(), new Pose2d(1.0, 0, new Rotation2d()),
        new Pose2d(2.3, 1.2, Rotation2d.fromDegrees(90.0))), 
      new TrajectoryConfig(MaxSafeVelocityMeters, MaxSafeAccelerationMeters)
    );
    
    return trajectory;
  }
  */

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return shootThenGo;
  }
}
