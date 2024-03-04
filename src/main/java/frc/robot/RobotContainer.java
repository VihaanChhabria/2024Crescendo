// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.*;
// import frc.robot.commands.AprilLauncherSetCmd;
import frc.robot.commands.CenterOnTagCmd;
import frc.robot.commands.LauncherAngleCmd;
import frc.robot.commands.LauncherManualAngleCmd;
import frc.robot.commands.RunBeltCmd;
// import frc.robot.commands.RunBeltCmd;
import frc.robot.commands.RunIntakeCmd;
import frc.robot.commands.RunLauncherCmd;
import frc.robot.commands.ToggleIntakeCmd;
import frc.robot.commands.DriveCmds.*;
//import frc.robot.commands.Pathfinding.PathFindToPosCmd;
//import frc.robot.commands.Pathfinding.StraightToPoseCmd;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = SwerveSubsystem.getInstance();
  private final VisionSubsystem vision = VisionSubsystem.getInstance();
  private final LauncherSubsystem launcher = LauncherSubsystem.getInstance();
  private final IntakeSubsystem intake = IntakeSubsystem.getInstance();
  private final ConveyorSubsystem conveyor = ConveyorSubsystem.getInstance();
  private final ShuffleBoardHandler ShuffleBoardHandler = new ShuffleBoardHandler();
  //private final PathfindingSubsystem pathing = new PathfindingSubsystem(0, 0);
  // private final AbsoluteDrive AbsoluteDrive;
  // private final FPSDrive FPSDrive;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  // private final XboxController driveController = new XboxController(OperatorConstants.kDriverControllerPort);
  // private final XboxController operatorController = new XboxController(OperatorConstants.kOperatorControllerPort);
  private final CommandXboxController driveController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  private final SendableChooser<Command> autChooser;

  //Pathfinding commands
  //Pose2d targetPose = new Pose2d(1.25,7,Rotation2d.fromDegrees(180));
  //PathConstraints constraints = new PathConstraints(2, 4, Units.degreesToRadians(540), Units.degreesToRadians(720));
  //private Command PathPlanningCommand = AutoBuilder.pathfindToPose(targetPose, constraints,0,0);

// private AbsoluteDrive AbsoluteDrive = new AbsoluteDrive(drivebase,
//                                                           // Applies deadbands and inverts controls because joysticks
//                                                           // are back-right positive while robot
//                                                           // controls are front-left positive
//                                                           () -> MathUtil.applyDeadband(-driveController.getLeftY(),
//                                                                                        OperatorConstants.LEFT_Y_DEADBAND),
//                                                           () -> MathUtil.applyDeadband(-driveController.getLeftX(),
//                                                                                        OperatorConstants.LEFT_X_DEADBAND),
//                                                           () -> -driveController.getRightX(),
//                                                           () -> -driveController.getRightY());
private FPSDrive FPSDrive = new FPSDrive(drivebase,
                                                          // Applies deadbands and inverts controls because joysticks
                                                          // are back-right positive while robot
                                                          // controls are front-left positive
                                                          () -> MathUtil.applyDeadband(-driveController.getLeftY(),
                                                                                       OperatorConstants.LEFT_Y_DEADBAND),
                                                          () -> MathUtil.applyDeadband(-driveController.getLeftX(),
                                                                                       OperatorConstants.LEFT_X_DEADBAND),
                                                          () -> -driveController.getRightX(), () -> true);

// private AbsoluteDrive CreepAbsoluteDrive = new AbsoluteDrive(drivebase,
//                                                           // Applies deadbands and inverts controls because joysticks
//                                                           // are back-right positive while robot
//                                                           // controls are front-left positive
//                                                           () -> MathUtil.applyDeadband(-driveController.getLeftY()/2,
//                                                                                        OperatorConstants.LEFT_Y_DEADBAND),
//                                                           () -> MathUtil.applyDeadband(-driveController.getLeftX()/2,
//                                                                                        OperatorConstants.LEFT_X_DEADBAND),
//                                                           () -> -driveController.getRightX(),
//                                                           () -> -driveController.getRightY());

private FPSDrive CreepFPSDrive = new FPSDrive(drivebase,
                                                          // Applies deadbands and inverts controls because joysticks
                                                          // are back-right positive while robot
                                                          // controls are front-left positive
                                                          () -> MathUtil.applyDeadband(-driveController.getLeftY()/2,
                                                                                       OperatorConstants.LEFT_Y_DEADBAND),
                                                          () -> MathUtil.applyDeadband(-driveController.getLeftX()/2,
                                                                                       OperatorConstants.LEFT_X_DEADBAND),
                                                          () -> -driveController.getRightX()/2, () -> true);
    
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);

    //auton commands
    NamedCommands.registerCommand("print", new PrintCommand("Hello World"));
    NamedCommands.registerCommand("centerOnTag", new CenterOnTagCmd(vision, drivebase, 0, 1.2));
    NamedCommands.registerCommand("behindCenterOnTag", new CenterOnTagCmd(vision, drivebase, 0, .7));
    NamedCommands.registerCommand("ToggleIntakeCmd", new ToggleIntakeCmd(intake));
    NamedCommands.registerCommand("timedBeltCmd", new RunBeltCmd(conveyor, -.75).withTimeout(2));
    NamedCommands.registerCommand("slowTimedBeltCmd", new RunBeltCmd(conveyor, -.5).withTimeout(2));
    NamedCommands.registerCommand("timedBeltCmdRev", new RunBeltCmd(conveyor, .7).withTimeout(.01));
    NamedCommands.registerCommand("timeIntakeCmd", new RunIntakeCmd(intake, -.9).withTimeout(2.5));
    NamedCommands.registerCommand("runLauncherCmd", new RunLauncherCmd(launcher, () -> .9625).withTimeout(2));
    NamedCommands.registerCommand("setLauncherTo60", new LauncherAngleCmd(launcher, ()->8.55, true));
    NamedCommands.registerCommand("AutoAngleLauncher", new LauncherAngleCmd(launcher, () -> -3.06*drivebase.triangulateDistanceToSpeaker()+10.2, true).withTimeout(1));
    NamedCommands.registerCommand("resetLauncher", new LauncherAngleCmd(launcher, () -> 0, true).withTimeout(1));
    NamedCommands.registerCommand("fixedDown", new LauncherAngleCmd(launcher,() ->  -1.0, true).withTimeout(1));
    // NamedCommands.registerCommand("AprilAim", new AprilLauncherSetCmd(vision, drivebase));

    // Configure the trigger bindings
    configureBindings();
    
    drivebase.setDefaultCommand(FPSDrive);

    autChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    //Driver bindings
    driveController.a().whileTrue(new InstantCommand(drivebase::zeroGyro));
    //driveController.x().toggleOnTrue(FPSDrive);
    //driveController.x().toggleOnTrue(FPSDrive);
    driveController.leftBumper().whileTrue(CreepFPSDrive);
    // driveController.leftTrigger().and(() -> CommandScheduler.getInstance().isScheduled(AbsoluteDrive)).whileTrue(CreepAbsoluteDrive);
    // driveController.leftTrigger().and(() -> CommandScheduler.getInstance().isScheduled(FPSDrive)).whileTrue(CreepFPSDrive);

    
    //driveController.leftBumper().and(() -> 

    //Operator bindings
    operatorController.pov(0).whileTrue(new LauncherManualAngleCmd(launcher, 1));
    operatorController.pov(180).whileTrue(new LauncherManualAngleCmd(launcher, -.5));

    operatorController.pov(270).onTrue(new LauncherAngleCmd(launcher, () -> 1.75, false));
    operatorController.pov(90).onTrue(new LauncherAngleCmd(launcher, () -> -1.3, false));

    operatorController.leftBumper().whileTrue(new RunLauncherCmd(launcher, () -> .9625)); //.9625
    operatorController.back().and(operatorController.leftBumper()).whileTrue(new RunLauncherCmd(launcher, ()->-.5));
  
    operatorController.rightBumper().whileTrue(new RunBeltCmd(conveyor, -.9));
    operatorController.back().and(operatorController.rightBumper()).whileTrue(new RunBeltCmd(conveyor, .5));

    operatorController.a().whileTrue(new RunIntakeCmd(intake, -.9));
    operatorController.back().and(operatorController.a()).whileTrue(new RunIntakeCmd(intake, .9));

    operatorController.x().onTrue(new ToggleIntakeCmd(intake)); 

    operatorController.b().whileTrue(new ParallelCommandGroup(new RunBeltCmd(conveyor, -.9), new RunIntakeCmd(intake, -.9)));
    operatorController.back().and(operatorController.b()).whileTrue(new ParallelCommandGroup(new RunBeltCmd(conveyor, .7), new RunIntakeCmd(intake, -.9)));
    // operatorController.b().whileTrue(new ParallelCommandGroup(new RunBeltCmd(conveyor, .9), new RunIntakeCmd(intake, -.9)));
    // operatorController.b().onTrue(new SequentialCommandGroup(new LauncherAngleCmd(launcher, () -> 1.6), new ToggleIntakeCmd(intake)));
    // operatorController.b().onFalse(new ToggleIntakeCmd(intake));

//     operatorController.y().whileTrue(new ParallelCommandGroup(new LauncherAngleCmd(launcher, () -> -2.24601*drivebase.triangulateDistanceToSpeaker()+9.8), new AbsoluteDrive(drivebase,
//                                                           // Applies deadbands and inverts controls because joysticks
//                                                           // are back-right positive while robot
//                                                           // controls are front-left positive
//                                                           () -> MathUtil.applyDeadband(-driveController.getLeftY()/2,
//                                                                                        OperatorConstants.LEFT_Y_DEADBAND),
//                                                           () -> MathUtil.applyDeadband(-driveController.getLeftX()/2,
//                                                                                        OperatorConstants.LEFT_X_DEADBAND),
//                                                           () -> -(drivebase.triangulateDistanceToSpeaker()*Math.sin(drivebase.getAngleToSpeaker())),
//                                                           () -> -(drivebase.triangulateDistanceToSpeaker()*Math.cos(drivebase.getAngleToSpeaker())))
// ));

// //10.9895
    operatorController.y().whileTrue(new LauncherAngleCmd(launcher, () -> -3.06*drivebase.triangulateDistanceToSpeaker()+10.2, false));
    operatorController.start().onTrue(new InstantCommand(launcher::resetEncoder));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(Object auton) {
    // An example command will be run in autonomous
    return autChooser.getSelected();
  }

  public void setDriveMode(){
    return;
  }

  public void setMotorBrake(boolean brake){
    drivebase.setMotorBrake(brake);
  }
}
