// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.*;
import frc.robot.commands.CenterOnTagCmd;
import frc.robot.commands.RunIntakeCmd;
import frc.robot.commands.DriveCmds.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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
  private final VisionSubsystem vision = new VisionSubsystem();
  private final LauncherSubsystem launcher = new LauncherSubsystem();

  private final AbsoluteDrive AbsoluteDrive;
  private final FPSDrive FPSDrive;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController driveController = new XboxController(OperatorConstants.kDriverControllerPort);
  private final XboxController operatorController = new XboxController(OperatorConstants.kOperatorControllerPort);

  private final SendableChooser<Command> autChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //auton commands
    NamedCommands.registerCommand("print", new PrintCommand("Hello World"));
    NamedCommands.registerCommand("centerOnTag", new CenterOnTagCmd(vision, drivebase, 0, 1.2));
    NamedCommands.registerCommand("behindCenterOnTag", new CenterOnTagCmd(vision, drivebase, 0, .7));

    // Configure the trigger bindings
    configureBindings();

    AbsoluteDrive = new AbsoluteDrive(drivebase,
                                                          // Applies deadbands and inverts controls because joysticks
                                                          // are back-right positive while robot
                                                          // controls are front-left positive
                                                          () -> MathUtil.applyDeadband(-driveController.getLeftY(),
                                                                                       OperatorConstants.LEFT_Y_DEADBAND),
                                                          () -> MathUtil.applyDeadband(-driveController.getLeftX(),
                                                                                       OperatorConstants.LEFT_X_DEADBAND),
                                                          () -> -driveController.getRightX(),
                                                          () -> -driveController.getRightY());
    FPSDrive = new FPSDrive(drivebase,
                                                          // Applies deadbands and inverts controls because joysticks
                                                          // are back-right positive while robot
                                                          // controls are front-left positive
                                                          () -> MathUtil.applyDeadband(-driveController.getLeftY(),
                                                                                       OperatorConstants.LEFT_Y_DEADBAND),
                                                          () -> MathUtil.applyDeadband(-driveController.getLeftX(),
                                                                                       OperatorConstants.LEFT_X_DEADBAND),
                                                          () -> -driveController.getRightX(), () -> true);
    drivebase.setDefaultCommand(AbsoluteDrive);

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
    new JoystickButton(operatorController, 1).whileTrue(new RunIntakeCmd(launcher, -.9625));
    new JoystickButton(operatorController, 2).whileTrue(new RunIntakeCmd(launcher, .5));
    new JoystickButton(driveController, 1).whileTrue(new InstantCommand(drivebase::zeroGyro));
    // new JoystickButton(driveController, 7).whileTrue(new InstantCommand(drivebase.setDefaultCommand(FPSDrive)));
    // new JoystickButton(driveController, 8).whileTrue(new InstantCommand(() -> drivebase.setDefaultCommand(AbsoluteDrive)));
    new JoystickButton(driveController, 2).whileTrue(new CenterOnTagCmd(vision, drivebase, 0, 2));

    new JoystickButton(driveController, 5).onTrue(new InstantCommand(() -> NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(7)));
    new JoystickButton(driveController, 6).onTrue(new InstantCommand(() -> NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0)));
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
