// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.*;
// import frc.robot.commands.AprilLauncherSetCmd;
// import frc.robot.commands.CenterOnTagCmd;
import frc.robot.commands.LauncherAngleCmd;
import frc.robot.commands.LauncherManualAngleCmd;
import frc.robot.commands.LuanchCmd;
import frc.robot.commands.RunBeltCmd;
// import frc.robot.commands.RunBeltCmd;
import frc.robot.commands.RunIntakeCmd;
import frc.robot.commands.RunLauncherCmd;
import frc.robot.commands.ToggleAmpCmd;
import frc.robot.commands.ToggleIntakeCmd;
import frc.robot.commands.DriveCmds.*;
//import frc.robot.commands.Pathfinding.PathFindToPosCmd;
//import frc.robot.commands.Pathfinding.StraightToPoseCmd;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
// import frc.robot.commands.AprilLauncherSetCmd;
// import frc.robot.commands.CenterOnTagCmd;
import frc.robot.commands.LauncherAngleCmd;
import frc.robot.commands.LuanchCmd;
import frc.robot.commands.RunBeltCmd;
// import frc.robot.commands.RunBeltCmd;
import frc.robot.commands.RunIntakeCmd;
import frc.robot.commands.RunLauncherCmd;
import frc.robot.commands.ToggleAmpCmd;
import frc.robot.commands.ToggleIntakeCmd;
//import frc.robot.commands.Pathfinding.PathFindToPosCmd;
//import frc.robot.commands.Pathfinding.StraightToPoseCmd;
import frc.robot.commands.DriveCmds.FPSDrive;
import frc.robot.subsystems.AmpSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem drivebase = SwerveSubsystem.getInstance();
    private final LauncherSubsystem launcher = LauncherSubsystem.getInstance();
    private final IntakeSubsystem intake = IntakeSubsystem.getInstance();
    private final ConveyorSubsystem conveyor = ConveyorSubsystem.getInstance();
    private final AmpSubsystem amp = AmpSubsystem.getInstance();

    private final CommandXboxController driveController = new CommandXboxController(
            OperatorConstants.kDriverControllerPort);
    private final CommandXboxController operatorController = new CommandXboxController(
            OperatorConstants.kOperatorControllerPort);

    private final SendableChooser<Command> autoChooser;

    private FPSDrive FPSDrive = new FPSDrive(drivebase,
            // Applies deadbands and inverts controls because joysticks
            // are back-right positive while robot
            // controls are front-left positive
            () -> MathUtil.applyDeadband(-driveController.getLeftY(),
                    OperatorConstants.LEFT_Y_DEADBAND),
            () -> MathUtil.applyDeadband(-driveController.getLeftX(),
                    OperatorConstants.LEFT_X_DEADBAND),
            () -> -driveController.getRightX(), () -> true);

    private FPSDrive CreepFPSDrive = new FPSDrive(drivebase,
            // Applies deadbands and inverts controls because joysticks
            // are back-right positive while robot
            // controls are front-left positive
            () -> MathUtil.applyDeadband(-driveController.getLeftY() / 2,
                    OperatorConstants.LEFT_Y_DEADBAND),
            () -> MathUtil.applyDeadband(-driveController.getLeftX() / 2,
                    OperatorConstants.LEFT_X_DEADBAND),
            () -> -driveController.getRightX() / 2, () -> true);

    private final Trigger DriverA = driveController.a();

    private final Trigger DriverB = driveController.b();

    private final Trigger DriverY = driveController.y();

    private final Trigger DriverX = driveController.x();

    private final Trigger DriverBack = driveController.back();

    private final Trigger DriverStart = driveController.start();

    private final Trigger DriverLeftBumper = driveController.leftBumper();

    private final Trigger DriverRightBumper = driveController.rightBumper();

    private final Trigger OperatorA = operatorController.a();

    private final Trigger OperatorB = operatorController.b();

    private final Trigger OperatorY = operatorController.y();

    private final Trigger OperatorX = operatorController.x();

    private final Trigger OperatorBack = operatorController.back();

    private final Trigger OperatorStart = operatorController.start();

    private final Trigger OperatorLeftBumper = operatorController.leftBumper();

    private final Trigger OperatorRightBumper = operatorController.rightBumper();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);

        // Configure the trigger bindings
        configureAutonCommands();
        configureBindings();

        drivebase.setDefaultCommand(FPSDrive);

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        // Driver bindings
        DriverA.onTrue(new InstantCommand(drivebase::zeroGyro));
        DriverLeftBumper.whileTrue(CreepFPSDrive);

        OperatorLeftBumper.whileTrue(new RunLauncherCmd(launcher, () -> 1000)); // .9625
        OperatorBack.and(OperatorLeftBumper).whileTrue(new RunLauncherCmd(launcher, () -> -100));
        OperatorRightBumper.whileTrue(new RunBeltCmd(conveyor, -.9));
        OperatorBack.and(OperatorRightBumper).whileTrue(new RunBeltCmd(conveyor, .8));
        OperatorA.whileTrue(new RunIntakeCmd(intake, -.9));
        OperatorBack.and(OperatorA).whileTrue(new RunIntakeCmd(intake, .9));
        OperatorX.onTrue(new ToggleIntakeCmd(intake));
        OperatorB
                .whileTrue(new ParallelCommandGroup(new RunBeltCmd(conveyor, -.9), new RunIntakeCmd(intake, -.9)));
        OperatorBack.and(OperatorB)
                .whileTrue(new ParallelCommandGroup(new RunBeltCmd(conveyor, .9625), new RunIntakeCmd(intake, -.9)));
        OperatorY.onTrue(new ToggleAmpCmd(amp, () -> .7));
        OperatorStart.whileTrue(new RunLauncherCmd(launcher, () -> .3, false));
    }

    public void configureAutonCommands() {
        // NamedCommands.registerCommand("print", new PrintCommand("Hello World"));

        // Intake/Belt Commands
        NamedCommands.registerCommand("ToggleIntakeCmd", new ToggleIntakeCmd(intake).withTimeout(0.01));
        NamedCommands.registerCommand("timedBeltCmd", new RunBeltCmd(conveyor, -.65).withTimeout(1));
        NamedCommands.registerCommand("slowTimedBeltCmd", new RunBeltCmd(conveyor, -.5).withTimeout(1));
        NamedCommands.registerCommand("timedBeltCmdRev", new RunBeltCmd(conveyor, 0).withTimeout(.001));
        NamedCommands.registerCommand("longRangeIntakeCmd", new RunIntakeCmd(intake, -.9375).withTimeout(1.725));
        NamedCommands.registerCommand("timeIntakeCmd", new RunIntakeCmd(intake, -.94).withTimeout(1.3));

        // Launcher Commands
        NamedCommands.registerCommand("launch",
                new LuanchCmd(intake, launcher, conveyor,
                        () -> -3.06 * drivebase.triangulateDistanceToSpeaker() + 10.2));
        NamedCommands.registerCommand("runLauncherCmd", new RunLauncherCmd(launcher, () -> 1000).withTimeout(.75));
        NamedCommands.registerCommand("setLauncherTo60", new LauncherAngleCmd(launcher, () -> 8.55, true));
        NamedCommands.registerCommand("AutoAngleLauncher",
                new LauncherAngleCmd(launcher, () -> -3.06 * drivebase.triangulateDistanceToSpeaker() + 10.2, false)
                        .withTimeout(.75));
        NamedCommands.registerCommand("resetLauncher", new LauncherAngleCmd(launcher, () -> 1.5, true).withTimeout(.5));
        NamedCommands.registerCommand("fixedDown", new LauncherAngleCmd(launcher, () -> 2, true).withTimeout(.5));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand(Object auton) {
        // An example command will be run in autonomous
        return autoChooser.getSelected();
    }

    public void setDriveMode() {
        return;
    }

    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }
}
