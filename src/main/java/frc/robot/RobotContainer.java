// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.sim.JoystickInputsRecord;
import frc.robot.sim.SimWrapper;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    /** Stores the starting pose of the currently selected auto */
    private Pose2d selectedAutoStartingPose = new Pose2d();

    /** Simulation wrapper - null when not in simulation */
    public final SimWrapper m_simWrapper;

    public RobotContainer() {
       autoChooser = AutoBuilder.buildAutoChooser("Tests");
        initAutoSelector();

        configureBindings();

        // $TODO - Wrapper for sim features
        if (Robot.isSimulation()) {
            m_simWrapper = new SimWrapper(
                drivetrain,
                this::resetRobotPose,
                drivetrain::addVisionMeasurement);
        }
        else {
            m_simWrapper = null;
        }

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
    }

    private Command getJoystickCommandForRobot() {
        return drivetrain.applyRequest(() -> {
            double leftX = joystick.getLeftX();
            double leftY = joystick.getLeftY();
            double rightX = joystick.getRightX();

            // $TODO - Wrapper for sim features
            if (Robot.isSimulation()) {
                JoystickInputsRecord newJoystickInputs = SimWrapper.transformJoystickOrientation(
                    drivetrain.getOperatorForwardDirection().getDegrees(),
                    leftX,
                    leftY,
                    rightX);
                leftX = newJoystickInputs.driveX();
                leftY = newJoystickInputs.driveY();
                rightX = newJoystickInputs.rotatetX();
            }

            return drive.withVelocityX(-leftY * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(-leftX * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(-rightX * MaxAngularRate); // Drive counterclockwise with negative X (left)
        });
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(getJoystickCommandForRobot());

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        joystick.povUp().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.povDown().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // $TODO - Bumper buttons
        if (Robot.isSimulation()) {
            // In simulation, inject drift with right bumper to test vision correction
            joystick.rightBumper().onTrue(drivetrain.runOnce(() ->
                m_simWrapper.getGroundTruthSim().injectDrift(0.5, 15.0)  // 0.5m translation, 15° rotation drift
            ));

            // Left bumper resets robot to the starting pose of the selected auto
            joystick.leftBumper().onTrue(drivetrain.runOnce(() ->
                m_simWrapper.getGroundTruthSim().cycleResetPosition(selectedAutoStartingPose)
            ));
        }

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

    /**
     * Initializes the auto selector and publishes it to SmartDashboard.
     */
    private void initAutoSelector() {
        SmartDashboard.putData("Auto Mode", autoChooser);

        // Update starting pose when auto selection changes
        autoChooser.onChange(this::onNewAutoSelected);

        // Initialize starting pose from default selection
        onNewAutoSelected(autoChooser.getSelected());
    }

    /**
     * Called when a new auto is selected from the chooser.
     * Updates the starting pose for simulation reset.
     */
    private void onNewAutoSelected(Command command) {
        if (command instanceof PathPlannerAuto auto) {
            Pose2d pose = auto.getStartingPose();
            selectedAutoStartingPose = pose != null ? pose : new Pose2d();
        } else {
            // "None" selection is an InstantCommand - reset to origin
            selectedAutoStartingPose = new Pose2d();
        }
    }

    /**
     * Called when the robot pose is reset in simulation.
     * This is triggered by GroundTruthSim via the consumer pattern.
     *
     * Resets both the ground truth pose and the drivetrain pose to the specified pose.
     * Also resets the vision system simulation pose history if a Vision instance is set.
     *
     * @param pose The new pose the robot has been reset to
     */
    private void resetRobotPose(Pose2d pose) {
        System.out.println("Robot pose reset to: " + pose);

        drivetrain.resetPose(pose);

        // $TODO - Clean reset
        if (Robot.isSimulation()) {
            m_simWrapper.resetSimPose(pose);
        }
    }
}
