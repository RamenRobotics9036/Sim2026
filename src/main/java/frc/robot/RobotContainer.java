// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

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
import frc.robot.visutils.LimelightOdometry;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the Robot
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // kSpeedAt12Volts desired top speed
    private static double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    // 3/4 of a rotation per second max angular velocity
    private static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric m_drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1)
        .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Open-loop control for motors
    private final SwerveRequest.SwerveDriveBrake m_brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt m_point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric m_forwardStraight = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry m_logger = new Telemetry(MaxSpeed);

    private final CommandXboxController m_joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> m_autoChooser;

    /** Stores the starting pose of the currently selected auto. */
    private Pose2d m_selectedAutoStartingPose = new Pose2d();

    /** Simulation wrapper - null when not in simulation. */
    public final SimWrapper m_simWrapper;

    public final LimelightOdometry m_limelightOdometry;

    /** Constructor. */
    @SuppressWarnings("removal")
    public RobotContainer() {
        m_autoChooser = AutoBuilder.buildAutoChooser("Tests");
        initAutoSelector();

        configureBindings();

        // $TODO - Wrapper for sim features
        if (Robot.isSimulation()) {
            m_simWrapper = new SimWrapper(
                m_drivetrain,
                this::resetRobotPose);
        }
        else {
            m_simWrapper = null;
        }

        m_limelightOdometry = new LimelightOdometry(m_drivetrain::addVisionMeasurement);

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
    }

    private Command getJoystickCommandForRobot() {
        return m_drivetrain.applyRequest(() -> {
            double leftX = m_joystick.getLeftX();
            double leftY = m_joystick.getLeftY();
            double rightX = m_joystick.getRightX();

            // $TODO - Wrapper for sim features
            if (Robot.isSimulation()) {
                JoystickInputsRecord newJoystickInputs = SimWrapper.transformJoystickOrientation(
                    m_drivetrain.getOperatorForwardDirection().getDegrees(),
                    leftX,
                    leftY,
                    rightX);
                leftX = newJoystickInputs.driveX();
                leftY = newJoystickInputs.driveY();
                rightX = newJoystickInputs.rotatetX();
            }

            return m_drive.withVelocityX(-leftY * MaxSpeed) // Drive forward with negative Y
                .withVelocityY(-leftX * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(-rightX * MaxAngularRate); // Counterclockwise negative X (left)
        });
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        m_drivetrain.setDefaultCommand(getJoystickCommandForRobot());

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            m_drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        m_joystick.a().whileTrue(m_drivetrain.applyRequest(() -> m_brake));
        m_joystick.b().whileTrue(
            m_drivetrain.applyRequest(
                () -> m_point.withModuleDirection(
                    new Rotation2d(-m_joystick.getLeftY(), -m_joystick.getLeftX()))));

        m_joystick.povUp().whileTrue(
            m_drivetrain.applyRequest(() -> m_forwardStraight.withVelocityX(0.5).withVelocityY(0)));
        m_joystick.povDown().whileTrue(
            m_drivetrain
                .applyRequest(() -> m_forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        m_joystick
            .back()
            .and(m_joystick.y())
            .whileTrue(m_drivetrain.sysIdDynamic(Direction.kForward));
        m_joystick
            .back()
            .and(m_joystick.x())
            .whileTrue(m_drivetrain.sysIdDynamic(Direction.kReverse));
        m_joystick
            .start()
            .and(m_joystick.y())
            .whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kForward));
        m_joystick
            .start()
            .and(m_joystick.x())
            .whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kReverse));

        // $TODO - Bumper buttons
        if (Robot.isSimulation()) {
            // In simulation, inject drift with right bumper to test vision correction
            m_joystick.rightBumper()
                .onTrue(m_drivetrain.runOnce(() -> m_simWrapper.injectDrift(0.5, 15.0)));

            // Left bumper resets robot to the starting pose of the selected auto
            m_joystick.leftBumper().onTrue(
                m_drivetrain
                    .runOnce(() -> m_simWrapper.cycleResetPosition(m_selectedAutoStartingPose)));
        }

        m_drivetrain.registerTelemetry(m_logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return m_autoChooser.getSelected();
    }

    /**
     * Initializes the auto selector and publishes it to SmartDashboard.
     */
    private void initAutoSelector() {
        SmartDashboard.putData("Auto Mode", m_autoChooser);

        // Update starting pose when auto selection changes
        m_autoChooser.onChange(this::onNewAutoSelected);

        // Initialize starting pose from default selection
        onNewAutoSelected(m_autoChooser.getSelected());
    }

    /**
     * Called when a new auto is selected from the chooser.
     * Updates the starting pose for simulation reset.
     */
    private void onNewAutoSelected(Command command) {
        if (command instanceof PathPlannerAuto auto) {
            Pose2d pose = auto.getStartingPose();
            m_selectedAutoStartingPose = pose != null ? pose : new Pose2d();
        }
        else {
            // "None" selection is an InstantCommand - reset to origin
            m_selectedAutoStartingPose = new Pose2d();
        }
    }

    /**
     * Called when the robot pose is reset in simulation.
     * This is triggered by GroundTruthSim via the consumer pattern.
     *
     * <p>Resets both the ground truth pose and the drivetrain pose to the specified pose.
     * Also resets the vision system simulation pose history if a Vision instance is set.
     *
     * @param pose The new pose the robot has been reset to
     */
    private void resetRobotPose(Pose2d pose) {
        System.out.println("Robot pose reset to: " + pose);

        m_drivetrain.resetPose(pose);

        // $TODO - Clean reset
        if (Robot.isSimulation()) {
            m_simWrapper.resetSimPose(pose);
        }
    }
}
