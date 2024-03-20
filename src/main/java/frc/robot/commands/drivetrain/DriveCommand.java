package frc.robot.commands.drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.DrivetrainConstants.SlewRateLimiterMode;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveCommand extends Command {
    private final DrivetrainSubsystem m_drivetrain;
    private final Supplier<Double> m_stickX;
    private final Supplier<Double> m_stickY;
    private final Supplier<Double> m_rotate;
    private final Supplier<Double> m_multiplier;
    private Watchdog m_watchdog = new Watchdog(0.02, () -> {
    });
    private SlewRateLimiter m_thetaLimiter = new SlewRateLimiter(DrivetrainConstants.ACCEL_LIMIT_THETA_MAGNITUDE);
    private SlewRateLimiter m_magnitudeLimiter = new SlewRateLimiter(DrivetrainConstants.ACCEL_LIMIT_THETA_MAGNITUDE);
    private SlewRateLimiter m_xLimiter = new SlewRateLimiter(DrivetrainConstants.ACCEL_LIMIT_AXES);
    private SlewRateLimiter m_yLimiter = new SlewRateLimiter(DrivetrainConstants.ACCEL_LIMIT_AXES);
    private SlewRateLimiter m_omegaLimiter = new SlewRateLimiter(DrivetrainConstants.ACCEL_LIMIT_OMEGA);

    /**
     * Creates a new DriveCommand. This class takes the drivetrain to drive, the
     * stick x and y inputs, the rotate inputs, and a final supplier that it
     * multiplies the stick inputs by to determine the speed to drive the drivetrain
     * at. Depending on the value of {@link DrivetrainConstants.SLEW_RATE_LIMITER},
     * behavior changes. See execute() to understand what happens based on each
     * mode.
     */
    public DriveCommand(DrivetrainSubsystem drivetrain, Supplier<Double> stickX, Supplier<Double> stickY,
            Supplier<Double> rotate, Supplier<Double> multiplier) {
        m_drivetrain = drivetrain;

        m_stickX = stickX;
        m_stickY = stickY;
        m_rotate = rotate;
        m_multiplier = multiplier;

        addRequirements(m_drivetrain);
    }

    public void execute() {
        m_watchdog.reset();

        double multiplier = m_multiplier.get();
        double speedX = m_stickY.get() * multiplier;
        double speedY = m_stickX.get() * multiplier;
        double speedOmega = m_omegaLimiter.calculate(m_rotate.get() * multiplier);
        if (DrivetrainConstants.SLEW_RATE_LIMITER_MODE == SlewRateLimiterMode.THETA_MAGNITUDE) {
            driveThetaMagnitudeSRL(speedX, speedY, speedOmega);
        } else if (DrivetrainConstants.SLEW_RATE_LIMITER_MODE == SlewRateLimiterMode.AXES) {
            driveAxesSRL(speedX, speedY, speedOmega);
        } else {
            m_drivetrain.drive(speedX, speedY, speedOmega);
        }

        m_watchdog.addEpoch("drivetrain_update");
        m_watchdog.disable();
        if (m_watchdog.isExpired()) {
            System.out.println("watchdog expired :( ");
            m_watchdog.printEpochs();
        }

        // Testing please remove unless you put this here and still need it
        SmartDashboard.putNumber("drivetrain reported theta", m_drivetrain.getRotation3d().getZ());
    }

    /**
     * This passes the speedX and speedY values into a {@link SlewRateLimiter}. It
     * doesn't limit the acceleration at which the drivetrain can change direction
     * of movement, which means that the steer motors can accelerate unbounded.
     */
    private void driveAxesSRL(double speedX, double speedY, double speedOmega) {
        speedX = m_xLimiter.calculate(m_stickY.get());
        speedY = m_yLimiter.calculate(m_stickX.get());
        SmartDashboard.putNumber("drive_speedX", speedX);
        SmartDashboard.putNumber("drive_speedY", speedY);
        SmartDashboard.putNumber("drive_speedOmega", speedOmega);
        m_drivetrain.drive(speedX, speedY, speedOmega);
    }

    /**
     * This interprets speedX and speedY as a vector and uses
     * {@link SlewRateLimiter} to interpolate between one vector and another.
     *
     * <p>
     * Currently, this is implemented using complex numbers. It may be easier to
     * visualize as vectors, but you should learn complex numbers anyway.
     *
     * <ol>
     * <li>Interpret speedX and speedY as a complex number c, where c = speedX +
     * speedY * i.
     * <li>Convert c to polar form, that is, r * e^i*theta.
     * <li>Pass r and theta to a slew rate limiter.
     * <li>Convert the resulting polar number back into rectangular form, d =
     * speedX'
     * + speedY' * i.
     * <li>Pass speedX' and speedY' to the drivetrain, and pass speedOmega
     * unmodified.
     */
    private void driveThetaMagnitudeSRL(double speedX, double speedY, double speedOmega) {
        double theta = m_thetaLimiter.calculate(new Rotation2d(speedX, speedY).getRadians());
        double r = m_magnitudeLimiter.calculate(Math.sqrt(speedX * speedX + speedY * speedY));
        Translation2d thetaMagnitudeMovement = new Translation2d(r, theta);
        SmartDashboard.putNumber("drive_theta", theta);
        SmartDashboard.putNumber("drive_magnitude", r);
        m_drivetrain.drive(thetaMagnitudeMovement, speedOmega);
    }
}
