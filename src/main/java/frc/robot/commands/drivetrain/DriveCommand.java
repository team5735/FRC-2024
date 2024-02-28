package frc.robot.commands.drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DrivetrainConstants;
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
        double speedX = -m_stickY.get() * multiplier;
        double speedY = -m_stickX.get() * multiplier;
        double speedOmega = m_omegaLimiter.calculate(m_rotate.get() * multiplier);
        if (DrivetrainConstants.USING_THETA_MAGNITUDE_LIMITING) {
            double theta = m_thetaLimiter.calculate(new Rotation2d(speedX, speedY).getRadians());
            double magnitude = m_magnitudeLimiter.calculate(Math.sqrt(speedX * speedX + speedY * speedY));
            Translation2d thetaMagnitudeMovement = new Translation2d(magnitude, theta);
            SmartDashboard.putNumber("drive_theta", theta);
            SmartDashboard.putNumber("drive_magnitude", magnitude);
            m_drivetrain.drive(thetaMagnitudeMovement, speedOmega);
        } else {
            speedX = m_xLimiter.calculate(-m_stickY.get() * multiplier);
            speedY = m_yLimiter.calculate(-m_stickX.get() * multiplier);
            SmartDashboard.putNumber("drive_speedX", speedX);
            SmartDashboard.putNumber("drive_speedY", speedY);
            SmartDashboard.putNumber("drive_speedOmega", speedOmega);
            m_drivetrain.drive(speedX, speedY, speedOmega);
        }

        m_watchdog.addEpoch("drivetrain_update");
        m_watchdog.disable();
        if (m_watchdog.isExpired()) {
            System.out.println("watchdog expired :( ");
            m_watchdog.printEpochs();
        }
    }
}
