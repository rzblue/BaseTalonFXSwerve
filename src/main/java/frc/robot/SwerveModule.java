package frc.robot;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycleRequest = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocityRequest = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PositionVoltage anglePositionRequest = new PositionVoltage(0);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID);
        mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
        resetToAbsolute();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
        mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.getConfigurator().setPosition(0.0);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState.optimize(getState().angle);

        // Cosine compensation
        var angleError = desiredState.angle.minus(getAngle());
        // Clamp scale factor to (0, 1) to prevent reversing
        // This shouldn't ever happen but just in case
        var velocityInDesiredDirection =
            MathUtil.clamp(angleError.getCos(), 0, 1) * desiredState.speedMetersPerSecond;

        setAngle(desiredState.angle);
        setSpeed(velocityInDesiredDirection, isOpenLoop);
    }

    private void setAngle(Rotation2d angle) {
        mAngleMotor.setControl(anglePositionRequest.withPosition(angle.getRotations()));
    }

    private void setSpeed(double speedMetersPerSecond, boolean isOpenLoop){
        // Convert linear speed of wheel to motor speed
        var requestedVelocityRPS = wheelMeterToMotorRot(speedMetersPerSecond);
        // Calculate motor velocity required to hold wheel still at the current azimuth velocity
        // Don't compensate if requested velocity is 0 - just stop the motor
        double compensationVelocity = 0;
        if (speedMetersPerSecond != 0) {
            compensationVelocity =
                mAngleMotor.getVelocity().getValueAsDouble() * Constants.Swerve.azimuthCouplingRatio;
        }
        var outputVelocity = requestedVelocityRPS + compensationVelocity;

        if (isOpenLoop) {
            driveDutyCycleRequest.Output =
                outputVelocity / wheelMeterToMotorRot(Constants.Swerve.maxSpeed);
            mDriveMotor.setControl(driveDutyCycleRequest);
        } else {
            driveVelocityRequest.Velocity = outputVelocity;
            mDriveMotor.setControl(driveVelocityRequest);
        }
    }

    public Rotation2d getCANcoder(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(mAngleMotor.getPosition().getValueAsDouble());
    }

    public void resetToAbsolute(){
        double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
        mAngleMotor.setPosition(absolutePosition);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.RPSToMPS(mDriveMotor.getVelocity().getValueAsDouble(), Constants.Swerve.wheelCircumference), 
            getAngle()
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(mDriveMotor.getPosition().getValueAsDouble(), Constants.Swerve.wheelCircumference), 
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValueAsDouble())
        );
    }

    public static double wheelMeterToMotorRot(double wheelMeters) {
        return Conversions.metersToRotations(wheelMeters, Constants.Swerve.wheelCircumference)
            * Constants.Swerve.driveGearRatio;
    }

    public static double motorRotToWheelMeter(double motorRot) {
        return Conversions.rotationsToMeters(
            motorRot / Constants.Swerve.driveGearRatio, Constants.Swerve.wheelCircumference);
    }
}
