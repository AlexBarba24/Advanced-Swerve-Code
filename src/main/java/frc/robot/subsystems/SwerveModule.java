package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule{

  private final TalonFX m_driveMotor;
  private final TalonSRX m_turningMotor;

  private final double m_id;

  private final Encoder m_turningEncoder;

  private final PIDController m_turningPidController;

  private final double MAX_VOLTAGE;

  private final double MAX_VELOCITY_METERS_PER_SECOND;


  public SwerveModule(TalonFX driveMotor, TalonSRX turningMotor, Encoder turningEncoder, double MAX_VOLT, double MAX_SPEED, double id){

    m_id = id;

    MAX_VOLTAGE = MAX_VOLT;
    MAX_VELOCITY_METERS_PER_SECOND = MAX_SPEED;

    m_turningEncoder = turningEncoder;
      
    m_driveMotor = driveMotor;
    m_turningMotor = turningMotor;

    // m_driveMotor.getSelectedSensorVelocity(); units (2048) per 100ms
    //m_driveMotor.getSelectedSensorPosition(); in units (2048)

    m_turningPidController = new PIDController(0, 0, 0);
    m_turningPidController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public double getDrivePosition(){
    return encoderTicksToMeter(m_driveMotor.getSelectedSensorPosition());
  }

  public void spinWheel(){
    m_turningMotor.set(ControlMode.PercentOutput, 0.5);
  }

  public double getSteerAngle(){ 
    return m_turningEncoder.getDistance()/360 * (1/1.2) * 2 * Math.PI;
    // return 0;
  }

  public double getDriveVelocity(){
    return encoderTicksToMeter(m_driveMotor.getSelectedSensorVelocity());
  }

  public double getTurningVelocity(){
    return m_turningEncoder.getRate()/360 * (1/1.2) * 2 * Math.PI;
  }

  public void resetEncoders(){
    m_turningEncoder.reset();
  }

  public SwerveModuleState getState(){
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSteerAngle()));
  }

  public void set(double driveVoltage, double steerAngle){
        setDesiredState(new SwerveModuleState((driveVoltage/MAX_VOLTAGE) * MAX_VELOCITY_METERS_PER_SECOND, new Rotation2d(steerAngle)));
  }

  public void setDesiredState(SwerveModuleState state) {
    if(Math.abs(state.speedMetersPerSecond) < 0.001){
      stop();
      return;
    }
    state = SwerveModuleState.optimize(state, getState().angle);
    m_driveMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND);
    m_turningMotor.set(ControlMode.PercentOutput,m_turningPidController.calculate(getSteerAngle(), state.angle.getRadians()));
    SmartDashboard.putNumber("Turning Speed of Module: ["+m_id+"]", m_turningPidController.calculate(getSteerAngle(), state.angle.getRadians()));
    SmartDashboard.putString("Swerve["+ m_id + "]", state.toString());
  }

  public void stop(){
    m_driveMotor.set(ControlMode.PercentOutput, 0);
    m_turningMotor.set(ControlMode.PercentOutput, 0);
  }

        /**
         * 
         * @param meters
         * @return encoder ticks 
         * Converts meters to encoder ticks 
         */
        public double meterToEncoderTicks(double meters){
            return meters * (2048/((1/6.67) * Units.inchesToMeters(4) * Math.PI));
    }
    /**
     * 
     * @param encoderTicks
     * @return meters for robot drive
     * Converts encoder ticks to meters
     */
    public double encoderTicksToMeter(double encoderTicks){
            return encoderTicks /  (2048/((1/6.67) * Units.inchesToMeters(4) * Math.PI));
    }

}