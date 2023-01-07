package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants.CanConstants;
import frc.robot.Constants.OperatorConstants.DriveConstants;
import frc.robot.Constants.OperatorConstants.RobotConstants;

public class DrivetrainSubsystem extends SubsystemBase {

    private SwerveDriveOdometry m_odometry;
    private GenericEntry odometryEntry;
  
    public final Field2d m_field = new Field2d();
  
  
    public static final double MAX_VOLTAGE = 12.0;
  
    TalonFX m_frontLeftDriveMotor = new TalonFX(CanConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR);
    TalonFX m_frontRightDriveMotor = new TalonFX(CanConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR);
    TalonFX m_backLeftDriveMotor = new TalonFX(CanConstants.BACK_LEFT_MODULE_DRIVE_MOTOR);
    TalonFX m_backRightDriveMotor = new TalonFX(CanConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR);
  
    TalonSRX m_frontLeftSteerMotor = new TalonSRX(CanConstants.FRONT_LEFT_MODULE_STEER_MOTOR);
    TalonSRX m_frontRightSteerMotor = new TalonSRX(CanConstants.FRONT_RIGHT_MODULE_STEER_MOTOR);
    TalonSRX m_backLeftSteerMotor = new TalonSRX(CanConstants.BACK_LEFT_MODULE_STEER_MOTOR);
    TalonSRX m_backRightSteerMotor = new TalonSRX(CanConstants.BACK_RIGHT_MODULE_STEER_MOTOR);
  
    Encoder m_frontLeftCanCoder = new Encoder(CanConstants.FRONT_LEFT_MODULE_STEER_ENCODER[0], CanConstants.FRONT_LEFT_MODULE_STEER_ENCODER[1]);
    Encoder m_backLeftCanCoder = new Encoder(CanConstants.BACK_LEFT_MODULE_STEER_ENCODER[0], CanConstants.BACK_LEFT_MODULE_STEER_ENCODER[1]);
    Encoder m_frontRightCanCoder = new Encoder(CanConstants.FRONT_RIGHT_MODULE_STEER_ENCODER[0], CanConstants.FRONT_RIGHT_MODULE_STEER_ENCODER[1]);
    Encoder m_backRightCanCoder = new Encoder(CanConstants.BACK_RIGHT_MODULE_STEER_ENCODER[0], CanConstants.BACK_RIGHT_MODULE_STEER_ENCODER[1]);
  
  
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 0.9;
    double m_timeDelay;
  
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND / Math.hypot(RobotConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, RobotConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0);
  
    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      // Front left
      new Translation2d(RobotConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
        RobotConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Front right
      new Translation2d(RobotConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
        -RobotConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Back left
      new Translation2d(-RobotConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
        RobotConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Back right
      new Translation2d(-RobotConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
        -RobotConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0));
  
    private final AHRS gyro = new AHRS(Port.kMXP);
  
    private final SwerveModule m_frontLeftModule;
    private final SwerveModule m_frontRightModule;
    private final SwerveModule m_backLeftModule;
    private final SwerveModule m_backRightModule;
  
    private SwerveModulePosition[] m_position={null, null, null, null};
  
    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  
    /** Creates a new DrivetrainSubsystem. */
    public DrivetrainSubsystem() {
      ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
  
      m_frontLeftModule = new SwerveModule(m_frontLeftDriveMotor, m_frontLeftSteerMotor, m_frontLeftCanCoder, MAX_VOLTAGE, MAX_VELOCITY_METERS_PER_SECOND, 1);
      m_frontRightModule = new SwerveModule(m_frontRightDriveMotor, m_frontRightSteerMotor, m_frontRightCanCoder, MAX_VOLTAGE, MAX_VELOCITY_METERS_PER_SECOND, 2);
      m_backLeftModule = new SwerveModule(m_backLeftDriveMotor, m_backLeftSteerMotor, m_backLeftCanCoder, MAX_VOLTAGE, MAX_VELOCITY_METERS_PER_SECOND, 3);
      m_backRightModule = new SwerveModule(m_backRightDriveMotor, m_backRightSteerMotor, m_backRightCanCoder, MAX_VOLTAGE, MAX_VELOCITY_METERS_PER_SECOND, 4);
  
      updatePos();
  
      m_frontLeftDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
      m_backLeftDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
      m_frontRightDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
      m_backRightDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
      
      m_frontLeftDriveMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
      m_frontRightDriveMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
      m_backLeftDriveMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
      m_backRightDriveMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
  
      m_frontLeftDriveMotor.setNeutralMode(NeutralMode.Brake);
      m_frontRightDriveMotor.setNeutralMode(NeutralMode.Brake);
      m_backLeftDriveMotor.setNeutralMode(NeutralMode.Brake);
      m_backRightDriveMotor.setNeutralMode(NeutralMode.Brake);
  
      m_odometry = new SwerveDriveOdometry(m_kinematics, getGyroscopeRotation(), m_position);
  
      odometryEntry = tab.add("Odometry", "Not Found").getEntry();
  
      SmartDashboard.putData("Field", m_field);
  
      tab.add("gyro rot", getGyroscopeRotation().getDegrees());
    }
  
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      odometryEntry.setString(getCurrentPose().toString());
  
      SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
  
      SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
  
      m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
      m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
      m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
      m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
  
      updatePos();
  
      m_odometry.update(getGyroscopeRotation(), m_position);
  
      m_field.setRobotPose(m_odometry.getPoseMeters());
  
      SmartDashboard.putData(m_field);
    }
  
    public void zeroGyroscope() {
      gyro.reset();
    }
  
    public void setGyroscope(double deg) {
      gyro.reset();
      gyro.setAngleAdjustment(deg);
    }
  
    public void drive(ChassisSpeeds chassisSpeeds) {
      m_chassisSpeeds = chassisSpeeds;
    }
    
          //gets the number of ticks of the front left drive motor
          public double getAverageEncoder(){
            //returns in 2048/rotation
            return m_frontLeftDriveMotor.getSelectedSensorPosition();
    }
  
    //Sets internal encoders of driving swerve motor to 0 
    public void resetDriveEncoders(){
            m_backLeftDriveMotor.setSelectedSensorPosition(0.0);
            m_frontLeftDriveMotor.setSelectedSensorPosition(0.0);
            m_backRightDriveMotor.setSelectedSensorPosition(0.0);
            m_frontRightDriveMotor.setSelectedSensorPosition(0.0);
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
  
    public void resetOdometry(Pose2d pose){
      m_odometry.resetPosition(getGyroscopeRotation(), m_position, pose);
    }
  
  
    public void updatePos(){
      m_position[0] = new SwerveModulePosition(m_frontLeftModule.getDrivePosition(), new Rotation2d(m_frontLeftModule.getSteerAngle()));
      m_position[1] = new SwerveModulePosition(m_frontRightModule.getDrivePosition(), new Rotation2d(m_frontRightModule.getSteerAngle()));
      m_position[2] = new SwerveModulePosition(m_backLeftModule.getDrivePosition(), new Rotation2d(m_backLeftModule.getSteerAngle()));
      m_position[3] = new SwerveModulePosition(m_backRightModule.getDrivePosition(), new Rotation2d(m_backRightModule.getSteerAngle()));
    }
  
    public Rotation2d getGyroscopeRotation() {
      return Rotation2d.fromDegrees(gyro.getAngle());
    }   
    /**
     * 
     * @return Pose of robot from the odometry
     */
    public Pose2d getCurrentPose(){
      return m_odometry.getPoseMeters();
    }
    /**
     * 
     * @return the kinematics of the swerve drive
     */
    public SwerveDriveKinematics getKinematics(){
      return m_kinematics;
    }
    /**
     * 
     * @param states converts states to chasis speeds
     */
    public void actuateModulesAuto(SwerveModuleState[] states){
      drive(m_kinematics.toChassisSpeeds(states));
    }   
       
    /**
     * @param value The value of the joystick that will be modified
     * @param exponent The power to which the joystick will be raised to 
     * @return The modified value of the joystick
     */
    public double modifyAxis(double value, int exponent){
      double deadValue = MathUtil.applyDeadband(value, DriveConstants.DEADBAND);
      double quarticValue = Math.copySign(Math.pow(deadValue, exponent), deadValue);
      return quarticValue;
    }
  
  }
  