// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private boolean autocompleted;//true when auto was run false is not comple

  private static final String kDefaultAuto = "highgoal and move";
  private static final String kCustomAuto = "My Auto";
  
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
   private double autoStartTime;

  //auto time slice tunning
  boolean timeSlice1Complete;
  boolean timeSlice2Complete;
  boolean timeSlice3Complete;

  //auto booleans for highgoal and move
  boolean function1Complete;
  boolean function2Complete;
  boolean function3Complete;
  boolean funtion4Complete;

  

  private XboxController driveController = new XboxController(1);
  private Joystick operatorContorlBoard = new Joystick(0);

  // Drivetrain Motors
  private WPI_TalonFX frontRightMasterFx = new WPI_TalonFX(3);
  private WPI_TalonFX frontLeftMasterFx = new WPI_TalonFX(2);
  private WPI_TalonFX rearRightSlaveFx = new WPI_TalonFX(5);
  private WPI_TalonFX rearLeftSlaveFx = new WPI_TalonFX(4);
  private DifferentialDrive m_Drive = new DifferentialDrive(frontRightMasterFx, frontLeftMasterFx);

  // Arm Mechanism Motors
  private WPI_TalonFX armTalonFx = new WPI_TalonFX(12);
  private WPI_TalonFX sliderTalonFx = new WPI_TalonFX(14);
  private CANSparkMax intakeRollers = new CANSparkMax(20, MotorType.kBrushless);
  private RelativeEncoder intakeEncoder;

  // other declarations for hardware
  private Pigeon2 _Pigeon2 = new Pigeon2(15);

  // camera
   UsbCamera opsCamera;
   

  // Limelight

  private Timer timer;

  //ADIS16470_IMU gyro = new ADIS16470_IMU();



 
  PIDController pid90 = new PIDController(0.038, 0.0, 0.008);//swithing p from 0.015 to 0.02 switching d from 0.005 to 0.007 to 0.008
  PIDController pid180 = new PIDController(0.035, 0.00, 0.015);


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
   // gyro.reset();
    m_chooser.setDefaultOption("charge station auto", kDefaultAuto);
    m_chooser.addOption("High goal and move", kCustomAuto);
 
    SmartDashboard.putData("Auto choices", m_chooser);
    int _smoothing = 4;
    int _sliderSmoothing = 3;//changing from 7 to 8 changed from 8 to 3


    //camera
    opsCamera = CameraServer.startAutomaticCapture(1);//turned on camera for camera test do so in other code if this is not ran
    

    //config for drive train
   frontRightMasterFx.setInverted(true);
   frontLeftMasterFx.setInverted(false);
   frontLeftMasterFx.setNeutralMode(NeutralMode.Brake);//**PUT BACK IN BREAK MODE**
   frontRightMasterFx.setNeutralMode(NeutralMode.Brake);//**PUT BACK IN BREAK MODE**

   rearRightSlaveFx.follow(frontRightMasterFx);
   rearLeftSlaveFx.follow(frontLeftMasterFx);
   rearLeftSlaveFx.setNeutralMode(NeutralMode.Brake);
   rearRightSlaveFx.setNeutralMode(NeutralMode.Brake);

   rearRightSlaveFx.setInverted(InvertType.FollowMaster);
   rearLeftSlaveFx.setInverted(InvertType.FollowMaster);

    frontLeftMasterFx.configOpenloopRamp(0, 10);
    frontRightMasterFx.configOpenloopRamp(0, 10);
    rearLeftSlaveFx.configOpenloopRamp(0, 10);
    rearRightSlaveFx.configOpenloopRamp(0, 10);


    frontRightMasterFx.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    frontLeftMasterFx.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rearRightSlaveFx.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rearLeftSlaveFx.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    frontRightMasterFx.setSelectedSensorPosition(0);
    frontLeftMasterFx.setSelectedSensorPosition(0);

  

   //config for intake
   intakeRollers.restoreFactoryDefaults();
   intakeEncoder = intakeRollers.getEncoder();
   intakeRollers.setIdleMode(IdleMode.kBrake);


   //config for arm
   armTalonFx.configFactoryDefault();

   armTalonFx.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
   armTalonFx.setSelectedSensorPosition(0);

   armTalonFx.setNeutralMode(NeutralMode.Coast);
    //**MOTION MAGIC**
   armTalonFx.configNominalOutputForward(0, 10);
   armTalonFx.configNominalOutputReverse(0, 10);
   armTalonFx.configPeakOutputForward(0.7, 10);
   armTalonFx.configPeakOutputReverse(-0.7, 10); 

   

   armTalonFx.configAllowableClosedloopError(0, 500, 10);

   armTalonFx.configForwardSoftLimitThreshold(80000, 10);
   armTalonFx.configReverseSoftLimitThreshold(0, 10);
   armTalonFx.configForwardSoftLimitEnable(true, 10);
   armTalonFx.configReverseSoftLimitEnable(true, 10);
   armTalonFx.configMotionCruiseVelocity(_smoothing, 10);

   armTalonFx.configOpenloopRamp(0.5,10);
   armTalonFx.configNeutralDeadband(0.001, 10); 
   
    //arm MotionMagic gains
   armTalonFx.selectProfileSlot(0, 0);
   armTalonFx.config_kF(0, 0.05, 10);
   armTalonFx.config_kP(0,0.1938, 10);//changing kP to increas arm speed from 0.09 to .1438 then changed to add .05 that came out to .1938
   armTalonFx.config_kD(0, 0, 10);
   armTalonFx.configMotionAcceleration(5500, 10);//increasing fro 1600 to 2133 to do 60*/second changing 2133 to 3500
   armTalonFx.configMotionCruiseVelocity(20500 , 10);//increasing from 3200 to 4267 to do 60*/second changing 4267 to 5500 up 
    //**END OF MOTION MAGIC FOR THIS GROUP


   //config for the slider
   sliderTalonFx.configFactoryDefault();

   

   sliderTalonFx.setNeutralMode(NeutralMode.Brake);

   sliderTalonFx.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
   sliderTalonFx.setSelectedSensorPosition(0);

   //**MOTION MAGIC

   sliderTalonFx.configNominalOutputForward(0, 10);
   sliderTalonFx.configNominalOutputReverse(0, 10);
   sliderTalonFx.configPeakOutputForward(0.7, 10);
   sliderTalonFx.configPeakOutputReverse(-0.7, 10);

   sliderTalonFx.configForwardSoftLimitThreshold(65000, 10);//change from 63400 to 52000
   sliderTalonFx.configReverseSoftLimitThreshold(-49000, 10);
   sliderTalonFx.configForwardSoftLimitEnable(true, 10);//set to true before match
   sliderTalonFx.configReverseSoftLimitEnable(true, 10);

   sliderTalonFx.configOpenloopRamp(0.5, 10);

   sliderTalonFx.configNeutralDeadband(0.01, 10);

   sliderTalonFx.configAllowableClosedloopError(0, 150, 10);
   sliderTalonFx.configMotionSCurveStrength(_sliderSmoothing, 10);

    sliderTalonFx.setInverted(true);

   sliderTalonFx.selectProfileSlot(0, 0);

   //slider MotionMagic Gains
   sliderTalonFx.config_kF(0, 0.05, 10);
   sliderTalonFx.config_kP(0, 0.3, 10);//change form .11 to .2 up to .3
  // sliderTalonFx.config_kD(0, 2.2, 10);//changed from 2.2 to 3.3 to see if hard stop on slider will disapear

   sliderTalonFx.configMotionAcceleration(6500, 10);//increasing acceleration to keep up with velocity from 3100 to 4100 up to 5500 up to 6500
   sliderTalonFx.configMotionCruiseVelocity(22200, 10);// increasing cruise velocity from 5200 to 6200 to 7500 to 8500
   //** END OF MOTION MAGIC CONFIGS

  
  }
  @Override
  public void autonomousInit() {

    autocompleted = false;
     timeSlice1Complete = false;
    timeSlice2Complete = false;
    timeSlice3Complete = false;


    function1Complete = false;
    function2Complete = false;
    function3Complete = false;
    funtion4Complete = false;
   autoStartTime = Timer.getFPGATimestamp();
    
   frontLeftMasterFx.setSelectedSensorPosition(0);
   frontRightMasterFx.setSelectedSensorPosition(0);
    
    m_Drive.setSafetyEnabled(false);
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    System.out.println("High goal and move: " + kCustomAuto);
 
  
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    SmartDashboard.putNumber("left encoder ticks ", frontLeftMasterFx.getSelectedSensorPosition());

     if (autocompleted) return;
    switch (m_autoSelected) {
      case kCustomAuto:
        if (armTalonFx.getSelectedSensorPosition() < 70672 && function2Complete == false) {
          double armPos = 70672;//changed from 68550 to 70672
          double sliderPos = 49290;//changed from 61788 to 46500changed to 49290
          double sliderHome = 0;
          double armHome = 0;
          intakeRollers.set(.15);
          armTalonFx.set(TalonFXControlMode.MotionMagic, armPos);
          Timer.delay(.5);
          sliderTalonFx.set(TalonFXControlMode.MotionMagic, sliderPos);
          Timer.delay(4);
          intakeRollers.set(-0.30);
          Timer.delay(1);
          System.out.println("stop motor part");
          intakeRollers.set(0);
          Timer.delay(.5);
          System.out.println("end of function1");
          if (armTalonFx.getSelectedSensorPosition() >= 70672) {
            sliderTalonFx.set(TalonFXControlMode.MotionMagic, sliderHome);
            Timer.delay(.5);
            armTalonFx.set(TalonFXControlMode.MotionMagic, armHome);
            System.out.println("end of function2");
            Timer.delay(4);
            function1Complete = true;
            
          }
          
          if (armTalonFx.getSelectedSensorPosition() <= 200 && sliderTalonFx.getSelectedSensorPosition() <= 200 && function2Complete == false) {
            System.out.println("starting auto drive...");
            armTalonFx.stopMotor();
            sliderTalonFx.stopMotor();
            m_Drive.arcadeDrive(-.50, 0);
            Timer.delay(3);
            m_Drive.arcadeDrive(0, 0);
            function2Complete = true;
          } else {
            System.out.println("starting turning off... " + armTalonFx.getSelectedSensorPosition() + ", "  + sliderTalonFx.getSelectedSensorPosition());
            armTalonFx.set(0);
            sliderTalonFx.set(0);
            intakeRollers.set(0);
            m_Drive.arcadeDrive(0, 0);
          }
          break;

        }
    } 
    switch (m_autoSelected) {
      case kDefaultAuto:
      SmartDashboard.putNumber("left encoder ticks ", frontLeftMasterFx.getSelectedSensorPosition());
          double currentTime = Timer.getFPGATimestamp();
    double elapsedTime = currentTime - autoStartTime;
      //first elapsed time equals 5.74 seconds
       if (elapsedTime < 12 && timeSlice1Complete == false ){ 
        
        double armPos = 70672;
        double sliderPos = 65495.28;
        double sliderHome = 0;
        double armHome = 0;
        intakeRollers.set(.15);
        Timer.delay(.001);
          armTalonFx.set(TalonFXControlMode.MotionMagic, armPos);
          Timer.delay(.5);
          sliderTalonFx.set(TalonFXControlMode.MotionMagic, sliderPos);
          Timer.delay(2.1);//2.5 little long going to 2.3
          intakeRollers.set(-0.30);
          Timer.delay(.6);
          //System.out.println("stop motor part " + elapsedTime + "  " + autoStartTime);
          intakeRollers.set(0);
          Timer.delay(.1);
          //System.out.println("end of function1 " + elapsedTime + "  " + autoStartTime);
        
    
          sliderTalonFx.set(TalonFXControlMode.MotionMagic, sliderHome);
          Timer.delay(.5);
          armTalonFx.set(TalonFXControlMode.MotionMagic, armHome);
          Timer.delay(1.6);
         // System.out.println("end of function2 " + elapsedTime + "  " + autoStartTime);
          timeSlice1Complete = true;
        } 
        
       //-126185 to 
        while ((frontLeftMasterFx.getSelectedSensorPosition() + frontRightMasterFx.getSelectedSensorPosition()) /2 >= -119755) {

          m_Drive.tankDrive(-0.50, -0.50);
          
        }
        System.out.println("stopDriving " + frontLeftMasterFx.getSelectedSensorPosition());
        m_Drive.tankDrive(0, 0);
        Timer.delay(0.1);
        while ((frontLeftMasterFx.getSelectedSensorPosition() + frontRightMasterFx.getSelectedSensorPosition()) /2 <= -66000) {
          m_Drive.tankDrive(0.45, 0.45);
        }
        m_Drive.tankDrive(0, 0);
        
    

    
    autocompleted = true;
  

  }
}

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("left Encoder ticks", frontLeftMasterFx.getSelectedSensorPosition());
  


    // driveroperations
    double speed = -(driveController.getLeftY());
    double turn = driveController.getRightX();

    m_Drive.arcadeDrive(speed , turn );

    if (driveController.getRightTriggerAxis() == 1){
      m_Drive.arcadeDrive(speed * .50, turn * .45);
    }


  
   // m_Drive.tankDrive(-driveController.getRightY(), -driveController.getLeftY(), true);
    //m_Drive.tankDrive(-driveController.getRightY(), driveController.getRightY());

     if (Math.abs(speed) < 0.05) {
      speed = 5;
    } else if (Math.abs(turn) < 0.05) {
      turn = 0;
    }
                                        //reset ADIS gyro
    /*if (driveController.getYButton()) {
      gyro.reset();
     }
                                        //turning to the right 90
     if (driveController.getBButton()) {
      pid90.setSetpoint(-90);
      double sensor = gyro.getAngle();
      double output = pid90.calculate(sensor);
      m_Drive.tankDrive(output, -output);
     }
                                          //turning right 180
     else if (driveController.getAButton()) {
  
      pid180.setSetpoint(-180);
      double sensor = gyro.getAngle();
      double output = pid180.calculate(sensor);
      m_Drive.tankDrive(output, -output);
     }*/
    
 
    

    //**MOTION MAGIC
    // mechanism operations


    if (operatorContorlBoard.getRawButton(5)) {
      double armTargetPos1 = 46000;//from 43575 to 37729 to 40575 to 42075
      double sliderTargetPos1 = 64500;//from 62553 to 49330  changed to 52289.8

      armTalonFx.set(ControlMode.MotionMagic, armTargetPos1);
      Timer.delay(.5);
      sliderTalonFx.set(TalonFXControlMode.MotionMagic, sliderTargetPos1);
    }

    else if (operatorContorlBoard.getRawButton(6)) {

      double preHumanIntakeArm = 54840;
      double preHumanIntakeSlider = -38000; // -47523 to -40000 to -39000 to -38000 changed to -40280

      armTalonFx.set(TalonFXControlMode.MotionMagic, preHumanIntakeArm);
      Timer.delay(.5);
      sliderTalonFx.set(TalonFXControlMode.MotionMagic, preHumanIntakeSlider);
    } 
     if (operatorContorlBoard.getRawButton(4)) {
      double humanIntakeArm = 52072; // 51000 to 52072

      double humanIntakeSlider = 0; // -6777 to -33318 to -30318 changed to -32137.08

      sliderTalonFx.set(TalonFXControlMode.MotionMagic, humanIntakeSlider);
      armTalonFx.set(TalonFXControlMode.MotionMagic, humanIntakeArm);

    }

    else if (operatorContorlBoard.getRawButton(1)) {
      double armTargetPos2 = 70672;//changing from 68550 to 70672 to 73585
      double sliderPos2 = 65495.28;//changing from 61788 to 50332 to 50011 to 48500 to 46500 changed to 65495.28 for double 4 slices

      armTalonFx.set(TalonFXControlMode.MotionMagic, armTargetPos2);
      Timer.delay(.5);// down from .7 to .5
      sliderTalonFx.set(TalonFXControlMode.MotionMagic, sliderPos2);
//button 6 is ok
    } else if (operatorContorlBoard.getRawButton(2)) {
      double armTargetPos0 = 0;
      double sliderTargetPos0 = 0;

      sliderTalonFx.set(TalonFXControlMode.MotionMagic, sliderTargetPos0);

      Timer.delay(.5);

      armTalonFx.set(TalonFXControlMode.MotionMagic, armTargetPos0);
    }
//origanally button 7 now 13
    /*if (operatorContorlBoard.getRawButton(13)) {
      sliderTalonFx.setSelectedSensorPosition(0);
      armTalonFx.setSelectedSensorPosition(0);

    }*/
//button 4 is ok
    if (operatorContorlBoard.getRawButtonPressed(7)) {
      intakeRollers.set(0.25);
    } else if (operatorContorlBoard.getRawButtonReleased(7)) {
      intakeRollers.stopMotor();
    } 
    //origanally button 2 mow button 3
    else if (operatorContorlBoard.getRawButtonPressed(8)) {
      intakeRollers.set(-0.25);
    } else if (operatorContorlBoard.getRawButtonReleased(8)) {
      intakeRollers.stopMotor();
    }

    //**END OF MOTION MAGIC

    // pigeon gyro
    double yaw = _Pigeon2.getYaw();
    double pitch = _Pigeon2.getPitch();

    SmartDashboard.putNumber("pitch", _Pigeon2.getPitch());
   // SmartDashboard.putNumber("angle", gyro.getAngle());

  }

  @Override
  public void disabledInit() {
  armTalonFx.stopMotor();
  sliderTalonFx.stopMotor();
  intakeRollers.stopMotor();
  m_Drive.stopMotor();
  }

  @Override
  public void disabledPeriodic() {

  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
