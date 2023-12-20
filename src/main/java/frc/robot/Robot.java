// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//2023 Comp Bot Code
//Last worked on 2/14/23

/*
\/\/\/\/\/\\/\/\/\/
GREG NO TOUCHY CODE.
/\/\/\/\/\/\/\/\/\/\
*/
//What this contains:
package frc.robot;

//import edu.wpi.first.wpilibj.Encoder;

import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.Timer; 

import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.RelativeEncoder;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.sql.Time;
import java.util.TimerTask;

import org.ejml.equation.Variable;



import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
//import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private final XboxController m_drivercontroller = new XboxController(0);

  private final XboxController m_grabbyarmcontroller = new XboxController(1);
  //private final Joystick m_Joystickbutton = new Joystick(1);

  DoubleSolenoid shifter = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  DoubleSolenoid clawGrabby = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);
  DoubleSolenoid armLifty = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 2);
  DoubleSolenoid armExtendy = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4,5);
  
  //DoubleSolenoid armExtend = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 7, 6);
  //DoubleSolenoid armRetract = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 9, 10);
  //DoubleSolenoid test = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);

  Compressor onBoardCompressor = new Compressor(30, PneumaticsModuleType.CTREPCM);

  private final Timer m_timer = new Timer();

  CANSparkMax arm = new CANSparkMax(20, CANSparkMax.MotorType.kBrushless);
  


  double clawEncoder;

  double armRoto;

  WPI_TalonSRX claw = new WPI_TalonSRX(5);

  AHRS gyro = new AHRS(Port.kMXP);

  Encoder turretEncoder = new Encoder(0, 1);

  WPI_TalonSRX turret = new WPI_TalonSRX(4);

  private RelativeEncoder m_encoder;

  WPI_TalonFX motor1 = new WPI_TalonFX(3);
  WPI_TalonFX motor2 = new WPI_TalonFX(2);
  WPI_TalonFX motor3 = new WPI_TalonFX(0);
  WPI_TalonFX motor4 = new WPI_TalonFX(1);

  MotorControllerGroup right = new MotorControllerGroup(motor1, motor2);
  MotorControllerGroup left = new MotorControllerGroup(motor3, motor4);

  DifferentialDrive m_drive;

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    gyro.calibrate();
    right.setInverted(true);
    m_drive = new DifferentialDrive(left,right);

    m_robotContainer = new RobotContainer();

    //armExtend.set(DoubleSolenoid.Value.kOff);
    //armRetract.set(DoubleSolenoid.Value.kOff);
    onBoardCompressor.enableDigital();

    m_encoder = arm.getEncoder();
    
    //armExtendy.set(false);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    

    //System.out.print(turretEncoder.getPosition());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    m_timer.reset();
    m_timer.start();
    
    

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    final float rollAngle = gyro.getRoll();
     
    if(m_timer.get() < 2.0){
      m_drive.tankDrive(0.5, 0.5);
    }else{
      m_drive.stopMotor();
    }

    if(m_timer.get() > 2.5 && m_timer.get() < 7.5){
      m_drive.tankDrive(-0.5, -0.5);
      
    }

    if(m_timer.get() > 8.0 && m_timer.get() < 11.5){
      m_drive.tankDrive(0.5, 0.5);
      
    }

    if( m_timer.get() > 12){
      if(rollAngle > 2.5){
        m_drive.tankDrive(-0.5, -0.5);
      }else if(rollAngle < -2.5){
        m_drive.tankDrive(0.5, 0.5);
      }else{
        m_drive.stopMotor();
      }
    } 
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double position = m_encoder.getPosition();
    clawEncoder = claw.getSelectedSensorPosition(0);
    armRoto = turret.getSelectedSensorPosition(0);

    SmartDashboard.putNumber("Encoder Position", m_encoder.getPosition());
    SmartDashboard.putNumber("Claw Position", clawEncoder);
    
    double leftTrigger = m_grabbyarmcontroller.getLeftTriggerAxis();
    double rightTrigger = m_grabbyarmcontroller.getRightTriggerAxis();
    
    
    //More Turret Stuff \/
    claw.stopMotor();
    if(leftTrigger != 0){
       if(rightTrigger != 0){
         claw.stopMotor();
       }else if(clawEncoder > -500){
        claw.set(-0.2);
       }
     }else if(rightTrigger != 0 && clawEncoder < 500){
      claw.set(.2);
     }

     arm.stopMotor();
    if (-m_grabbyarmcontroller.getLeftY() > 0 && position < 0){
      arm.set(-m_grabbyarmcontroller.getLeftY());
    }
    if (-m_grabbyarmcontroller.getLeftY() < 0 && position > -55){
      arm.set(-m_grabbyarmcontroller.getLeftY());
    }



    // This is the start of the return position button \/

      if(m_grabbyarmcontroller.getRawButton(5)){
        // stuff to do here
      }

    // This is the end of the return position button   /\
    //More Turret Stuff /\
      
     
     
    //final double turretAngle = turret.configForwardSoftLimitThreshold(kDefaultPeriod)
    
    //double turretMoto

    System.out.print(turret.getSelectedSensorPosition());

    if (m_drivercontroller.getRawButton(5)) {
      m_drive.tankDrive(-m_drivercontroller.getLeftY(), -m_drivercontroller.getRightY());
    } else {
      m_drive.arcadeDrive(-m_drivercontroller.getLeftY(), -m_drivercontroller.getRightX());
    }

    //turret spinny stuff \/
    armExtendy.set(Value.kOff);
    if (m_grabbyarmcontroller.getRawButton(5)){
      armExtendy.set(Value.kForward);
    }
    armLifty.set(Value.kOff);
    if (m_grabbyarmcontroller.getRawButton(4)){//Y
      armLifty.set(Value.kForward);
    }
    if(m_grabbyarmcontroller.getRawButton(2)){//B
      armLifty.set(Value.kReverse);
    }

    SmartDashboard.putNumber("Arm Roto", armRoto);

    turret.stopMotor();
    if (-m_grabbyarmcontroller.getRightX() > 0/*Y*/ /*&& turretAngle > 20*/){
      turret.set(-m_grabbyarmcontroller.getRightX());
    }
    if (-m_grabbyarmcontroller.getRightX() < 0/*B*/ /*&& turretAngle < 90*/){
      turret.set(-m_grabbyarmcontroller.getRightX());
    }
     
    shifter.set(DoubleSolenoid.Value.kOff);
    if (m_drivercontroller.getRawButton(3)){
        shifter.set(DoubleSolenoid.Value.kForward);
    }
    if (m_drivercontroller.getRawButton(1)){
      shifter.set(DoubleSolenoid.Value.kReverse);
    } 
    clawGrabby.set(Value.kOff);
    clawGrabby.set(DoubleSolenoid.Value.kOff);
    if (m_grabbyarmcontroller.getRawButton(3)/*X*/){
        clawGrabby.set(DoubleSolenoid.Value.kForward);
    }
    if (m_grabbyarmcontroller.getRawButton(1)/*A*/){
      clawGrabby.set(DoubleSolenoid.Value.kReverse);
    }
    
    
    //Joystick stuff for turret/arm
    /*if(m_Joystickbutton.getRawButton(7) == true ){
      System.out.print("Joystick works");
      armExtend.set(DoubleSolenoid.Value.kForward);
    }
    if(m_Joystickbutton.getRawButton(8) == true ){
      armExtend.set(DoubleSolenoid.Value.kReverse);
    }*/
  } 

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
