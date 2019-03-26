/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // Initialize motor controllers
  private final Spark armsMotor = new Spark(0);
  private final Spark rightBackMotor = new Spark(1);
  private final Spark rightFrontMotor = new Spark(2);
  private final Spark leftFrontMotor = new Spark(3);
  private final Spark leftBackMotor = new Spark(4);
  private final Spark elevatorMotor = new Spark(5);
  // Create speed controller groups for DifferentialDrive object
  private final SpeedControllerGroup leftMotors = new SpeedControllerGroup(leftBackMotor, leftFrontMotor);
  private final SpeedControllerGroup rightMotors = new SpeedControllerGroup(rightBackMotor, rightFrontMotor);
  private final DifferentialDrive m_robotDrive
      = new DifferentialDrive(rightMotors, leftMotors);
  // Initialize joystick
  private final Joystick m_stick = new Joystick(0);
  // Initialize timer
  private final Timer m_timer = new Timer();
  // Initialize pneumatic components
  private final Compressor mainCompressor = new Compressor(0);
  private final DoubleSolenoid clawPiston = new DoubleSolenoid(0, 1);
  private final DoubleSolenoid discPiston = new DoubleSolenoid(2, 3); 
  // Initialize joystick buttons
  //private final JoystickButton elevatorUpButton = new JoystickButton(m_stick, 6);
  //private final JoystickButton elevatorDownButton = new JoystickButton(m_stick, 4);
  private final JoystickButton armsEjectButton = new JoystickButton(m_stick, 1);
  private final JoystickButton armsPullButton = new JoystickButton(m_stick, 2);
  private final JoystickButton clawRetractButton = new JoystickButton(m_stick, 3);
  private final JoystickButton clawExtendButton = new JoystickButton(m_stick, 5);
  private final JoystickButton discGrabberRetractButton = new JoystickButton(m_stick, 6);
  private final JoystickButton discGrabberExtendButton = new JoystickButton(m_stick, 4);
  // Initialize pressure sensor
  private final AnalogInput pressureSensor = new AnalogInput(0);
  // Initialize power distribution panel
  private final PowerDistributionPanel mainPDP = new PowerDistributionPanel(0);
  //private final DigitalInput testSwitch = new DigitalInput(0);

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    SmartDashboard.putBoolean("PCM Sticky Fault Reset", false);
    SmartDashboard.putBoolean("PDP Sticky Fault Reset", false);
    SmartDashboard.putBoolean("PDP Energy Reset", false);
    SmartDashboard.putString("Claw Status", "Off");
    SmartDashboard.putString("Disc Status", "Off");
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Pressure Sensor", 250 * (pressureSensor.getVoltage() / 5) - 20);
    SmartDashboard.putBoolean("PCM Fault", mainCompressor.getCompressorCurrentTooHighFault() || mainCompressor.getCompressorNotConnectedFault() || mainCompressor.getCompressorShortedFault());
    SmartDashboard.putBoolean("PCM Sticky Fault", mainCompressor.getCompressorCurrentTooHighStickyFault() || mainCompressor.getCompressorNotConnectedStickyFault() || mainCompressor.getCompressorShortedStickyFault());
    SmartDashboard.putBoolean("PCM Compressor Over Current", mainCompressor.getCompressorCurrentTooHighFault());
    SmartDashboard.putBoolean("PCM Compressor Not Connected", mainCompressor.getCompressorNotConnectedFault());
    SmartDashboard.putBoolean("PCM Compressor Shorted", mainCompressor.getCompressorShortedFault());
    SmartDashboard.putBoolean("PCM Compressor Over Current [Sticky]", mainCompressor.getCompressorCurrentTooHighStickyFault());
    SmartDashboard.putBoolean("PCM Compressor Not Connected [Sticky]", mainCompressor.getCompressorNotConnectedStickyFault());
    SmartDashboard.putBoolean("PCM Compressor Shorted [Sticky]", mainCompressor.getCompressorShortedStickyFault());
    SmartDashboard.putNumber("PDP Temperature (C)", mainPDP.getTemperature());
    SmartDashboard.putNumber("PDP Current (A)", mainPDP.getTotalCurrent());
    SmartDashboard.putNumber("PDP Energy Usage (J)", mainPDP.getTotalEnergy());
    SmartDashboard.putNumber("PDP Power Usage (W)", mainPDP.getTotalPower());
   if (SmartDashboard.getBoolean("PCM Sticky Fault Reset", false)) {
      mainCompressor.clearAllPCMStickyFaults();
      SmartDashboard.putBoolean("PCM Sticky Fault Reset", false);
    }
   if (SmartDashboard.getBoolean("PDP Sticky Fault Reset", false)) {
      mainPDP.clearStickyFaults();
      SmartDashboard.putBoolean("PDP Sticky Fault Reset", false);
    }
    if (SmartDashboard.getBoolean("PDP Energy Reset", false)) {
      mainPDP.resetTotalEnergy();
      SmartDashboard.putBoolean("PDP Energy Reset", false);
    }
    if (clawPiston.get().equals(Value.kForward)) SmartDashboard.putString("Claw Status", "Retracted");
    else if (clawPiston.get().equals(Value.kReverse)) SmartDashboard.putString("Claw Status", "Extended");
    else SmartDashboard.putString("Claw Status", "Off");

    if (discPiston.get().equals(Value.kForward)) SmartDashboard.putString("Disc Status", "Retracted");
    else if (discPiston.get().equals(Value.kReverse)) SmartDashboard.putString("Disc Status", "Extended");
    else SmartDashboard.putString("Disc Status", "Off");
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
      teleopPeriodic();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    m_robotDrive.arcadeDrive(m_stick.getY(), (m_stick.getThrottle() + 1) * m_stick.getZ());
    if (m_stick.getPOV() == 0) elevatorMotor.set(1);
    else if (m_stick.getPOV() == 180) elevatorMotor.set(-1);
    else elevatorMotor.set(0);
    if (armsEjectButton.get()) armsMotor.set(1);
    else if (armsPullButton.get()) armsMotor.set(-1);
    else armsMotor.set(0);
    if (clawExtendButton.get()) clawPiston.set(Value.kReverse);
    else if (clawRetractButton.get()) clawPiston.set(Value.kForward);
    if (discGrabberExtendButton.get()) discPiston.set(Value.kForward);
    else if (discGrabberRetractButton.get()) discPiston.set(Value.kReverse);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
