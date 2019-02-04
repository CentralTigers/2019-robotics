/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.CvSink;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.NetworkButton;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private final NetworkTableInstance netInst = NetworkTableInstance.getDefault();
  private static final String kDefaultAuto = "Default";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final Spark arms = new Spark(0);
  private final Spark rightBack = new Spark(1);
  private final Spark rightFront = new Spark(2);
  private final Spark leftFront = new Spark(3);
  private final Spark leftBack = new Spark(4);
  private final Spark elevator = new Spark(5);
  private final SpeedControllerGroup leftMotors = new SpeedControllerGroup(leftBack, leftFront);
  private final SpeedControllerGroup rightMotors = new SpeedControllerGroup(rightBack, rightFront);
  private final DifferentialDrive m_robotDrive
      = new DifferentialDrive(rightMotors, leftMotors);
  private final Joystick m_stick = new Joystick(0);
  private final Timer m_timer = new Timer();
  private final Compressor mainCompressor = new Compressor(0);
  private final DoubleSolenoid clawEject = new DoubleSolenoid(0, 1);
  private final DoubleSolenoid discGrabber = new DoubleSolenoid(2, 3);
  private final NetworkButton pcmResetButton = new NetworkButton(, "PCM Sticky Fault Reset");
  //private final JoystickButton solIn = new JoystickButton(m_stick, 5);
  //private final JoystickButton solOut = new JoystickButton(m_stick, 6);
  private final AnalogInput pressureSensor = new AnalogInput(0);
  //private final DigitalInput testSwitch = new DigitalInput(0);

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
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
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    m_robotDrive.arcadeDrive(-m_stick.getY(), m_stick.getZ());
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
