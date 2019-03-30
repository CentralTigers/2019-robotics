/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot; // Don't touch! FIRST required.

// Import the necessary libraries from WPILib (or other sources) here

import edu.wpi.first.wpilibj.TimedRobot; // Don't touch! FIRST required library.
import edu.wpi.first.wpilibj.Timer; // Don't touch! Controls event timing.
import edu.wpi.first.wpilibj.DoubleSolenoid.Value; // Needed for DoubleSolenoid library, see Value.kForward and Value.kReverse below.
import edu.wpi.first.wpilibj.buttons.JoystickButton; // Allows use of joystick buttons. Requires Joystick library.
import edu.wpi.first.wpilibj.smartdashboard.*; // Send and recieve data from the dashboard.
import edu.wpi.first.wpilibj.Joystick; // Allows the use of a joystick. Doesn't have to be an Extreme 3D Pro either, any USB joystick should work.
import edu.wpi.first.wpilibj.Spark; // Allows use of Spark motor controllers.
import edu.wpi.first.wpilibj.SpeedControllerGroup; // Allows motor controllers to be combined into groups. Each motor in the group will get the same signal when a group is used in place of a single controller.
import edu.wpi.first.wpilibj.drive.DifferentialDrive; // Allows use of the DifferentialDrive command. This automatically handles giving instructions to the motor controllers based on joystick input. Also available is MecanumDrive, for using Mecanum wheels automatically.
import edu.wpi.first.wpilibj.AnalogInput; // Allows reading from the analong input ports on the RoboRIO.
import edu.wpi.first.wpilibj.Compressor; // Allows manually controlling a compressor and reading PCM error codes.
import edu.wpi.first.wpilibj.DoubleSolenoid; // Allows use of double sided solenoids.
import edu.wpi.first.wpilibj.PowerDistributionPanel; // Allows reading PDP statistics to dashboard.
import edu.wpi.first.cameraserver.*; // Allows sending camera data from RoboRIO USB (or IP cameras) to dashboard automatically. DON'T use with Raspberry Pi.

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // Define global variables here! That would be motors, buttons, joysticks, etc.
  // No code can be run here though, use robotInit() for that.
  // ----------------------------------------------------------------------------------
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
  // Create a differential drive
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
  //private final JoystickButton elevatorUpButton = new JoystickButton(m_stick, 6); // OLD, don't uncomment
  //private final JoystickButton elevatorDownButton = new JoystickButton(m_stick, 4); // OLD, don't uncomment
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
  //private final DigitalInput testSwitch = new DigitalInput(0); // OLD test code, don't uncomment

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   * 
   * NOTE from Ethan: DO NOT DEFINE ANY GLOBAL VARIABLES HERE! Any variables created within a method cannot be accessed by the rest of the robot code.
   * Of course, local variables can be used to help with initialization.
   */
  @Override
  public void robotInit() {
     // Send default values to the dashboard, so there are no blank indicators
    SmartDashboard.putBoolean("PCM Sticky Fault Reset", false);
    SmartDashboard.putBoolean("PDP Sticky Fault Reset", false);
    SmartDashboard.putBoolean("PDP Energy Reset", false);
    SmartDashboard.putBoolean("Claw Status", false);
    SmartDashboard.putBoolean("Disc Status", false);
     // Start transmitting camera data to dashboard
    CameraServer.getInstance().startAutomaticCapture("Bottom Camera", 0);
    CameraServer.getInstance().startAutomaticCapture("Claw Camera", 1);
  }

  /**
   * This function is called every robot packet (all the time), no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Pressure Sensor", 250 * (pressureSensor.getVoltage() / 5) - 20); // Calculate PSI from pressure sensor voltage and send to dashboard.
    SmartDashboard.putBoolean("PCM Fault", mainCompressor.getCompressorCurrentTooHighFault() || mainCompressor.getCompressorNotConnectedFault() || mainCompressor.getCompressorShortedFault()); // If faults occur, light indicator on dashboard.
    SmartDashboard.putBoolean("PCM Sticky Fault", mainCompressor.getCompressorCurrentTooHighStickyFault() || mainCompressor.getCompressorNotConnectedStickyFault() || mainCompressor.getCompressorShortedStickyFault()); // If faults have occured previously, light indicator on dashboard.
    // Check for specific problems and light cooresponding indicator on dashboard.
    SmartDashboard.putBoolean("PCM Compressor Over Current", mainCompressor.getCompressorCurrentTooHighFault());
    SmartDashboard.putBoolean("PCM Compressor Not Connected", mainCompressor.getCompressorNotConnectedFault());
    SmartDashboard.putBoolean("PCM Compressor Shorted", mainCompressor.getCompressorShortedFault());
    SmartDashboard.putBoolean("PCM Compressor Over Current [Sticky]", mainCompressor.getCompressorCurrentTooHighStickyFault());
    SmartDashboard.putBoolean("PCM Compressor Not Connected [Sticky]", mainCompressor.getCompressorNotConnectedStickyFault());
    SmartDashboard.putBoolean("PCM Compressor Shorted [Sticky]", mainCompressor.getCompressorShortedStickyFault());
    // Send PDP statistics to dashboard
    SmartDashboard.putNumber("PDP Temperature (C)", mainPDP.getTemperature());
    SmartDashboard.putNumber("PDP Current (A)", mainPDP.getTotalCurrent());
    SmartDashboard.putNumber("PDP Energy Usage (J)", mainPDP.getTotalEnergy());
    SmartDashboard.putNumber("PDP Power Usage (W)", mainPDP.getTotalPower());
    // Respond to button presses from dashboard
   if (SmartDashboard.getBoolean("PCM Sticky Fault Reset", false)) {
      mainCompressor.clearAllPCMStickyFaults();
      SmartDashboard.putBoolean("PCM Sticky Fault Reset", false); // This resets the button to not pressed, so this code doesn't run forever.
    }
   if (SmartDashboard.getBoolean("PDP Sticky Fault Reset", false)) {
      mainPDP.clearStickyFaults();
      SmartDashboard.putBoolean("PDP Sticky Fault Reset", false); // This resets the button to not pressed, so this code doesn't run forever.
    }
    if (SmartDashboard.getBoolean("PDP Energy Reset", false)) {
      mainPDP.resetTotalEnergy();
      SmartDashboard.putBoolean("PDP Energy Reset", false); // This resets the button to not pressed, so this code doesn't run forever.
    }
    // Read piston status information and send to dashboard
    // Value.kForward and Value.kReverse are backwards on this robot
    if (clawPiston.get().equals(Value.kForward)) SmartDashboard.putBoolean("Claw Status", false);
    else if (clawPiston.get().equals(Value.kReverse)) SmartDashboard.putBoolean("Claw Status", true);
    else SmartDashboard.putBoolean("Claw Status", false);

    if (discPiston.get().equals(Value.kForward)) SmartDashboard.putBoolean("Disc Status", false);
    else if (discPiston.get().equals(Value.kReverse)) SmartDashboard.putBoolean("Disc Status", true);
    else SmartDashboard.putBoolean("Disc Status", false);
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
   * 
   * NOTE from Ethan: I deleted the case selection code, but it's included by default in blank robot projects if you need it.
   * You can use the timer in autonomousPeriodic() to space out autonomous events, so the FIRST safety watchdog doesn't shut off the robot.
   * Basically, if you use a sleep or wait command to time the code, and not the timer, the program will be shut down for not responding.
   */
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
  }

  /**
   * This function is called periodically during autonomous period.
   */
  @Override
  public void autonomousPeriodic() {
      teleopPeriodic(); // Due to the sandstorm this year allowing drivers to drive manually during autonomous, just run the teleOp code during autonomous.
  }

  /**
   * This function is called periodically during operator control / teleOp period.
   */
  @Override
  public void teleopPeriodic() {
    m_robotDrive.arcadeDrive(m_stick.getY(), ((-m_stick.getThrottle() + 1) / 2.0)  * m_stick.getZ()); /**  Read values from the joystick and drive the robot using arcade controls.
    This arcade drive has been modified to use the Z (twist on Extreme 3D Pro) axis for rotation instead of the X axis.
    Additionally, the throttle slider on the bottom of the Extreme 3D Pro has been used to adjust sensitivty of rotation.
     */
    if (m_stick.getPOV() == 0 || m_stick.getPOV() == 45 || m_stick.getPOV() == 315) elevatorMotor.set(1); // Read the POV knob on the Extreme 3D Pro. If it's up (315, 0, 45 degrees), raise elevator.
    else if (m_stick.getPOV() == 180 || m_stick.getPOV() == 135 || m_stick.getPOV() == 225) elevatorMotor.set(-1); // If it's down (135, 180, 225 degrees), lower elevator.
    else elevatorMotor.set(0); // Otherwise stop elevator (it will keep going in the last direction if this line isn't here)
    if (armsEjectButton.get()) armsMotor.set(1); // If the button to eject the ball is being held, run motors full speed in that direction
    else if (armsPullButton.get()) armsMotor.set(-1); // If the button to pull in the ball is being held, run motors full speed in that direction
    else armsMotor.set(0); // Otherwise stop the arm motors
    if (clawExtendButton.get()) clawPiston.set(Value.kReverse); // If the button to extend the claw pistons is being held, set the claw piston double solenoid to Reverse. NOTE! This actually extends the pistons on this robot. The actual direction the piston moves depends on the tubing configuration.
    else if (clawRetractButton.get()) clawPiston.set(Value.kForward); // If the button to retract the claw pistons is being held, set the claw piston double solenoid to Forward. NOTE! This actually retracts the pistons on this robot. The actual direction the piston moves depends on the tubing configuration.
    if (discGrabberExtendButton.get()) discPiston.set(Value.kForward); // Same thing as for claw pistons, except forward extends the pistons
    else if (discGrabberRetractButton.get()) discPiston.set(Value.kReverse); // Same thing as for claw pistons, except reverse retracts the pistons
  }

  /**
   * This function is called periodically during test mode.
   * 
   * NOTE from Ethan: if you wanted to make a test mode (for example, to test things in the pit or when the robot is on the transport cart) you could do that here.
   * You might want to allow the compressor to charge pneumatics but not read any input from the joystick for example, so the robot doesn't accidentally drive off the cart if someone bumps the joystick.
   */
  @Override
  public void testPeriodic() {
  }

  /** 
   * There are other methods from the TimedRobot class that can be overriden as well.
   * Examples:
   * - robotDisabled() runs when the robot is disabled
   * - teleopInit() runs when the robot first enters teleop
   * 
   * And so on. Check the documentation for the full list of methods.
   */
}
