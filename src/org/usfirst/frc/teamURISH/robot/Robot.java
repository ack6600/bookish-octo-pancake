
package org.usfirst.frc.teamURISH.robot;

import org.usfirst.frc.teamURISH.robot.commands.ExampleCommand;
import org.usfirst.frc.teamURISH.robot.subsystems.ExampleSubsystem;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	public static final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
	public static OI oi;

	private static final double P_VALUE = .5;
	private static final double I_VALUE = .02;
	private static final double D_VALUE = 0;
	private static final double F_VALUE = .5;
	
    Command autonomousCommand;
    SendableChooser chooser;
	private CANTalon talon2;
	private CANTalon followerTalon4;
	private CANTalon talon3;
	private CANTalon followerTalon1;
	private Joystick xboxController;
	private RobotDrive robotDrive;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
		oi = new OI();
        chooser = new SendableChooser();
        chooser.addDefault("Default Auto", new ExampleCommand());
//        chooser.addObject("My Auto", new MyAutoCommand());
        SmartDashboard.putData("Auto mode", chooser);
        talon2 = createTalon(2);
        talon3 = createTalon(3);
        followerTalon4 = new CANTalon(4);
        followerTalon4.setControlMode(TalonControlMode.Follower.value);
        followerTalon4.set(2);
        followerTalon1 = new CANTalon(1);
        followerTalon1.setControlMode(TalonControlMode.Follower.value);
        followerTalon1.set(3);
        xboxController = new Joystick(0);
        robotDrive = new RobotDrive(talon2,talon3);
  
    }
	
	/**
     * This function is called once each time the robot enters Disabled mode.
     * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
     */
    public void disabledInit(){

    }
	
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the getString code to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the chooser code above (like the commented example)
	 * or additional comparisons to the switch structure below with additional strings & commands.
	 */
    public void autonomousInit() {
        autonomousCommand = (Command) chooser.getSelected();
        
		/* String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
		switch(autoSelected) {
		case "My Auto":
			autonomousCommand = new MyAutoCommand();
			break;
		case "Default Auto":
		default:
			autonomousCommand = new ExampleCommand();
			break;
		} */
    	
    	// schedule the autonomous command (example)
        if (autonomousCommand != null) autonomousCommand.start();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    public void teleopInit() {
		// This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to 
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) autonomousCommand.cancel();
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
        robotDrive.setMaxOutput(256);
        robotDrive.setLeftRightMotorOutputs(xboxController.getRawAxis(1), xboxController.getRawAxis(5));
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
        LiveWindow.run();
    }
    private CANTalon createTalon(final int deviceNumber) {
        CANTalon cantalon = new CANTalon(deviceNumber);
    	cantalon.setFeedbackDevice(FeedbackDevice.QuadEncoder);

        cantalon.configNominalOutputVoltage(+0.0f, -0.0f);
        cantalon.configPeakOutputVoltage(+12.0f, -12.0f);

        cantalon.configEncoderCodesPerRev(1000);

        // keep the motor and sensor in phase
        cantalon.reverseSensor(false);

        // Soft limits can be used to disable motor drive when the sensor position
        // is outside of the limits
        cantalon.setForwardSoftLimit(10000);
        cantalon.enableForwardSoftLimit(false);
        cantalon.setReverseSoftLimit(-10000);
        cantalon.enableReverseSoftLimit(false);
        
        // brake mode: true for brake; false for coast
        cantalon.enableBrakeMode(true);
        
        // Voltage ramp rate in volts/sec (works regardless of mode)
        // 0V to 6V in one sec
        cantalon.setVoltageRampRate(6.0);

        // The allowable close-loop error whereby the motor output is neutral regardless
        // of the calculated result. When the closed-loop error is within the allowable 
        // error the PID terms are zeroed (F term remains in effect) and the integral 
        // accumulator is cleared. Value is in the same units as the closed loop error.
        // Initially make the allowable error 10% of a revolution
//        int allowableClosedLoopErr = (int) (0.1 * AMOpticalEncoderSpecs.PULSES_PER_REV);
        cantalon.setAllowableClosedLoopErr(100);

        cantalon.setProfile(0);
        cantalon.setP(P_VALUE);
        cantalon.setI(I_VALUE);
        cantalon.setD(D_VALUE);
        cantalon.setF(F_VALUE);
        cantalon.setIZone(100);
        cantalon.setControlMode(TalonControlMode.Speed.value);
        //this.talon.setCloseLoopRampRate(RAMP_RATE);
		return cantalon;
    }
    public static double scale(double x) {
        if (Math.abs(x) < 0.2) {
            return 0;
        }
        if (x > 0) {
            return (x - 0.2) / (1 - 0.2);
        }
        if (x < 0) {
            return (x + 0.2) / (1 - 0.2);
        }
        return 0;
    }
}
