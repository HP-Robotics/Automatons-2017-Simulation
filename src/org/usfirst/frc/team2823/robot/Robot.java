package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;


public class Robot extends IterativeRobot {
	//declare objects
	RobotDrive robotDrive;
	Joystick stick;
	EncoderThread encoderThread;
	
	EncoderPIDSource xSource;
	EncoderPIDSource ySource;
	EncoderPIDSource rSource;
	
	EncoderPIDOutput xOutput;
	EncoderPIDOutput yOutput;
	EncoderPIDOutput rOutput;
	
	AdvancedPIDController xControl;
	AdvancedPIDController yControl;
	AdvancedPIDController rControl;
	
	AnalogGyro gyro;
	
	//declare constants
	//wheel PWM channels
	final int kFrontLeftChannel = 0;
	final int kFrontRightChannel = 1;
	final int kRearLeftChannel = 2;
	final int kRearRightChannel = 3;
	
	//joystick zero-sensitivity threshold
	final double kStickThreshold = 0.05;

	// The channel on the driver station that the joystick is connected to
	final int kJoystickChannel = 0;
	
	//declare variables
	//drive values
	private double driveX;
	private double driveY;
	private double driveR;
	
	double initTime;

	@Override
	public void robotInit() {
		
		stick = new Joystick(kJoystickChannel);
		
		robotDrive = new RobotDrive(kFrontLeftChannel, kRearLeftChannel, kFrontRightChannel, kRearRightChannel);
		robotDrive.setExpiration(0.1);
		
		encoderThread = new EncoderThread(this);
		encoderThread.start();
		
		xSource = new EncoderPIDSource(encoderThread, EncoderPIDSource.Axis.X);
		ySource = new EncoderPIDSource(encoderThread, EncoderPIDSource.Axis.Y);
		rSource = new EncoderPIDSource(encoderThread, EncoderPIDSource.Axis.R);
		
		xOutput = new EncoderPIDOutput(this, EncoderPIDOutput.Axis.X);
		yOutput = new EncoderPIDOutput(this, EncoderPIDOutput.Axis.Y);
		rOutput = new EncoderPIDOutput(this, EncoderPIDOutput.Axis.R);
		
		//old 0.0045, 0.000001, 0.35
		xControl = new AdvancedPIDController(0.004, 0.000001, 0.4, xSource, xOutput, 0.01);
		yControl = new AdvancedPIDController(0.004, 0.000001, 0.4, ySource, yOutput, 0.01);
		rControl = new AdvancedPIDController(0.002, 0.000001, 0.5, rSource, rOutput, 0.01);
		
		gyro = new AnalogGyro(0);
		gyro.calibrate();
	}

	@Override
	public void autonomousInit() {
		
	}

	@Override
	public void autonomousPeriodic() {
		
	}
	
	@Override
	public void teleopInit() {
		initTime = Timer.getFPGATimestamp();
		
		//xControl.setSetpoint(5000 * -27);
		//yControl.setSetpoint(5000 * -27);
		//rControl.setSetpoint(3.14 * -27);
		rControl.setSetpoint(0);
		
		xControl.enable();
		yControl.enable();
		rControl.enable();
	}

	@Override
	public void teleopPeriodic() {
		//prevent joysticks from driving robot when within a threshold value of zero
		double x = Math.abs(stick.getX()) < kStickThreshold ? 0.0 : stick.getX();
		double y = Math.abs(stick.getY()) < kStickThreshold ? 0.0 : stick.getY();
		double z = Math.abs(stick.getZ()) < kStickThreshold ? 0.0 : stick.getZ();
		
		//get gyro angle
		double t = gyro.getAngle();
		
		//update parametric PID setpoints
		//double xPoint = 1000 - Math.pow(20 * (Timer.getFPGATimestamp() - initTime) - Math.sqrt(1000), 2);
		double xPoint = 2500 / (1 + Math.pow(Math.E, -2 * ((Timer.getFPGATimestamp() - initTime) - 2.5)));
		double yPoint = (10000 / 5) * (Timer.getFPGATimestamp() - initTime);
		
		xControl.setSetpoint(xPoint * -27);
		yControl.setSetpoint(yPoint * -27);
		
		if(yPoint > 10000) {
			yControl.setSetpoint(10000 * -27);
		}
		
		/*if(xPoint < 0) {
			xControl.setSetpoint(0);
		}*/
		
		//System.out.println(t);
		//System.out.println(driveR + " " + encoderThread.getR());
		//System.out.println(encoderThread.getX() + " " + encoderThread.getY() + " | " + encoderThread.getR());

		//apply drive values to drive the robot
		//robotDrive.mecanumDrive_Cartesian(stick.getX(), stick.getY(), stick.getZ(), -t);
		robotDrive.mecanumDrive_Cartesian(driveX, driveY, driveR, -t);
	}
	
	@Override
	public void testInit() {
		
	}
	
	@Override
	public void testPeriodic() {
		
	}
	
	@Override
	public void disabledInit() {
		xControl.disable();
		yControl.disable();
		rControl.disable();
		
		robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
	}
	
	public void setDriveX(double x) {
		driveX = x;
	}
	
	public void setDriveY(double y) {
		driveY = y;
	}
	
	public void setDriveR(double r) {
		driveR = r;
	}
}
