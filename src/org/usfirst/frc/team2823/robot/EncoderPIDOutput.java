package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.PIDOutput;

public class EncoderPIDOutput implements PIDOutput {
	private Robot robot;
	private Axis a;
	
	public enum Axis {
		X, Y, R
	}
	
	public EncoderPIDOutput(Robot robot, Axis a) {
		this.robot = robot;
		this.a = a;
	}

	@Override
	public void pidWrite(double output) {
		switch(a) {
			case X:
				robot.setDriveX(output);
				break;
			case Y:
				robot.setDriveY(output);
				break;
			case R:
				robot.setDriveR(output);
				break;
		}
	}
}
