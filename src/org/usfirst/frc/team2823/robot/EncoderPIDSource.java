package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class EncoderPIDSource implements PIDSource {
	private EncoderThread e;
	private Axis a;
	
	public enum Axis {
		X, Y, R
	}
	
	public EncoderPIDSource(EncoderThread e, Axis a) {
		this.e = e;
		this.a = a;
	}
	
	@Override
	public double pidGet() {
		switch(a) {
		case X:
			return e.getX();
		case Y:
			return e.getY();
		case R:
			return e.getR();
		default:
			return 0.0;
		}
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		return PIDSourceType.kDisplacement;
	}		
}