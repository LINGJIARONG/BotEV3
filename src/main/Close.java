package main;

import lejos.hardware.Sound;
import lejos.hardware.motor.Motor;

public class Close {
	public static void main(String[] args) throws InterruptedException {
		Motor.B.setSpeed(1200);
		Motor.B.backward();
		//Motor.A.backward();                                                                                                        
		//Motor.C.backward();

		//Sound.beep();
		Thread.sleep(1000);
		Motor.B.stop();
		Motor.B.close();
		Motor.A.stop();
		Motor.A.close();
		Motor.C.stop();
		Motor.C.close();
	
	}
}
