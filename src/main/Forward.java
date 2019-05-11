package main;

import lejos.hardware.motor.Motor;

public class Forward {
	public static void main(String[] args) throws InterruptedException {
	
		Motor.C.setSpeed(150);
		Motor.D.setSpeed(150);
	
		Motor.C.forward();
		Motor.D.forward();
		
		Thread.sleep(8000);
		
		Motor.C.stop();
		Motor.D.stop();
		
		Motor.C.backward();
		Motor.D.backward();
		
		Thread.sleep(8000);
		
		Motor.C.stop();
		Motor.D.stop();
		
		
	}

}
