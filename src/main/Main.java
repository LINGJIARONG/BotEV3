package main;

import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class Main{

	static EV3UltrasonicSensor ir=new EV3UltrasonicSensor(SensorPort.S3);
	static EV3TouchSensor touch=new EV3TouchSensor(SensorPort.S2);
	static SampleProvider rangeSampler;
	static SampleProvider rangeSampler2;
	

	public static void turn360() {
		Motor.C.stop();
		Motor.D.rotate(800);//to fix
	}
	
	private static void backward() {
		Motor.C.backward();
		Motor.D.backward();
		
	}
	
	public static void forward() {
		Motor.C.forward();
		Motor.D.forward();
	}
	
	public static void openArm() throws InterruptedException {
		Motor.B.setSpeed(8000); 
		Motor.B.forward();
		Thread.sleep(400);
		Motor.B.stop(true);
	}
	
	public static void closeArm() throws InterruptedException {
		Motor.B.setSpeed(8000);
		Motor.B.backward();
		Thread.sleep(400);
		Motor.B.stop(true);
	}
	
	private static void updateRange(EV3UltrasonicSensor ir, float[] lastRange) {
		rangeSampler = ir.getDistanceMode();
		rangeSampler.fetchSample(lastRange, 0);
	}
	

	private static void updateTouch(EV3TouchSensor touch2, float[] lastTouch) {
		rangeSampler2 = touch2.getTouchMode();
		rangeSampler2.fetchSample(lastTouch, 0);

	}
	
	
	public static void main(String[] args) throws InterruptedException {

		try {

			rangeSampler = ir.getDistanceMode();
			float []lastRange = new float[rangeSampler.sampleSize()];
			rangeSampler.fetchSample(lastRange, 0);
			SampleProvider rangeSampler2;
			rangeSampler2 = touch.getTouchMode();

			float[]lastTouch = new float[rangeSampler2.sampleSize()];
			rangeSampler2.fetchSample(lastTouch, 0);
			long beginTime = System.currentTimeMillis();


			forward();
			if(lastRange[0]<=5) {
				openArm();
			}
			while(true) {
				updateRange(ir,lastRange);
				updateTouch(touch,lastTouch);
				LCD.drawString("ir sensor: " + lastRange[0],0,5);
				LCD.refresh();			
				if(lastTouch[0]==1) {
					closeArm();
					turn360();
					forward();
					Delay.msDelay(1000);
					openArm();
					Delay.msDelay(1000);
					backward();
					Delay.msDelay(1000);
					turn360();
					closeArm();
					break;
				}


				LCD.drawString("Ultra sensor : " + lastRange[0],0,5);
				LCD.refresh();


				long nowTime = System.currentTimeMillis();
				if((nowTime - beginTime) > 20000) break;

			}
		}finally{

			Motor.C.forward();
			Motor.D.forward();
			Thread.sleep(1000);

			Motor.C.stop(true);
			Motor.D.stop(true);
			
			Motor.C.close();
			Motor.D.close();
			ir.close();
			touch.close();

		}

	}

	



}

