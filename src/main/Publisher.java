package main;

import lejos.hardware.lcd.LCD;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

public class Publisher {

	

	public static void main(String[] args) throws InterruptedException {
		
		EV3UltrasonicSensor ir=new EV3UltrasonicSensor(SensorPort.S3);
		EV3TouchSensor touch=new EV3TouchSensor(SensorPort.S2);
		EV3ColorSensor color =new EV3ColorSensor(SensorPort.S4);
		
		SampleProvider rangeSampler;
		rangeSampler = ir.getDistanceMode();
		float []lastRange = new float[rangeSampler.sampleSize()];
		rangeSampler.fetchSample(lastRange, 0);
		
		
		SampleProvider rangeSampler2;
		rangeSampler2 = touch.getTouchMode();
		float[]lastTouch = new float[rangeSampler2.sampleSize()];
		rangeSampler2.fetchSample(lastTouch, 0);
		

		SampleProvider rangeSampler3;
		rangeSampler3 = color.getRGBMode();
		float[]lastColor = new float[rangeSampler3.sampleSize()];
		rangeSampler3.fetchSample(lastColor, 0);
		
		
		long beginTime = System.currentTimeMillis(); // record time
		

		while(true) {
			//refresh sensor data
			rangeSampler = ir.getDistanceMode();
			rangeSampler.fetchSample(lastRange, 0);
			rangeSampler2 = touch.getTouchMode();
			rangeSampler2.fetchSample(lastTouch, 0);
			rangeSampler3 = color.getRGBMode();
			rangeSampler3.fetchSample(lastColor, 0);

			//draw data to screen
			/*
			LCD.drawString("ir sensor: " + lastRange[0],0,5);
			Thread.sleep(700);
			LCD.clear(3);
			LCD.refresh();
			*/
/*
			LCD.drawString("touch sensor: " + lastTouch[0],1,5);
			Thread.sleep(700);
			LCD.clear(3);
			LCD.refresh();

*/
		//	System.out.println("R:"+lastColor[0]+" G:"+lastColor[1]+" B:"+lastColor[2]);
			System.out.println("range"+lastRange[0]);
		/*	LCD.drawString("sensor R: " + lastColor[0],0,5);
			Thread.sleep(1000);
			LCD.drawString("sensor G: " + lastColor[1],0,5);
			Thread.sleep(1000);
			LCD.drawString("sensor B: " + lastColor[2],0,5);
			Thread.sleep(1000);
			*/
			Thread.sleep(2000);

			LCD.refresh();

			
			
			
	
			
			long nowTime = System.currentTimeMillis();
			if((nowTime - beginTime) > 90000) break;
		}
		
		ir.close();
		touch.close();
		color.close();

	}
}
