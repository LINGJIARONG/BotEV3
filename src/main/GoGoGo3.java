package main;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;


public class GoGoGo3 {
	public static final int turnSpeed = 150;//-> turn with this speed to search object
	public static final long ms2turn = 1150; // ms to turn while in charge 
	public static final long ms2turnEmpty = 800; //ms to turn while empty
	//inner class because outer class doesn't work
	public class EV3Helper {
		public String color1;
		public String color2;
		private  final double ROTATE_DEGREES_FACTOR = 11.3; //How many degrees one motor has to rotate for the robot to rotate 1 degree (while the other motor is stopped)
		private  final int DEFAULT_MOTOR_SPEED = 700;

		//WHITE
		private  final float WHITE_R_MIN= 0.11f;
		private  final float WHITE_R_MAX=0.19f;
		private  final float WHITE_G_MIN=0.11f;
		private  final float WHITE_G_MAX=0.19f;
		private  final float WHITE_B_MIN=0.08f;
		private  final float WHITE_B_MAX=0.14f;

		//RED
		private  final float RED_R_MIN= 0.08f;
		private  final float RED_R_MAX=0.1f;
		private  final float RED_G_MIN=0.01f;
		private  final float RED_G_MAX=0.03f;
		private  final float RED_B_MIN=0.01f;
		private  final float RED_B_MAX=0.02f;

		//GREEN
		private  final float GREEN_R_MIN=0.02f;	
		private  final float GREEN_R_MAX=0.05f;
		private  final float GREEN_G_MIN=0.05f;
		private  final float GREEN_G_MAX=0.09f;
		private  final float GREEN_B_MIN=0.01f;
		private  final float GREEN_B_MAX=0.03f;

		//YELLOW
		private  final float YELLOW_R_MIN=0.16f;
		private  final float YELLOW_R_MAX=0.19f;
		private  final float YELLOW_G_MIN=0.13f;
		private  final float YELLOW_G_MAX=0.16f;
		private  final float YELLOW_B_MIN=0.01f;
		private  final float YELLOW_B_MAX=0.04f;

		//BLACK
		private final float BLACK_R_MIN=0.01f;
		private final float BLACK_R_MAX=0.018f;
		private final float BLACK_G_MIN=0.01f;
		private final float BLACK_G_MAX=0.02f;
		private final float BLACK_B_MIN=0f;
		private final float BLACK_B_MAX=0.02f;

		//GREY
		private final float GREY_R_MIN=0.05f;
		private final float GREY_R_MAX=0.09f;
		private final float GREY_G_MIN=0.05f;
		private final float GREY_G_MAX=0.09f;
		private final float GREY_B_MIN=0.03f;
		private final float GREY_B_MAX=0.06f;

		private RegulatedMotor motorRight;
		private RegulatedMotor motorLeft;
		private RegulatedMotor motorPince;

		
		private EV3UltrasonicSensor ultrasonicSensor;
		private EV3ColorSensor colorSensor;
		private EV3TouchSensor touchSensor;

		private SampleProvider colorSampler;
		private SampleProvider rangeSampler;
		private SampleProvider touchSampler;

		private float[] lastRange;
		private float[] lastColor;
		private float[] lastTouch;
		/**
		 *  arm isOpen or not ?
		 */
		private boolean open;

		private long startTime=System.currentTimeMillis();   //startTime
		private long endTime=0l;//end Time 
		/**
		 * not used . 
		 */
		public long realTime;	

		public int caught=0;





		/**
		 * Instantiates a new helper class and optionally calibrates the cannon.
		 * Robot is ready for operation after completion.
		 * @param skipMotorCannonCalibration Set to true if you don't want to calibrate the cannon
		 */
		public EV3Helper() {
			open=false;
			motorRight = new EV3LargeRegulatedMotor(MotorPort.C);
			motorLeft = new EV3LargeRegulatedMotor(MotorPort.D);
			motorPince=new EV3LargeRegulatedMotor(MotorPort.B);
			//Sets default motor speed for driving motors
			motorRight.setSpeed(DEFAULT_MOTOR_SPEED);
			motorLeft.setSpeed(DEFAULT_MOTOR_SPEED);

			ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S3);
			colorSensor = new EV3ColorSensor(SensorPort.S4);
			touchSensor=new EV3TouchSensor(SensorPort.S2);

			rangeSampler = ultrasonicSensor.getDistanceMode();
			lastRange = new float[rangeSampler.sampleSize()];

			touchSampler=touchSensor.getTouchMode();
			lastTouch= new float[touchSampler.sampleSize()];


			colorSampler = colorSensor.getRGBMode();
			lastColor = new float[colorSampler.sampleSize()];

			updateRange(ultrasonicSensor, lastRange);
			updateTouch(touchSensor, lastTouch);
			updateColor(colorSensor, lastColor);
		}

		public RegulatedMotor getMotorRight() {
			return motorRight;
		}

		public RegulatedMotor getMotorLeft() {
			return motorLeft;
		}




		public   double getRotateDegreesFactor() {
			return ROTATE_DEGREES_FACTOR;
		}



		public  int getDefaultMotorSpeed() {
			return DEFAULT_MOTOR_SPEED;
		}

		public RegulatedMotor getMotorPince() {
			return motorPince;
		}

		public EV3UltrasonicSensor getUltrasonicSensor() {
			return ultrasonicSensor;
		}

		public EV3ColorSensor getColorSensor() {
			return colorSensor;
		}

		public EV3TouchSensor getTouchSensor() {
			return touchSensor;
		}

		public SampleProvider getRangeSampler() {
			return rangeSampler;
		}

		public float[] getLastRange() {
			return lastRange;
		}

		/**
		 * Fetches a distance sample from the EV3 infrared sensor.
		 * Close to centimeters.
		 * @return Distance from sensor in something that's quite close to centimeters
		 */
		public float getDistance() {
			rangeSampler.fetchSample(lastRange, 0);
			return lastRange[0];
		}
		/**
		 * 
		 * @param r
		 * @param g
		 * @param b
		 * @return
		 */
		public boolean isWHITE(float r,float g,float b) {
			return r<=WHITE_R_MAX&&r>=WHITE_R_MIN
					&&g<=WHITE_G_MAX&&g>=WHITE_G_MIN
					&&b<=WHITE_B_MAX&&b>=WHITE_B_MIN;
		}

		/**
		 * 
		 * @param r
		 * @param g
		 * @param b
		 * @return
		 */
		public boolean isRED(float r,float g,float b) {
			return r<=RED_R_MAX&&r>=RED_R_MIN
					&&g<=RED_G_MAX&&g>=RED_G_MIN
					&&b<=RED_B_MAX&&b>=RED_B_MIN;
		}

		/**
		 * 
		 * @param r
		 * @param g
		 * @param b
		 * @return
		 */
		public boolean isBLACK(float r,float g,float b) {
			return r<=BLACK_R_MAX&&r>=BLACK_R_MIN
					&&g<=BLACK_G_MAX&&g>=BLACK_G_MIN
					&&b<=BLACK_B_MAX&&b>=BLACK_B_MIN;
		}

		/**
		 * 
		 * @param r
		 * @param g
		 * @param b
		 * @return
		 */
		public boolean isGREEN(float r,float g,float b) {
			return r<=GREEN_R_MAX&&r>=GREEN_R_MIN
					&&g<=GREEN_G_MAX&&g>=GREEN_G_MIN
					&&b<=GREEN_B_MAX&&b>=GREEN_B_MIN;
		}

		/**
		 * 
		 * @param r
		 * @param g
		 * @param b
		 * @return
		 */
		public boolean isYELLOW(float r,float g,float b) {
			return r<=YELLOW_R_MAX&&r>=YELLOW_R_MIN
					&&g<=YELLOW_G_MAX&&g>=YELLOW_G_MIN
					&&b<=YELLOW_B_MAX&&b>=YELLOW_B_MIN;
		}

		/**
		 * 
		 * @param r
		 * @param g
		 * @param b
		 * @return
		 */
		public boolean isGREY(float r,float g,float b) {
			return r<=GREY_R_MAX&&r>=GREY_R_MIN
					&&g<=GREY_G_MAX&&g>=GREY_G_MIN
					&&b<=GREY_B_MAX&&b>=GREY_B_MIN;
		}

		/**
		 * Fetches a color sample from the EV3 color sensor.
		 * @return name of measured color
		 */
		public String getColorName() {
			colorSampler = colorSensor.getRGBMode();
			colorSampler.fetchSample(lastColor, 0);
			float r=lastColor[0];
			float g=lastColor[1];
			float b=lastColor[2];
			if(isWHITE(r, g, b))
				return "white";
			else if(isBLACK(r, g, b))
				return "black";
			else if(isGREEN(r, g, b))
				return "green";
			else if(isRED(r, g, b))
				return "red";
			else if(isYELLOW(r, g, b))
				return "yellow";
			else if(isGREY(r, g, b))
				return "grey";
			else 
				return "unknown";
		}


		/**
		 * Drives forward until stop is called.
		 * will speed 'DEFAULT_MOTOR_SPEED'
		 * Returns immediately
		 * initialize startTime
		 */
		public void forward(){
			startTime=System.currentTimeMillis(); //start time 
			motorLeft.setSpeed(DEFAULT_MOTOR_SPEED-50);
			motorRight.setSpeed(DEFAULT_MOTOR_SPEED);
			motorLeft.forward();
			motorRight.forward();

		}


		/**
		 * Drives backward until stop is called.
		 * Returns immediately
		 * initialize startTime
		 */
		public void backward(){		
			startTime=System.currentTimeMillis(); //start time 
			motorLeft.setSpeed(DEFAULT_MOTOR_SPEED-50);
			motorRight.setSpeed(DEFAULT_MOTOR_SPEED);
			motorLeft.backward();
			motorRight.backward();
		}


		/**
		 * Stops both motors immediately
		 * calculate the run time 
		 */
		public void stop(){
			motorLeft.stop(true);
			motorRight.stop(true);
			endTime=System.currentTimeMillis();
			realTime=endTime-startTime;
		}
	
		/**
		 * keep turning Left 
		 * initialize startTime
		 */
		public void turnLeft() {
			startTime=System.currentTimeMillis(); //start time 
			motorRight.setSpeed(turnSpeed);
			motorLeft.stop(true);
			motorRight.forward();
		}

		/**
		 *!!! turn left 180 degrees while in charge!!
		 * 
		 */
		public void turnLeft180() {
			long time=System.currentTimeMillis(); //start time 
			motorLeft.stop(true);
			motorRight.setSpeed(3100);
			motorRight.forward();
			long time2=System.currentTimeMillis();
			while( ( time2-time)<=ms2turn) {
				time2=System.currentTimeMillis();
				System.out.println("time2"+time2+"\nms2turn"+ms2turn);
			}
			motorRight.stop(true);
		}
		
		/**
		 *!!! turn left 180 degrees while in charge!!
		 *  
		 */
		public void turnLeft100() {
			long time=System.currentTimeMillis(); //start time 
			motorLeft.stop(true);
			motorRight.setSpeed(3000);
			motorRight.forward();
			long time2=System.currentTimeMillis();
			while( ( time2-time)<=ms2turn-150) {
				time2=System.currentTimeMillis();
				
			}
			motorRight.stop(true);
		}

		/**
		 * function used when finish placing-> reset position to detect another object 
		 * turn left 180 degrees without charge !
		 * not necessarily 180 degrees.
		 */
		public void turnLeft180Empty() {
			long time=System.currentTimeMillis(); //start time 
			motorLeft.stop(true);
			motorRight.setSpeed(1200);
			motorRight.forward();
			long time2=System.currentTimeMillis();
			while( ( time2-time)<=ms2turnEmpty) {
				time2=System.currentTimeMillis();
			}
			motorRight.stop(true);
		}
		/**
		 * function used when finish placing-> reset position to detect another object 
		 * turn left 180 degrees without charge !
		 * not necessarily 180 degrees.
		 */
		public void turnLeft180Empty(long fit) {
			long time=System.currentTimeMillis(); //start time 
			motorLeft.stop(true);
			motorRight.setSpeed(1200);
			motorRight.forward();
			long time2=System.currentTimeMillis();
			while( ( time2-time)<=fit) {
				time2=System.currentTimeMillis();
			}
			motorRight.stop(true);
		}

		/**
		 * Turns right the given number of degrees
		 * @param degrees
		 * @throws InterruptedException 
		 */
		public void turnRight(int degrees) throws InterruptedException {
			motorLeft.setSpeed(1200);
			motorRight.stop(true);
			Thread.sleep(degrees);
			forward();
		}
		
		public boolean Normal() {
			updateSensors();
			return lastRange[0]>=0.28;
			
		}
		

		/**
		 * keep turning right 
		 */
		public void turnRight() {
			startTime=System.currentTimeMillis(); //start time 
			motorRight.stop(true);
			motorLeft.forward();
		}

		/**
		 *  open arm if it's not open
		 * @throws InterruptedException
		 */
		public void openArm() throws InterruptedException {
			if(open)
				return;
			open=true;
			motorPince.setSpeed(8000); 
			motorPince.forward();
			Thread.sleep(480);

			motorPince.stop(true);
		}

/**
 *  close arm
 * @throws InterruptedException
 */
		public void closeArm() throws InterruptedException {
			if(!open)
				return;
			open=false;
			motorPince.setSpeed(8000);
			motorPince.backward();
			Thread.sleep(480);
			motorPince.stop(true);
		}


		private void updateRange(EV3UltrasonicSensor ir, float[] lastRange) {
			rangeSampler = ir.getDistanceMode();
			rangeSampler.fetchSample(lastRange, 0);
		}


		private void updateTouch(EV3TouchSensor touch2, float[] lastTouch) {
			touchSampler = touch2.getTouchMode();
			touchSampler.fetchSample(lastTouch, 0);

		}

		private void updateColor(EV3ColorSensor color, float[] lastColor) {
			colorSampler = color.getRGBMode();
			colorSampler.fetchSample(lastColor, 0);
		}
		/**
		 * update all sensors
		 */
		public void updateSensors() {
			updateRange(ultrasonicSensor, lastRange);
			updateTouch(touchSensor, lastTouch);
			updateColor(colorSensor, lastColor);
		}

		public   float getRedBMax() {
			return RED_B_MAX;
		}

		public   float getGreenRMin() {
			return GREEN_R_MIN;
		}

		public   float getGreenRMax() {
			return GREEN_R_MAX;
		}

		public   float getGreenGMin() {
			return GREEN_G_MIN;
		}

		public   float getGreenGMax() {
			return GREEN_G_MAX;
		}

		public   float getGreenBMin() {
			return GREEN_B_MIN;
		}

		public   float getGreenBMax() {
			return GREEN_B_MAX;
		}

		public   float getBlackRMin() {
			return BLACK_R_MIN;
		}

		public   float getBlackRMax() {
			return BLACK_R_MAX;
		}

		public  float getBlackGMin() {
			return BLACK_G_MIN;
		}

		public  float getBlackGMax() {
			return BLACK_G_MAX;
		}

		public  float getBlackBMin() {
			return BLACK_B_MIN;
		}

		public  float getBlackBMax() {
			return BLACK_B_MAX;
		}

		public   float getGreyRMin() {
			return GREY_R_MIN;
		}

		public   float getGreyRMax() {
			return GREY_R_MAX;
		}

		public   float getGreyGMin() {
			return GREY_G_MIN;
		}

		public   float getGreyGMax() {
			return GREY_G_MAX;
		}

		public   float getGreyBMin() {
			return GREY_B_MIN;
		}

		public   float getGreyBMax() {
			return GREY_B_MAX;
		}

/**
 * 
 * @return
 */
		public float[] getLastColor() {
			return lastColor;
		}
		/**
		 * 
		 * @return touch sensor value: 0/1-> empty/onTouch
		 */
		public float[] getLastTouch() {
			return lastTouch;
		}
		public void waitForPress() {
			Button.waitForAnyPress();
		}
		/**
		 * close All motors
		 */
		public void closeMotor() {
			motorRight.close();
			motorLeft.close();
			motorPince.close();
		}
		/**
		 * close all sensors
		 */
		public void closeSensor() {
			colorSensor.close();
			touchSensor.close();
			ultrasonicSensor.close();
		}

		public void decale() throws InterruptedException {
			motorRight.stop();
			motorLeft.forward();
			Thread.sleep(400); 
			forward();
			Thread.sleep(500); 
			motorLeft.stop();
			motorRight.forward();
			Thread.sleep(100); 
			forward();

		}



	}

	/**
	 * necessary time to return the right angle 
	 */
	static long degree=0;
	/**
	 * necessary time to return to the initial postion 
	 */
	static long pos;

	static long recul=355;//backward time to leave object alone

	public static void main(String[] args) throws InterruptedException {
		//to skip first return home function()
		int setHome=0;
		EV3Helper ev3=new GoGoGo3().new EV3Helper();
		
		float dis;//result of ultrasonic sensor 
		boolean obj=false;
		float DIS_MAX = 1.0f;
		//to control the total runtime
		ev3.waitForPress();
		try {
			while(true) {//to find bricks : just turn left forever 

				ev3.updateSensors();
				dis=ev3.getLastRange()[0];
				if(ev3.caught<=2)
					DIS_MAX= 0.75f;
				else
					DIS_MAX=1.3f;
				//search mode 
				if(dis>=DIS_MAX||dis<=0.3) { //not in range -> turn left to search brick
					ev3.turnLeft(); // initial start time to locate direction
					obj=true;
				}
				System.out.println("distance : "+dis); 
				
				//catch mode
				if(dis<DIS_MAX&&dis>0.3) {//consider as a brick 
					ev3.stop(); //don't turn left anymore 	
					Sound.twoBeeps(); //found it!!!!!				
					ev3.forward();//go and catch it , initial a start time 
					ev3.openArm();
					long time=System.currentTimeMillis();//control if >3500ms still not catching anything
					while(ev3.getLastTouch()[0]!=1) {
						ev3.updateSensors();	
						dis=ev3.getLastRange()[0];
						if(dis>=0.3&&obj) {//regulation to left
							ev3.stop();
							obj=false;
							ev3.turnLeft();
							
							int factor=350;
							if(dis>=0.5)
								factor=500;
							System.out.println("factor:\n"+factor);
							Thread.sleep(factor);
							ev3.stop();
							ev3.forward();
							
						}
						ev3.updateSensors();	
						dis=ev3.getLastRange()[0];
						if(dis<=0.2) {//wall or a robot dectected
							ev3.stop();
							Sound.beepSequenceUp();
							Thread.sleep(1000);
							ev3.updateSensors();	
							dis=ev3.getLastRange()[0];
							if(dis>0.28)
								break;
							Sound.beepSequenceUp();
							ev3.turnLeft100();
							degree++;//help to find way home
							ev3.updateSensors();
							break;
						}
						long timeControl=System.currentTimeMillis();
						if(timeControl-time>=3700)
							break;
					}
					
					obj=true;
				}
				
				ev3.updateSensors();
				//touch ==1-> 2 cases: 1) go to destination(the first time catch)
				// 2)return to destination(the second time or later)
				if(ev3.getLastTouch()[0]==1) {
					ev3.caught++;
					Sound.buzz();//ok 
					ev3.stop();
					ev3.closeArm();
					
					if(setHome==0)	
						ev3.decale();
					ev3.forward();
					
					if(setHome==0) {
						while(!"white".equals(ev3.getColorName())) {
							ev3.updateSensors(); //forward until meet while line (destination)
						}
						ev3.stop();
						//place object
						ev3.openArm();
						Sound.beepSequenceUp();
						ev3.backward();
						Delay.msDelay(recul);
						ev3.stop();
						ev3.closeArm();
						//turn back
						ev3.turnLeft180Empty();
						//time +1
						setHome++;
						continue;
					}
					
					//don't go home 
					//go to destination first and home is there!!!
					else
						goHome(ev3);

				}

			}
		}catch(Exception e) {
		
		}
		finally {
			ev3.closeMotor();
			ev3.closeSensor();
		}


	}
	/** 
	 *  try to go home and place down 
	 * @param e
	 * @throws InterruptedException
	 */
	private static void goHome(EV3Helper e) throws InterruptedException {
		e.turnLeft180();
		e.stop();
		e.forward();
		long time=System.currentTimeMillis();
		//forward until white line 
		while(!"white".equals(e.getColorName())) {
			e.updateSensors();
			if(e.lastRange[0]<=0.27)
				e.turnRight(700);
			e.forward();
			e.updateSensors();
			if("green".equals(e.getColorName())&&System.currentTimeMillis()-time>=2000) 
				e.turnLeft180Empty(ms2turnEmpty);
		}
		e.stop();		
	

		e.openArm();
		Sound.beepSequenceUp();
		e.backward();
		if(e.caught>=3)
			recul=1111;
		if(e.caught>=5)
			recul=1800;
		Delay.msDelay(recul);
		e.stop();
		e.closeArm();
		if(e.caught==1)
		e.turnLeft180Empty(ms2turnEmpty+200);
		else if(e.caught==2)
		e.turnLeft180Empty(ms2turnEmpty+400);
		
		else
		e.turnLeft180Empty(ms2turnEmpty+50);
		continu(e);


	}

	private static void continu(EV3Helper e) {


	}
	
	public static void after(EV3Helper e) {
		
	}

}
