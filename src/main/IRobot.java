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
/**
 * 
 * 
 * contains two inner classes:   </br>
 * 1.EV3Helper (to define some system parameters and basic methods)
 * <ol>
 * <li>basic methods to simplify code of stategy in main</li>
 * <li>use Ev3 Api </li>
 * <li>system parameters invisible in other class, calibrated and only visible in this class</li>
 * <li>main used these methods to make strategeis</li>
 * </ol>
 * 
 * 
 * 2.main (run the system, define strategies)
 * <ol>
 * <li>strategies and learning</li>
 * <li>use Ev3Helper </li>
 * <li>run robot </li>
 * </ol>
 * @author Group E2
 */

public class IRobot {
	/**
	 * turn with this speed to search object in search mode, 
	 * if it's too fast, risk missing object,
	 * if it's too slow, lose time
	 */
	public static final int turnSpeed = 50;
	/**
	 *  time (millisecond) to turn back while in charge ,
	 *  call every time when we catch an object(suppose the destination is backwards)
	 */
	public static final long ms2turn = 1100; 
	/**
	 * time (millisecond) to turn back while empty ,
	 * call every time when we place down an object(suppose other objects is backwards)
	 */
	public static final long ms2turnEmpty = 900; 
	/**
	 * 
	 * @author E2 Group
	 * define basic methods such as turn left,forwards,turn right...
	 * define system parameters such as default speed, RGB factor for every colors
	 */
	public class EV3Helper {
		/**
		 * How many degrees one motor has to rotate for the robot to rotate 1 degree  (while the other motor is stopped)
		 *  
		 *  ,didn't use for some reasons
		 */
		private  final double ROTATE_DEGREES_FACTOR = 11.3; 
		/**
		 * default speed, attention: the max speed is 100*(voltage of battery),
		 * so to be stable(same moving speed every run), we set to 650.</br>
		 * if too slow , we won't have bonus point 
		 * if too fast , we can't go straight because of the difference between the left motor and the right motor
		 */
		private  final int DEFAULT_MOTOR_SPEED = 650;
		/**
		 * Minimum R factor to considerate as white
		 */
		private  final float WHITE_R_MIN= 0.11f;
		/**
		 * Maximum R factor to considerate as white
		 */
		private  final float WHITE_R_MAX=0.19f;
		/**
		 * Minimum G factor to considerate as white
		 */
		private  final float WHITE_G_MIN=0.11f;
		/**
		 * Maximum G factor to considerate as white
		 */
		private  final float WHITE_G_MAX=0.19f;
		/**
		 * Minimum B factor to considerate as white
		 */
		private  final float WHITE_B_MIN=0.08f;
		/**
		 * Maximum B factor to considerate as white
		 */
		private  final float WHITE_B_MAX=0.14f;

		//RED
		/**
		 * Minimum R factor to considerate as red
		 */
		private  final float RED_R_MIN= 0.08f;
		/**
		 * Maximum R factor to considerate as red
		 */
		private  final float RED_R_MAX=0.1f;
		/**
		 * Minimum G factor to considerate as red
		 */
		private  final float RED_G_MIN=0.01f;
		/**
		 * Maximum G factor to considerate as red
		 */
		private  final float RED_G_MAX=0.03f;
		/**
		 * Minimum B factor to considerate as red
		 */
		private  final float RED_B_MIN=0.01f;
		/**
		 *  Maximum B factor to considerate as red
		 */
		private  final float RED_B_MAX=0.02f;

		//GREEN
		/**
		 * Minimum R factor to considerate as green
		 */
		private  final float GREEN_R_MIN=0.02f;	
		/**
		 *  Maximum R factor to considerate as green
		 */
		private  final float GREEN_R_MAX=0.05f;
		/**
		 * Minimum G factor to considerate as green
		 */
		private  final float GREEN_G_MIN=0.05f;
		/**
		 *  Maximum G factor to considerate as green
		 */
		private  final float GREEN_G_MAX=0.09f;
		/**
		 * Minimum B factor to considerate as green
		 */
		private  final float GREEN_B_MIN=0.01f;
		/**
		 *  Maximum B factor to considerate as green
		 */
		private  final float GREEN_B_MAX=0.03f;

		//YELLOW
		/**
		 * Minimum R factor to considerate as yellow
		 */
		private  final float YELLOW_R_MIN=0.16f;
		/**
		 *  Maximum R factor to considerate as yellow
		 */
		private  final float YELLOW_R_MAX=0.19f;
		/**
		 * Minimum G factor to considerate as yellow
		 */
		private  final float YELLOW_G_MIN=0.13f;
		/**
		 *  Maximum G factor to considerate as yellow
		 */
		private  final float YELLOW_G_MAX=0.16f;
		/**
		 * Minimum B factor to considerate as yellow
		 */
		private  final float YELLOW_B_MIN=0.01f;
		/**
		 *  Maximum B factor to considerate as yellow
		 */
		private  final float YELLOW_B_MAX=0.04f;

		//BLACK
		/**
		 * Minimum R factor to considerate as black
		 */
		private final float BLACK_R_MIN=0.01f;
		/**
		 *  Maximum R factor to considerate as black
		 */
		private final float BLACK_R_MAX=0.018f;
		/**
		 * Minimum G factor to considerate as black
		 */
		private final float BLACK_G_MIN=0.01f;
		/**
		 *  Maximum G factor to considerate as black
		 */
		private final float BLACK_G_MAX=0.02f;
		/**
		 * Minimum B factor to considerate as black
		 */
		private final float BLACK_B_MIN=0f;
		/**
		 *  Maximum B factor to considerate as black
		 */
		private final float BLACK_B_MAX=0.02f;

		//GREY
		/**
		 * Minimum R factor to considerate as grey
		 */
		private final float GREY_R_MIN=0.05f;
		/**
		 *  Maximum R factor to considerate as grey
		 */
		private final float GREY_R_MAX=0.09f;
		/**
		 * Minimum G factor to considerate as grey
		 */
		private final float GREY_G_MIN=0.05f;
		/**
		 *  Maximum G factor to considerate as grey
		 */
		private final float GREY_G_MAX=0.09f;
		/**
		 * Minimum B factor to considerate as grey
		 */
		private final float GREY_B_MIN=0.03f;
		/**
		 *  Maximum B factor to considerate as grey
		 */
		private final float GREY_B_MAX=0.06f;

		//Blue
		/**
		 * Minimum R factor to considerate as blue
		 */
		private final float BLUE_R_MIN=0f;
		/**
		 *  Maximum R factor to considerate as blue
		 */
		private final float BLUE_R_MAX=0.04f;
		/**
		 * Minimum G factor to considerate as blue
		 */
		private final float BLUE_G_MIN=0.05f;
		/**
		 *  Maximum G factor to considerate as blue
		 */
		private final float BLUE_G_MAX=0.08f;
		/**
		 * Minimum B factor to considerate as blue
		 */
		private final float BLUE_B_MIN=0.05f;
		/**
		 *  Maximum B factor to considerate as blue
		 */
		private final float BLUE_B_MAX=0.08f;

		/**
		 * right motor : to do an action on the right motor 
		 */
		private RegulatedMotor motorRight;
		/**
		 * left motor: to do an action on the left motor 
		 */
		private RegulatedMotor motorLeft;
		/**
		 * Arm motor: to do an action on arm 
		 */
		private RegulatedMotor motorPince;
		/**
		 * Sensor: to detect distance
		 */
		private EV3UltrasonicSensor ultrasonicSensor;
		/**
		 * Sensor: to detect color
		 */
		private EV3ColorSensor colorSensor;
		/**
		 * Sensor: to detect touching
		 */
		private EV3TouchSensor touchSensor;
		/**
		 * store color sample
		 */
		private SampleProvider colorSampler;
		/**
		 * store distance sample
		 */
		private SampleProvider rangeSampler;
		/**
		 * store touching sample
		 */
		private SampleProvider touchSampler;

		/**
		 * lastRange[0]-> newest ultrasonic sensor data
		 */
		private float[] lastRange;
		/**
		 * lastColor[0]-> newest color sensor data
		 */
		private float[] lastColor;
		/**
		 * lastTouch[0]-> newest touching sensor data
		 */
		private float[] lastTouch;
		/**
		 *  Arm is Open or not ?
		 *  called before open arm to avoid open twice 
		 *  called before close arm to avoid close twice 
		 */
		private boolean open;
		/**
		 * restore start time ,
		 * called when run 
		 */
		private long startTime=System.currentTimeMillis();   //startTime
		/**
		 * restore end time ,
		 * called when stop 
		 */
		private long endTime=0l;//end Time 
		/**
		 * endTime-startTime: to calculate the distance that we run 
		 */
		public long realTime;	
		/**
		 * How many objects already collected,
		 * to make different strategies according to different number 
		 */
		public int caught=0;

		/**
		 * Instantiates a new helper class and calibrates the cannon.
		 * Robot is ready for operation after completion.
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
		//getters 
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
		 * Fetches a distance sample from the EV3 ultrasonic sensor.
		 * @return Distance from sensor in something that's quite close to centimeters
		 */
		public float getDistance() {
			rangeSampler.fetchSample(lastRange, 0);
			return lastRange[0];
		}

		/**
		 * 
		 * @param r current factor R
		 * @param g current factor G
		 * @param b current factor B
		 * @return true if it's white , false if not 
		 */
		public boolean isWHITE(float r,float g,float b) {
			return r<=WHITE_R_MAX&&r>=WHITE_R_MIN
					&&g<=WHITE_G_MAX&&g>=WHITE_G_MIN
					&&b<=WHITE_B_MAX&&b>=WHITE_B_MIN;
		}

		/**
		 * 
		 * @param r current factor R
		 * @param g current factor G
		 * @param b current factor B
		 * @return true if it's red , false if not 
		 */
		public boolean isRED(float r,float g,float b) {
			return r<=RED_R_MAX&&r>=RED_R_MIN
					&&g<=RED_G_MAX&&g>=RED_G_MIN
					&&b<=RED_B_MAX&&b>=RED_B_MIN;
		}

		/**
		 * 
		 * @param r current factor R
		 * @param g current factor G
		 * @param b current factor B
		 * @return true if it's black , false if not 
		 */
		public boolean isBLACK(float r,float g,float b) {
			return r<=BLACK_R_MAX&&r>=BLACK_R_MIN
					&&g<=BLACK_G_MAX&&g>=BLACK_G_MIN
					&&b<=BLACK_B_MAX&&b>=BLACK_B_MIN;
		}

		/**
		 * 
		 * @param r current factor R
		 * @param g current factor G
		 * @param b current factor B
		 * @return true if it's blue , false if not 
		 */
		public boolean isBLUE(float r,float g,float b) {
			return r<=BLUE_R_MAX&&r>=BLUE_R_MIN
					&&g<=BLUE_G_MAX&&g>=BLUE_G_MIN
					&&b<=BLUE_B_MAX&&b>=BLUE_B_MIN;
		}

		/**
		 * 
		 * @param r current factor R
		 * @param g current factor G
		 * @param b current factor B
		 * @return true if it's green , false if not 
		 */
		public boolean isGREEN(float r,float g,float b) {
			return r<=GREEN_R_MAX&&r>=GREEN_R_MIN
					&&g<=GREEN_G_MAX&&g>=GREEN_G_MIN
					&&b<=GREEN_B_MAX&&b>=GREEN_B_MIN;
		}

		/**
		 * 
		 * @param r current factor R
		 * @param g current factor G
		 * @param b current factor B
		 * @return true if it's yellow , false if not 
		 */
		public boolean isYELLOW(float r,float g,float b) {
			return r<=YELLOW_R_MAX&&r>=YELLOW_R_MIN
					&&g<=YELLOW_G_MAX&&g>=YELLOW_G_MIN
					&&b<=YELLOW_B_MAX&&b>=YELLOW_B_MIN;
		}

		/**
		 * 
		 * @param r current factor R
		 * @param g current factor G
		 * @param b current factor B
		 * @return true if it's grey , false if not 
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
			else if(isBLUE(r,g,b))
				return "blue";
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
			motorLeft.setSpeed(DEFAULT_MOTOR_SPEED-5);
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
			motorLeft.setSpeed(DEFAULT_MOTOR_SPEED-5);
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
			motorRight.setSpeed(turnSpeed+100);
			motorLeft.stop(true);
			motorRight.forward();
		}

		/**
		 *!!! turn left 180 degrees while in charge!!
		 * ok calibrated according to surface.
		 */
		public void turnLeft180() {
			long time=System.currentTimeMillis(); //start time 
			motorLeft.stop(true);
			motorRight.setSpeed(600);
			motorRight.forward();
			long time2=System.currentTimeMillis();
			while( ( time2-time)<=ms2turn) {
				time2=System.currentTimeMillis();
			}
			motorRight.stop(true);
		}

		/**
		 *!!! turn left 180 degrees while in charge!!
		 * ok calibrated according to surface.
		 */
		public void turnLeftSecond() {
			long time=System.currentTimeMillis(); //start time 
			motorLeft.stop(true);
			motorRight.setSpeed(600);
			motorRight.forward();
			long time2=System.currentTimeMillis();
			while( ( time2-time)<=ms2turn-200) {
				time2=System.currentTimeMillis();
			}
			motorRight.stop(true);
		}

		/**
		 *!!! turn left 180 degrees while in charge!!
		 * ok calibrated according to surface.
		 */
		public void turnLeft100() {
			long time=System.currentTimeMillis(); //start time 
			motorLeft.stop(true);
			motorRight.setSpeed(3000);
			motorRight.forward();
			long time2=System.currentTimeMillis();
			while( ( time2-time)<=ms2turn-350) {
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
			motorRight.setSpeed(600);
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
			motorRight.setSpeed(600);
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
			Thread.sleep(420);
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

		/**
		 * 
		 * @return touch sensor value: 0/1-> empty/onTouch
		 */
		public float[] getLastTouch() {
			return lastTouch;
		}
		/**
		 * wait for press to begin
		 */
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
		/**
		 *  turn left for several time to avoid the second object , then turn right to go straight
		 * @throws InterruptedException
		 */
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
	 * necessary time to return to the initial position 
	 */
	static long pos;

	/**
	 * backward time to leave object alone
	 */
	static long recul=355;

	/**
	 * strategies 
	 * @throws InterruptedException
	 */
	public static void main(String[] args) throws InterruptedException {
		/**
		 * to skip first return home function()
		 */
		int setHome=0;
		EV3Helper ev3=new IRobot().new EV3Helper();

		float dis; //result of ultrasonic sensor 
		boolean obj=false;
		float DIS_MAX = 1.0f;
		
		ev3.waitForPress();
		try {
			while(true) {//to find bricks : just turn left forever.
				ev3.stop();
				ev3.updateSensors();
				dis=ev3.getLastRange()[0];
				if(ev3.caught<=3)
					DIS_MAX= 0.9f;//pp
				else
					DIS_MAX=1.4f;
				//search mode 
				if(dis>=DIS_MAX||dis<=0.3) { //not in range -> turn left to search brick
					ev3.turnLeft(); 
					obj=true;
				}
				ev3.updateSensors();
				dis=ev3.getLastRange()[0];
				//catch mode
				if(dis<=DIS_MAX&&dis>0.3) {//consider as a brick 
					ev3.stop(); //don't turn left anymore 	
					ev3.forward();//go and catch it , initial a start time 
					ev3.openArm();
					long time=System.currentTimeMillis();//control if >3500ms still not catching anything
					while(ev3.getLastTouch()[0]!=1) {
						ev3.updateSensors();
						ev3.getColorName();
						ev3.updateSensors();	
						dis=ev3.getLastRange()[0];
						if(dis>=0.3&&obj) {//regulation to left
							ev3.stop();
							obj=false;
							ev3.turnLeft();
							int factor=440;//o
							if(dis>=0.35&&dis<0.45)
								factor=400;
							else if(dis>=0.45&&dis<0.5)
								factor=365;
							else if(dis<0.55&&dis>=0.5)
								factor=340;
							else if(dis<0.6&&dis>=0.55)
								factor=310;
							else if(dis>=0.6)
								factor=280;
							System.out.println("dis: "+ dis +" , factor:\n"+factor);
							Thread.sleep(factor);
							ev3.stop();
							ev3.forward();

						}
						ev3.updateSensors();	
						dis=ev3.getLastRange()[0];
						if(dis<=0.2) {//wall or a robot dectected
							ev3.stop();
							//Sound.beepSequenceUp();
							Thread.sleep(1000);
							ev3.updateSensors();	
							dis=ev3.getLastRange()[0];
							if(dis>0.25)
								break;
							//Sound.beepSequenceUp();
							ev3.turnLeft100();
							degree++;//help to find way home
							ev3.updateSensors();
							break;
						}
						long timeControl=System.currentTimeMillis();
						if(timeControl-time>=2800)
							break;
					}

					obj=true;
				}

				ev3.updateSensors();
				//touch ==1-> 2 cases: 1) go to destination(the first time catch)
				// 2)return to destination(the second time or later)
				if(ev3.getLastTouch()[0]==1) {
					ev3.caught++;
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
						//Sound.beepSequenceUp();
						ev3.backward();
						Delay.msDelay(recul);
						ev3.stop();
						ev3.closeArm();
						//turn back
						ev3.turnLeft180Empty(ms2turnEmpty+250);
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

		if(e.caught==3)
			e.turnLeftSecond();
		else
			e.turnLeft180();//calibrated
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
			if("green".equals(e.getColorName())&&System.currentTimeMillis()-time>=200) 
				e.turnLeft180Empty(ms2turnEmpty);
		}
		e.stop();		
		e.openArm();
		//Sound.beepSequenceUp();
		e.backward();
		if(e.caught>=3)
			recul=1111;
		if(e.caught>=5)
			recul=1800;
		Delay.msDelay(recul);
		e.stop();
		e.closeArm();
		if(e.caught==1) {
			e.turnLeft180Empty(ms2turnEmpty-330);
			Sound.beepSequenceUp();
		}
		else if(e.caught==2)
			e.turnLeft180Empty(ms2turnEmpty+550);

		else
			e.turnLeft180Empty(ms2turnEmpty+100);

	}
	

}
