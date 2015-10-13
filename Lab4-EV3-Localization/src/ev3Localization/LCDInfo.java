package ev3Localization;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.robotics.SampleProvider;
import lejos.utility.Timer;
import lejos.utility.TimerListener;

public class LCDInfo implements TimerListener{
	public static final int LCD_REFRESH = 100;
	private Odometer odo;
	private Timer lcdTimer;
	private TextLCD LCD = LocalEV3.get().getTextLCD();;
	private SampleProvider usSensor;
	private float[] usData;
	SampleProvider colorSensor;
	float[] colorData;
	
	// arrays for displaying data
	private double [] pos;
	
	public LCDInfo(Odometer odo, SampleProvider usSensor, float[] usData, SampleProvider colorSensor, float[] colorData) {
		this.odo = odo;
		this.lcdTimer = new Timer(LCD_REFRESH, this);
		this.usSensor = usSensor;
		this.usData = usData;
		this.colorSensor = colorSensor;
		this.colorData = colorData;
		
		// initialise the arrays for displaying data
		pos = new double [3];
		
		// start the timer
		lcdTimer.start();
	}
	
	public void timedOut() { 
		odo.getPosition(pos);
		LCD.clear();
		LCD.drawString("X: ", 0, 0);
		LCD.drawString("Y: ", 0, 1);
		LCD.drawString("H: ", 0, 2);
		LCD.drawInt((int)(pos[0] * 10), 3, 0);
		LCD.drawInt((int)(pos[1] * 10), 3, 1);
		LCD.drawInt((int)pos[2], 3, 2);
		usSensor.fetchSample(usData, 0);
		LCD.drawString(String.valueOf(usData[0]*100), 0, 4);
		colorSensor.fetchSample(colorData, 0);
		LCD.drawString(String.valueOf(colorData[0]*100), 0, 5);
	}
}
