package main;

import lejos.hardware.Sound;

public class Robot{
	 class son {
			public void beep() {
				Sound.beep();
				Sound.twoBeeps();
			}
		}
	public static void main(String[] args) {
		son s=new Robot().new son();
		s.beep();
	}
}