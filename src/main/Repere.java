package main;

public class Repere {
	float x;
	float y;
	float dis;
	public float getDis() {
		return dis;
	}
	public void setDis(float dis) {
		this.dis = dis;
	}

	/**
	 * direction of head
	 * to do
	 */
	float dir;
	
	public float getX() {
		return x;
	}
	public void setX(float x) {
		this.x = x;
	}
	public float getY() {
		return y;
	}
	public void setY(float y) {
		this.y = y;
	}
	public float getDir() {
		return dir;
	}
	public void setDir(float dir) {
		this.dir = dir;
	}

	public Repere() {
		x=0f;
		y=0f;
		dir=0f;
		dis=0f;
	}
	
}
