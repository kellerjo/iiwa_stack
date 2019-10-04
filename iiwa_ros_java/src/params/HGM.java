package params;

public class HGM {

	
	public static final double jointLimitLockBuffer_deg = 10.0;
	public static final double jointLimitReleaseBuffer_deg = 10.2;
	public static final long enterAxisLimitLockWait_ms = 20;
	
	public static final double minimumAngleForRotation_deg = 0.6;
	public static final double zAxisRefreshDistance_mm = 10.0;
	
	public static final double robotObjectDistanceLock_mm = 50;
	public static final double robotObjectDistanceRelease_mm = 55;
	
	public static final double minimalTargetDistanceForRotation_mm = 30;
	
	// TODO
	public static final double jointTorqueReleaseThreshhold = 0.5;
	
	public static double[] axisLimits = new double[7];
}
