package utils;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import com.kuka.roboticsAPI.geometricModel.Frame;

public class RotationHandler {

	private Vector3D targetPosition;
	private Vector3D toolPosition;
	private Vector3D eeToTargetLine;
	private Rotation toolRotation;
	private Vector3D toolLine;
	private Rotation toolToTargetRotation;
	private Rotation finalRotation;
	double[] angles_ZYX;
	
	public RotationHandler(){
	}
	
	public void calculateRotation(Frame toolFrame, Vector3D targetVec){
		
		toolPosition = new Vector3D(toolFrame.getX(), toolFrame.getY(), toolFrame.getZ());
		targetPosition = targetVec;
		
    	eeToTargetLine = (targetPosition.subtract(toolPosition)).normalize();
    	toolRotation = new Rotation(RotationOrder.ZYX, RotationConvention.VECTOR_OPERATOR, toolFrame.getAlphaRad(), toolFrame.getBetaRad(),toolFrame.getGammaRad());
    	toolLine = toolRotation.applyTo(new Vector3D(0,0,1)).normalize();
    	toolToTargetRotation = new Rotation(toolLine, eeToTargetLine);
    	finalRotation = toolToTargetRotation.compose(toolRotation, RotationConvention.VECTOR_OPERATOR);
    	angles_ZYX = finalRotation.getAngles(RotationOrder.ZYX, RotationConvention.VECTOR_OPERATOR);
	}
	
	public double calculateAngleBetween(Frame frame1, Frame frame2){
		

    	Rotation rot1 = new Rotation(RotationOrder.ZYX, RotationConvention.VECTOR_OPERATOR, frame1.getAlphaRad(), frame1.getBetaRad(),frame1.getGammaRad());
    	Rotation rot2 = new Rotation(RotationOrder.ZYX, RotationConvention.VECTOR_OPERATOR, frame2.getAlphaRad(), frame2.getBetaRad(),frame2.getGammaRad());
    	return Rotation.distance(rot1, rot2);

	}
	
	
	public double getAngle_deg(){
    	return Math.toDegrees(Vector3D.angle(toolLine, eeToTargetLine));
	}
	
	public double[] getZYXRotation_rad(){
		return angles_ZYX;
	}
	
	public Frame applyRotation(Frame frame){
		return frame.setAlphaRad(angles_ZYX[0]).setBetaRad(angles_ZYX[1]).setGammaRad(angles_ZYX[2]);
	}
}
