/*
 * Copyright (c) 2011-2013, Peter Abeles. All Rights Reserved.
 *
 * This file is part of BoofCV (http://boofcv.org).
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package robotwebcam;

import boofcv.alg.sfm.overhead.CameraPlaneProjection;
import boofcv.gui.image.ShowImages;
import boofcv.io.image.UtilImageIO;
import boofcv.misc.BoofMiscOps;
import boofcv.struct.calib.MonoPlaneParameters;
import georegression.struct.point.Point2D_F64;

import javax.swing.*;
import java.awt.*;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.image.BufferedImage;

/**
 * Displays an image and loads MonoPlaneParameters.  When you click on the image it tells you the 2D coordinate of the point on the plane
 * and the distance that point is from the previous point you clicked on.  This can be used to verify the correctness of your calibration.
 *
 * @author Peter Abeles
 */
public class CheckImageToPlaneMath extends JPanel implements MouseListener
{
	BufferedImage image;
	CameraPlaneProjection planeProjection;

	int numClicks = 0;
	Point2D_F64 pointOnPlaneA = new Point2D_F64();
	Point2D_F64 pointOnPlaneB = new Point2D_F64();

	public CheckImageToPlaneMath(BufferedImage image,
                                MonoPlaneParameters param) {
		this.image = image;
		planeProjection = new CameraPlaneProjection();
		planeProjection.setConfiguration(param.planeToCamera,param.intrinsic);

		setPreferredSize(new Dimension(image.getWidth(),image.getHeight()));
		addMouseListener(this);
		requestFocus();
	}

	@Override
	public void paintComponent(Graphics g) {
		super.paintComponent(g);
		Graphics2D g2 = (Graphics2D)g;

		g2.drawImage(image,0,0,null);
	}

	@Override
	public void mouseClicked(MouseEvent e) {

		Point2D_F64 point = numClicks++ %2 == 0 ? pointOnPlaneA : pointOnPlaneB;

		planeProjection.pixelToPlane(e.getX(),e.getY(),point);

		System.out.println("Location on grid = "+point);
		System.out.println("Distance from last click "+pointOnPlaneA.distance(pointOnPlaneB));
	}

	@Override
	public void mousePressed(MouseEvent e) {}

	@Override
	public void mouseReleased(MouseEvent e) {}

	@Override
	public void mouseEntered(MouseEvent e) {}

	@Override
	public void mouseExited(MouseEvent e) {}

	public static void main( String args[] ) {
		BufferedImage input = UtilImageIO.loadImage("data/calibration_floor.jpg");

		MonoPlaneParameters param = BoofMiscOps.loadXML("camera_plane_param.xml");

      CheckImageToPlaneMath app = new CheckImageToPlaneMath(input,param);
		ShowImages.showWindow(app, "Click On Image");
	}

}
