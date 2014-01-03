package robotwebcam;

import boofcv.abst.calib.ConfigChessboard;
import boofcv.abst.calib.PlanarCalibrationDetector;
import boofcv.alg.distort.LensDistortionOps;
import boofcv.alg.geo.PerspectiveOps;
import boofcv.alg.geo.calibration.PlanarCalibrationTarget;
import boofcv.alg.geo.calibration.Zhang99ComputeTargetHomography;
import boofcv.alg.geo.calibration.Zhang99DecomposeHomography;
import boofcv.core.image.ConvertBufferedImage;
import boofcv.factory.calib.FactoryPlanarCalibrationTarget;
import boofcv.io.image.UtilImageIO;
import boofcv.misc.BoofMiscOps;
import boofcv.struct.calib.IntrinsicParameters;
import boofcv.struct.calib.MonoPlaneParameters;
import boofcv.struct.distort.PointTransform_F64;
import boofcv.struct.image.ImageFloat32;
import georegression.geometry.GeometryMath_F64;
import georegression.geometry.RotationMatrixGenerator;
import georegression.metric.Distance3D_F64;
import georegression.metric.Intersection3D_F64;
import georegression.struct.line.LineParametric3D_F64;
import georegression.struct.plane.PlaneGeneral3D_F64;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.transform.se.SePointOps_F64;
import org.ejml.data.DenseMatrix64F;

import java.awt.image.BufferedImage;

/**
 * Compares the extrinsic parameters (plane to camera transform) needed by visual odometry using an image of a calibration
 * grid laying flat on the ground.   It is worth noting that the smaller the acute angle is between the camera and the calibration
 * grid the worst its accuracy will be.  If the camera is pointed straight forward you will likely have a bad estimate and will need
 * some other approach for estimating the camera's pose.
 *
 * @author Peter Abeles
 */
public class EstimatePlaneParameters
{
   public static int gridCols = 5;
   public static int gridRows = 7;

   public static double sizeOfSquareInMeters = 0.03;

   /**
    * Detect the calibration grid and compute its pose.
    */
   public static Se3_F64 findCalibrationTargetPose( IntrinsicParameters intrinsic , BufferedImage image ) {
      // Detects the target and calibration point inside the target
      PlanarCalibrationDetector detector = FactoryPlanarCalibrationTarget.detectorChessboard(new ConfigChessboard(gridCols, gridRows));
      // Specifies the location of calibration points in the target's coordinate system.  Note that z=0

      PlanarCalibrationTarget target = FactoryPlanarCalibrationTarget.gridChess(gridCols, gridRows, sizeOfSquareInMeters);
      // Computes the homography
      Zhang99ComputeTargetHomography computeH = new Zhang99ComputeTargetHomography(target);
      // decomposes the homography
      Zhang99DecomposeHomography decomposeH = new Zhang99DecomposeHomography();

      // Need to remove lens distortion for accurate pose estimation
      PointTransform_F64 distortToUndistorted = LensDistortionOps.transformRadialToPixel_F64(intrinsic);

      // convert the intrinsic into matrix format
      DenseMatrix64F K = PerspectiveOps.calibrationMatrix(intrinsic, null);

      ImageFloat32 gray = ConvertBufferedImage.convertFrom(image, (ImageFloat32) null);

      // detect calibration points
      if( !detector.process(gray) )
         throw new RuntimeException("Failed to detect target");

      // Remove lens distortion from detected calibration points
      java.util.List<Point2D_F64> points = detector.getPoints();
      for( Point2D_F64 p : points ) {
         distortToUndistorted.compute(p.x,p.y,p);
      }

      // Compute the homography
      if( !computeH.computeHomography(points) )
         throw new RuntimeException("Can't compute homography");

      DenseMatrix64F H = computeH.getHomography();

      // compute camera pose from the homography matrix
      decomposeH.setCalibrationMatrix(K);
      Se3_F64 targetToCamera = decomposeH.decompose(H);

      return targetToCamera;
   }

   /**
    * The calibration target's pose can be partially unspecified due to symmetry in the target.  This uses trig to compute the
    * pitch and roll of the camera relative to the ground plane and constructs a reasonable transform.  The closest point
    * on the ground to the camera is the original.  +x is along the projection of the optical axis to the ground and +y is to the left.
    */
   public static Se3_F64 computeExtrinsic( Se3_F64 targetToCamera ) {
      // Find the optical axis in target frame
      LineParametric3D_F64 opticalAxis = new LineParametric3D_F64();
      opticalAxis.slope.set(0,0,1);
      SePointOps_F64.transformReverse(targetToCamera,opticalAxis.p,opticalAxis.p);
      GeometryMath_F64.multTran(targetToCamera.R,opticalAxis.slope,opticalAxis.slope);

      // find the distance that the camera is from the ground plane
      PlaneGeneral3D_F64 plane = new PlaneGeneral3D_F64(0,0,1,0);

      double d = Math.abs(Distance3D_F64.distance(plane, opticalAxis.p));

      // find the distance to the plane along the optical axis
      Point3D_F64 tmp0 = new Point3D_F64();

      // assume the camera is straight forward or pointed towards the plane
      double pitch;
      if( Intersection3D_F64.intersect(plane, opticalAxis, tmp0) ) {
         double distAlongAxis = tmp0.distance(opticalAxis.p);
         pitch = Math.PI/2-Math.acos(d / distAlongAxis);
      } else {
         pitch = 0;
      }

      // Time to find roll.
      // Only roll will change the distance these two points are from the plane
      double r = d/3;
      tmp0.set(-r, 0, 0);
      SePointOps_F64.transformReverse(targetToCamera, tmp0, tmp0);
      double d0 = Math.abs(Distance3D_F64.distance(plane,tmp0));
      double diff = d0-d;

      // use the fact that it is a right triangle and that triangles inner angles sum to 180
      double roll = Math.PI - (Math.acos(Math.abs(diff)/r) + Math.PI/2);
      if( diff < 0 )
         roll = -roll;

      // now construct the transform
      // remember that the ground plane's normal is +y, which is down through the floor
      Se3_F64 planeToCamera = new Se3_F64();
      planeToCamera.R = RotationMatrixGenerator.eulerXYZ(pitch,0,-roll,null);
      planeToCamera.getT().set(0,d,0);

      return planeToCamera;
   }

   public static void main( String args[] ) {

      // Load camera calibration
      IntrinsicParameters intrinsic =
            BoofMiscOps.loadXML("data/intrinsic.xml");

      BufferedImage image = UtilImageIO.loadImage("data/calibration_floor.jpg");

      Se3_F64 targetToCamera = findCalibrationTargetPose(intrinsic,image);

      MonoPlaneParameters param = new MonoPlaneParameters();
      param.intrinsic = intrinsic;
      param.planeToCamera = computeExtrinsic(targetToCamera);

      // save the results
      BoofMiscOps.saveXML(param,"camera_plane_param.xml");

   }
}
