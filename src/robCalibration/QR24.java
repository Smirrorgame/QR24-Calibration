package robCalibration;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.QRDecomposition;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.linear.SingularValueDecomposition;


/**
 * QR24 contains methods for QR24-calibration-algorithm described in the paper "Non-orthogonal tool/flange and robot/world calibration".
 * Using the algorithm it is possible to calibrate a six degrees-of-freedom robot manipulator by means of an suitable tracking system.
 * To calculate the two wanted matrices where the first matrix transforms from the end-effector to the marker of the tracking system and
 * the second matrix transforms from the base of the robot to the tracking system's sensor, it is necessary to take at least two
 * measurements while each measurement contains the robot pose-matrix  and transformation matrix for transform from tracker sensor to marker.
 * 
 * ===
 * CONVENTIONS for this class:
 * continuously using 4x4 homogeneous matrices
 * unit of length: millimeter
 * ===
 * 
 */

public class QR24 {

	/**
	 * List of specified robot pose matrices.
	 * Index i refers to the i-1 measurement.
	 */
	public ArrayList<RealMatrix> poseMatrices = new ArrayList<RealMatrix>();
	
	/**
	 * List of measured pose-matrices of the marker.
	 * Index i refers to the i-1 measurement.
	 */
	public ArrayList<RealMatrix> markerPoseMatrices = new ArrayList<RealMatrix>();
	
	/**
	 * The Constructor
	 * @param c the controller for sending Messages to Robots and Tracking System 
	 */
	public QR24 (){
		
	}
	
	/**
	 * This Method takes the measured matrices set M and N and creates a linear equation system to solve
	 * for matrices X and Y
	 * @param M consisting of Mi
	 * @param N consisting if Ni
	 * @return An array containing the matrix X and Y, leading with X 
	 * @throws Exception Error when there're no measurements
	 */
	public RealMatrix[] calibrate() throws Exception {
		
		int measurements = markerPoseMatrices.size();
		
		// if there's not data measured throw an error
		if (poseMatrices.size()<=0 || markerPoseMatrices.size()<=0) {
			throw new Exception("No measurements taken.");
		}
		
		System.out.println("[Calibrate] poseMatrices:"+poseMatrices.size()+", Measured: "+markerPoseMatrices.size());
		
		// create A and B matrix/vector related to the number of measurements
		RealMatrix A = new Array2DRowRealMatrix(12*measurements, 24);
		RealVector B = new ArrayRealVector(12*measurements);
				
		// Here are the coefficientmatrix A and the solution vector B
		// generated from all given Measurements Mi and Ni
		for(int cnt=0;cnt<poseMatrices.size();cnt++) {
			A.setSubMatrix(createAEntry(poseMatrices.get(cnt), markerPoseMatrices.get(cnt)).getData(), cnt*12, 0);
			B.setSubVector(cnt*12, new ArrayRealVector(createBEntry(poseMatrices.get(cnt))));
		}
		
		// create solver for the linear equation system and solve it 
		DecompositionSolver solver = new QRDecomposition(A).getSolver();
		RealVector w = solver.solve(B);
		
		// generate Matrix X and Y from the solved vector
		RealMatrix Y = getFromW(w.getSubVector(12, 12));
		RealMatrix X = getFromW(w.getSubVector(0, 12));
		
		// normalize rotational part
		RealVector nx = X.getColumnVector(0);
		RealVector ox = X.getColumnVector(1);
		RealVector px = X.getColumnVector(2);
				
		nx.mapDivideToSelf(nx.getNorm());
		ox.mapDivideToSelf(ox.getNorm());
		px.mapDivideToSelf(px.getNorm());
				
		X.setColumnVector(0, nx);
		X.setColumnVector(1, ox);
		X.setColumnVector(2, px);
				
		RealVector ny = Y.getColumnVector(0);
		RealVector oy = Y.getColumnVector(1);
		RealVector py = Y.getColumnVector(2);
				
		ny.mapDivideToSelf(ny.getNorm());
		oy.mapDivideToSelf(oy.getNorm());
		py.mapDivideToSelf(py.getNorm());
				
		Y.setColumnVector(0, ny);
		Y.setColumnVector(1, oy);
		Y.setColumnVector(2, py);
		
		
		// return the calculated X and Y matrices
		return new RealMatrix[] {X,Y};
	}
	
	public void genMatrices(List<String> effector, List<String> marker) {
		markerPoseMatrices.clear();
		poseMatrices.clear();
		int minMeasurements = Math.min(marker.size(), effector.size());
		for (int i=0;i<minMeasurements;i++) {
			String markerString = marker.get(i);
			String[] markerVal = markerString.split(" ");
			String effectorString = effector.get(i);
			String[] effectorVal = effectorString.split(" ");
			if(markerVal[1].equals("n")) {
				System.out.println("Messung "+i+" Übersprungen");
				continue;
			}
			double[][] tracker = new double[4][4];
			double[][] robot = new double[4][4];
			try
		    {
				for(int row=0;row<3;row++) {
					for(int col=0;col<4;col++) {
						tracker[row][col] = Double.parseDouble(markerVal[row*4+col+2]);;
						robot[row][col] = Double.parseDouble(effectorVal[row*4+col]);;
					}
				}
				tracker[3][0] = 0;
				tracker[3][1] = 0;
				tracker[3][2] = 0;
				tracker[3][3] = 1;
				robot[3][0] = 0;
				robot[3][1] = 0;
				robot[3][2] = 0;
				robot[3][3] = 1;
		    }
		    catch (NumberFormatException nfe)
		    {
		    	System.out.println(nfe);
		    }
			markerPoseMatrices.add(new Array2DRowRealMatrix(tracker));
			poseMatrices.add(new Array2DRowRealMatrix(robot));
		}
	}
	
	/**
	 * Berechnet eine Transformationsmatrix anhand eines gegebenen Vektors 
	 * @param w Vektor mit Einträgen der Matrix
	 * @return die berechnete Matrix
	 */
	public RealMatrix getFromW(RealVector w) {
		RealMatrix M = new Array2DRowRealMatrix(4,4);
		
		M.setEntry(0, 0, w.getEntry(0));
		M.setEntry(1, 0, w.getEntry(1));
		M.setEntry(2, 0, w.getEntry(2));
		M.setEntry(0, 1, w.getEntry(3));
		M.setEntry(1, 1, w.getEntry(4));
		M.setEntry(2, 1, w.getEntry(5));
		M.setEntry(0, 2, w.getEntry(6));
		M.setEntry(1, 2, w.getEntry(7));
		M.setEntry(2, 2, w.getEntry(8));
		M.setEntry(0, 3, w.getEntry(9));
		M.setEntry(1, 3, w.getEntry(10));
		M.setEntry(2, 3, w.getEntry(11));
		M.setEntry(3, 3, 1);
		
		return M;
	}
		
	/**
	 * create A matrix for a single pair of measuring data M and N
	 * @param m pose matrix of robot
	 * @param n measured data by tracking system
	 * @return Ai matrix for a single pair of measuring data M and N
	 */
	private RealMatrix createAEntry(RealMatrix m,RealMatrix n) {
		
		RealMatrix rotM = getRot(m);
		
		RealMatrix N = n.copy();
		RealMatrix Ai = new Array2DRowRealMatrix(12,24); //12x24 matrix for one pair of measured matrices (M,N)
		RealMatrix Z = new Array2DRowRealMatrix(3,3); //3x3 zero-matrix
		RealMatrix Identity12 = MatrixUtils.createRealIdentityMatrix(12); //12x12 identity-matrix
		
		// first column
		Ai.setSubMatrix(rotM.scalarMultiply(N.getEntry(0, 0)).getData(), 0, 0);
		Ai.setSubMatrix(rotM.scalarMultiply(N.getEntry(1, 0)).getData(), 3, 0);
		Ai.setSubMatrix(rotM.scalarMultiply(N.getEntry(2, 0)).getData(), 6, 0);
		
		//TODO: in den letzten Zeilen die Werte mit t bestehend aus gewissen Nij multiplizieren
		double t1 = -(N.getEntry(0, 0)*N.getEntry(0, 3)+
				N.getEntry(1,0)*N.getEntry(1,3)+
				N.getEntry(2, 0)*N.getEntry(2, 3));
		
		Ai.setSubMatrix(rotM.scalarMultiply(t1).getData(), 9, 0);
		                                                   
		// second column
		Ai.setSubMatrix(rotM.scalarMultiply(N.getEntry(0, 1)).getData(), 0, 3);
		Ai.setSubMatrix(rotM.scalarMultiply(N.getEntry(1, 1)).getData(), 3, 3);
		Ai.setSubMatrix(rotM.scalarMultiply(N.getEntry(2, 1)).getData(), 6, 3);
		
		double t2 = -(N.getEntry(0, 1)*N.getEntry(0,3)+
				N.getEntry(1,1)*N.getEntry(1,3)+
				N.getEntry(2,1)*N.getEntry(2, 3));
		Ai.setSubMatrix(rotM.scalarMultiply(t2).getData(), 9, 3);

		// third column
		Ai.setSubMatrix(rotM.scalarMultiply(N.getEntry(0, 2)).getData(), 0, 6);
		Ai.setSubMatrix(rotM.scalarMultiply(N.getEntry(1, 2)).getData(), 3, 6);
		Ai.setSubMatrix(rotM.scalarMultiply(N.getEntry(2, 2)).getData(), 6, 6);
		
		double t3 = -(N.getEntry(0, 2)*N.getEntry(0, 3)+
				N.getEntry(1,2)*N.getEntry(1,3)+
				N.getEntry(2,2)*N.getEntry(2,3));
		Ai.setSubMatrix(rotM.scalarMultiply(t3).getData(), 9, 6);
		
		// fourth column
		Ai.setSubMatrix(Z.getData(), 0, 9);
		Ai.setSubMatrix(Z.getData(), 3, 9);
		Ai.setSubMatrix(Z.getData(), 6, 9);
		Ai.setSubMatrix(rotM.getData(), 9, 9);
		
		// fifth column
		Ai.setSubMatrix(Identity12.scalarMultiply(-1d).getData(), 0, 12);
		return Ai;
	}
	
	/**
	 * create B vector as an array of double values
	 * @param m RealMatrix contains matrix that we need to get the translational part of the matrix
	 * @return array of double values
	 */
	private double[] createBEntry(RealMatrix m) {
		double[] bi = {0,0,0,0,0,0,0,0,0,-m.getEntry(0, 3),-m.getEntry(1, 3),-m.getEntry(2, 3)};
		return bi;
	}
	
	/**
	 * Returns rotational part of a matrix
	 * @param mat matrix has to have at least 3 rows and 3 columns
	 * @return RealMatrix containing rotational part of mat
	 */
	private RealMatrix getRot(RealMatrix mat) {
		return mat.copy().getSubMatrix(
				new int[] {0,1,2}, new int[] {0,1,2});
	}

	/**
	 * Print the given matrix to the console
	 * @param rm the matrix to print
	 */
	public void printTable(RealMatrix rm) {
		int colDimNumber = rm.getColumnDimension();
		int rowDimNumber = rm.getRowDimension();
		ArrayList<String> headers = new ArrayList<String>();
		String[] alphabet = {"a","b","c","d","e","f","g","h","i","j","k","l","m","n","o","p","q","r","s","t","u","v","w","x","y","z"};
		for(int colDim = 0; colDim < colDimNumber; colDim++) {
			headers.add(alphabet[colDim]);
		}
		ArrayList<ArrayList<String>> content = new ArrayList<ArrayList<String>>();
		for(int rowDim = 0; rowDim < rowDimNumber; rowDim++) {
			double[] rowValues = rm.getRow(rowDim);
			ArrayList<String> rowArrayList = new ArrayList<String>();
			for(int colDim = 0; colDim < colDimNumber; colDim++) {
				rowArrayList.add(rowValues[colDim]+ "");
			}
			content.add(rowArrayList);
		}
		ConsoleTable ct = new ConsoleTable(headers,content);
		ct.printTable();
	}
		
}
