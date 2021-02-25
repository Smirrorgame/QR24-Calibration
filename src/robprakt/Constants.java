package robprakt;

public class Constants {
	
	/**
	 * width of window
	 */
	final static public int mainFrameWidth = 800;
	
	/**
	 * height of window
	 */
	final static public int mainFrameHeight = 600;
		
	public static final double[] convertPoseDataToDoubleArray(String data, int index) {
		String[] dataStringArray = data.split(" ");
		double[] dataDoubleArray = new double[12];
		try
	    {
			for(int counter = index; counter < dataDoubleArray.length + index; counter++) {
				dataDoubleArray[counter-index] = Double.parseDouble(dataStringArray[counter]);
			}
	    }
	    catch (NumberFormatException nfe)
	    {
	      System.out.println("[Constants] An error occured while converting an String array to double array.");
	    }
		return dataDoubleArray;
	}
}
