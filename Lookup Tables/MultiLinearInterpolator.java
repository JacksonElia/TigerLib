// TigerLib 2024

import java.util.Arrays;

public final class MultiLinearInterpolator {

  private final double[][] lookupTable;
  
  /**
   * Handles finding values through a lookup table in a linear fashion.
   * @param lookupTable an array containing {x, y1, y2, ...yn} points, the x values must be in ascending order.
   */
  public MultiLinearInterpolator(double[][] lookupTable) {
    this.lookupTable = lookupTable;
  }

  /**
   * Returns multiple linearly-interpolated values from the lookup table corresponding to the given input value.
   * @return A double array containing each linearly interpolated value. It will be as long as the lookup
   * table rows minus 1.
   */
  public double[] getLookupValue(double inputXValue) {  
    // Check if inputXValue is less than the table's first value, if it is, return the lowest y values
    if (inputXValue < lookupTable[0][0]) {
      return Arrays.copyOfRange(lookupTable[0], 1, lookupTable[0].length);  
    } // Check if inputXValue is greater than the table's last value, if it is, return the greatest y values
    else if (inputXValue > lookupTable[lookupTable.length - 1][0]) {
      return Arrays.copyOfRange(lookupTable[lookupTable.length - 1], 1, lookupTable[0].length);
    }
  
    for(int i = 0; i < lookupTable.length; i++) {
      if (inputXValue == lookupTable[i][0]) {
        return Arrays.copyOfRange(lookupTable[i], 1, lookupTable[0].length);
      } else if (inputXValue > lookupTable[i][0] && inputXValue < lookupTable[i + 1][0]) {
        double[] interpolatedValues = new double[lookupTable[0].length - 1];
        for(int j = 1; j < lookupTable[0].length; j++) {
          double slope = (lookupTable[i + 1][j] - lookupTable[i][j]) / (lookupTable[i + 1][0] - lookupTable[i][0]);
          double yIntercept = lookupTable[i][j];
          interpolatedValues[j - 1] = slope * (inputXValue - lookupTable[i][0]) + yIntercept;
        }
        return interpolatedValues;
      }
    }
      
    // This should never be reached, but returns the first value to be safe
    return Arrays.copyOfRange(lookupTable[0], 1, lookupTable[0].length);   
  }
}
