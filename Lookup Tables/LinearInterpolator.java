public final class LinearInterpolator {

  private final double[][] lookupTable;
  
  /**
   * Handles finding values through a lookup table in a linear fashion.
   * @param lookupTable an array containing {x, y} points, the x values must be in ascending order.
   */
  public LinearInterpolator(double[][] lookupTable) {
    this.lookupTable = lookupTable;
  }

  /**
   * Returns a linearly-interpolated value from the lookup table corresponding to the given input value.
   * @param inputXValue 
   * @return
   */
  public double getLookupValue(double inputXValue) {    
    // Check if inputXValue is less than the table's first value, if it is, return the lowest y value
    if (inputXValue < lookupTable[0][0]) {
        return lookupTable[0][1];
    } // Check if inputXValue is greater than the table's first value, if it is, return the greatest y value
    else if (inputXValue > lookupTable[lookupTable.length - 1][0]) {
        return lookupTable[lookupTable.length - 1][1];
    }
    
    for(int i = 0; i < lookupTable.length; i++) {
        if (inputXValue == lookupTable[i][0]) {
            return lookupTable[i][1];
        } else if (inputXValue > lookupTable[i][0] && inputXValue < lookupTable[i + 1][0]) {
            double slope = (lookupTable[i + 1][1] - lookupTable[i][1]) / (lookupTable[i + 1][0] - lookupTable[i][0]);
            double yIntercept = lookupTable[i][1];
            // y = mx + b
            return slope * (inputXValue - lookupTable[i][0]) + yIntercept;
        }
    }
        
    // This should never be reached, but returns the first value to be safe
    return lookupTable[0][1];
  }

}
