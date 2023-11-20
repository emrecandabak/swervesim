package frc.lib.math;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Rotation3d;

public class Rotation3dUtils {

    public static Double[] rotation3dToDoubleArray(Rotation3d rot){
        ArrayList<Double> arrayList = new ArrayList<>();
        
        for(String elem : rot.toString().replace("Rotation3d(Quaternion(", "").replace("))", "").replace(" ", "").split(",")){
            arrayList.add(Double.parseDouble(elem));
        }
        Double[] out = new Double[arrayList.size()];
        out = arrayList.toArray(out);
        return out;
    }

    public static Double[] concatenateArrays(Double[] array1, Double[] array2) {
        int length1 = array1.length;
        int length2 = array2.length;
        Double[] result = new Double[length1 + length2];
    
        System.arraycopy(array1, 0, result, 0, length1);
        System.arraycopy(array2, 0, result, length1, length2);
    
        return result;
    }
    
}
