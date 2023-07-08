package teamtators.util;

import com.ctre.phoenix6.hardware.CANcoder;

import frc.constants.SwerveConstants;

public class CANCoderWrapper extends CANcoder {

    private double conversionFactor;

    public CANCoderWrapper(int id) {
        super(id);
    }
    public CANCoderWrapper(int id, String canbus) {
        super(id,canbus);
    }


    public void setConversionFactor(double conversionFactor) {
        this.conversionFactor = conversionFactor;
    }

    public double getConvertedAbsolute() {
        return (conversionFactor * (super.getAbsolutePosition().getValue()));
    }

    public void setInverted(boolean inversion) {
        if (inversion) {
            conversionFactor = Math.abs(conversionFactor) * -1;
        }
    }

    public static double degreesToModuleRotations(double degrees){
        return (degrees/360* SwerveConstants.SwerveModule.MOTOR_ROTATIONS_PER_MODULE_ROTATION);
    }

}
