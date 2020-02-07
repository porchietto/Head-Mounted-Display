from MPU9250Driver import MPU
import time
import os

def cuentaA(channel):
    Pass
    
def main():
    # Set up class
    gyro = 500      # 250, 500, 1000, 2000 [deg/s]
    acc = 4         # 2, 4, 7, 16 [g]
    mag = 16        # 14, 16 [bit]
    tau = 0.98
    mpu = MPU(gyro, acc, mag, tau)

    # Set up the IMU and mag sensors
    mpu.setUpIMU()
    mpu.setUpMAG()
    
    # Calibrate the mag or provide values that have been verified with the visualizer
    #mpu.calibrateMagGuide()
    bias = [145, 145, -155]
    scale = [1.10, 1.05, 1.05]
    #mpu.setMagCalibration(bias, scale)

    # Calibrate the gyro with N points
    mpu.calibrateGyro(1000)
    
    c=0
    # Run until stopped
    try:
        startTime = time.time()
        while(time.time() - startTime < 10):
            c+=1# Get new values
            mpu.integartorFilter()
        print('R: {:<8.3f} P: {:<8.3f} Y: {:<8.3f}'.format(mpu.gyroRoll,mpu.gyroPitch,mpu.gyroYaw))
    except KeyboardInterrupt:
        Pass
    mpu.GPIO.cleanup(mpu.RPi_GPIO)
    '''Reseteo MPU'''
    try:
        mpu.bus.write_byte_data(mpu.MPU9250_ADDRESS, mpu.PWR_MGMT_1, 0x80)
    except:
        print('no se pundo escribir en la mpu')
    # End if user hits control c
    print("Closing")
    print(c)
# Main loop
if __name__ == '__main__':
    main()
