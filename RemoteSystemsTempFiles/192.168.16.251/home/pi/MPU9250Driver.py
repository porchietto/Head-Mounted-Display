import smbus
import math
import time
import RPi.GPIO as GPIO

class MPU:
    def __init__(self, gyro, acc, mag, tau):
        # Class / object / constructor setup
        self.ax = None; self.ay = None; self.az = None;
        self.temp = None; 
        self.gx = None; self.gy = None; self.gz = None;
        self.mx = None; self.my = None; self.mz = None;

        self.gyroXcal = 0
        self.gyroYcal = 0
        self.gyroZcal = 0

        self.magXcal = 0; self.magXbias = 0; self.magXscale = 0;
        self.magYcal = 0; self.magYbias = 0; self.magYscale = 0;
        self.magZcal = 0; self.magZbias = 0; self.magZscale = 0;

        self.gyroRoll = 0
        self.gyroPitch = 0
        self.gyroYaw = 0

        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        self.dtTimer = 0
        self.tau = tau

        self.q = [1,0,0,0]
        self.beta = 1

        self.gyroScaleFactor, self.gyroHex = self.gyroSensitivity(gyro)
        self.accScaleFactor,  self.accHex  = self.accelerometerSensitivity(acc)
        self.magScaleFactor,  self.magHex  = self.magnetometerSensitivity(mag)

        self.bus = smbus.SMBus(1)
        
        self.MPU9250_ADDRESS    = 0x68
        self.AK8963_ADDRESS     = 0x0C
        
        self.XG_OFFSET_H        = 0x13 #6 Bytes: XG_OFFSET_H, XG_OFFSET_L, YG_OFFSET_H, YG_OFFSET_L, ZG_OFFSET_H, ZG_OFFSET_L
        self.SMPLRT_DIV         = 0x19
        self.CONFIG             = 0x1A # GYRO_LPF
        self.GYRO_CONFIG        = 0x1B
        self.ACCEL_CONFIG       = 0x1C
        self.ACCEL_CONFIG2      = 0x1d
        self.FIFO_EN            = 0x23
        self.I2C_MST_CTRL       = 0x24
        self.I2C_SLV0_ADDR      = 0x25
        self.I2C_SLV0_REG       = 0x26
        self.I2C_SLV0_CTRL      = 0x27
        self.INT_PIN_CFG        = 0x37
        self.INT_ENABLE         = 0x38
        self.INT_STATUS         = 0x3A
        self.ACCEL_XOUT_H       = 0x3B #6 Bytes: ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L, ACCEL_ZOUT_H, ACCEL_ZOUT_L
        self.TEMP_OUT_H         = 0x41 #2 Bytes: TEMP_OUT_H, TEMP_OUT_L
        self.GYRO_XOUT_H        = 0x43 #6 Bytes: GYRO_XOUT_H, GYRO_XOUT_L, GYRO_YOUT_H, GYRO_YOUT_L, GYRO_ZOUT_H, GYRO_ZOUT_L
        self.EXT_SENS_DATA_00   = 0x49
        self.I2C_SLV0_DO        = 0x63
        self.I2C_MST_DELAY_CTRL = 0X67 
        self.SIGNAL_PATH_RESET  = 0x68
        self.USER_CTRL          = 0x6A
        self.PWR_MGMT_1         = 0x6B
        self.PWR_MGMT_2         = 0x6C
        self.FIFO_COUNTH        = 0x72 #2 Bytes: FIFO_COUNTH, FIFO_COUNTL
        self.FIFO_R_W           = 0x74
        self.WHO_AM_I_MPU9250   = 0x75
        
        self.WHO_AM_I_AK8963    = 0x00
        self.AK8963_XOUT_L      = 0x03 #6 Bytes: HXL, HXH, HYH, HZL, HZH
        self.AK8963_CNTL        = 0x0A
        self.AK8963_CNTL2       = 0x0B
        self.AK8963_ASAX        = 0x10 #3 Bytes: ASAX, ASAY, ASAZ
                
        self.MPU9250_SAMPLERATE_MIN = 5 #samples per second is the lowest
        self.MPU9250_SAMPLERATE_MAX = 32000 # samples per second is the absolute maximum

        self.MPU9250_COMPASSRATE_MIN = 1 #samples per second is the lowest
        self.MPU9250_COMPASSRATE_MAX = 100 #samples per second is maximum
        self.RPi_GPIO           = 7 #Pin 7 o GPIO 4
        self.GPIO               = GPIO
        self.Interrupt          = True


    def gyroSensitivity(self, x):
        # Create dictionary with standard value of 500 deg/s
        return {
            250:  [131.0, 0x00],
            500:  [65.5,  0x08],
            1000: [32.8,  0x10],
            2000: [16.4,  0x18]
        }.get(x,  [65.5,  0x08])

    def accelerometerSensitivity(self, x):
        # Create dictionary with standard value of 4 g
        return {
            2:  [16384.0, 0x00],
            4:  [8192.0,  0x08],
            8:  [4096.0,  0x10],
            16: [2048.0,  0x18]
        }.get(x,[8192.0,  0x08])

    def magnetometerSensitivity(self, x):
        # Create dictionary with standard value of 16 bit
        return {
            14:  [10.0*4912.0/8190.0,  0x06],
            16:  [10.0*4912.0/32760.0, 0x16],
        }.get(x,[10.0*4912.0/32760.0,  0x16])

    def setUpIMU(self):
        # Check to see if there is a good connection with the MPU 9250
        try:
            whoAmI = self.bus.read_byte_data(self.MPU9250_ADDRESS, self.WHO_AM_I_MPU9250)
        except:
            print('whoAmI IMU read failed')
            return

        if (whoAmI == 0x71):
            #DEVICE_RESET
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.PWR_MGMT_1, 0x80)
            time.sleep(0.1)
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.SIGNAL_PATH_RESET, 0x03)
            time.sleep(0.1)
            
            #self.bus.write_byte_data(self.MPU9250_ADDRESS, self.SMPLRT_DIV, 0x00)

            # Clock Source PLL with X axis gyroscope reference
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.PWR_MGMT_1, 0x06)
            
            # Config  
            '''EXT_SYNC_SET FSYNC bit location [5:3]
            0 function disabled
            1 TEMP_OUT_L[0]
            2 GYRO_XOUT_L[0]
            3 GYRO_YOUT_L[0]
            4 GYRO_ZOUT_L[0]
            5 ACCEL_XOUT_L[0]
            6 ACCEL_YOUT_L[0]
            7 ACCEL_ZOUT_L[0]
            '''
            EXT_SYNC_SET = 0 << 3
            '''
            DLPF_CFG    Bandwidth(Hz)   Frequency Samples
            0           250             8KHz  
            1           184             1Khz
            2           92              1Khz
            3           41              1Khz
            4           20              1Khz
            5           10              1Khz
            6           5               1Khz
            7           3600            8KHz
            '''
            DLPF = 4
            CONFIG = EXT_SYNC_SET + DLPF
            
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.CONFIG, CONFIG)
           
            '''SAMPLE_RATE=Internal_Sample_Rate / (1 + SMPLRT_DIV)'''
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.SMPLRT_DIV, 0)
            
            if self.Interrupt is True:
                self.bus.write_byte_data(self.MPU9250_ADDRESS, self.INT_PIN_CFG, 0x00)
                self.bus.write_byte_data(self.MPU9250_ADDRESS, self.INT_ENABLE, 0x01)
                GPIO.setmode(GPIO.BOARD)
                GPIO.setup(self.RPi_GPIO, GPIO.IN)

            '''Nadie escribe en la FIFO'''
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.FIFO_EN, 0x00)
            
            '''I2C configuration multi-master I2C 400KHz'''
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_MST_CTRL, 0x0D)
            
            
            
            # Configure the accelerometer
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.ACCEL_CONFIG, self.accHex)

            # Configure the gyro
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.GYRO_CONFIG, self.gyroHex)   
            
            # Display message to user
            print("MPU set up:")
            print('\tAccelerometer: ' + str(hex(self.accHex)) + ' ' + str(self.accScaleFactor))
            print('\tGyroscope: ' + str(hex(self.gyroHex)) + ' ' + str(self.gyroScaleFactor) + "\n")
        else:
            # Bad connection or something went wrong
            print("IMU WHO_AM_I was: " + hex(whoAmI) + ". Should have been " + hex(0x71))

    def setUpMAG(self):
        # Initialize connection with mag for a WHO_AM_I test
        self.bus.write_byte_data(self.MPU9250_ADDRESS, self.USER_CTRL, 0x20)                              # Enable I2C Master mode
        self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_ADDR, self.AK8963_ADDRESS | 0x80)    # Set the I2C slave address of AK8963 and set for read.
        self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_REG, self.WHO_AM_I_AK8963)           # I2C slave 0 register address from where to begin data transfer
        self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_CTRL, 0x81)                          # Enable I2C and transfer 1 byte
        time.sleep(0.05)

        # Check to see if there is a good connection with the mag
        try:
            whoAmI = self.bus.read_byte_data(self.MPU9250_ADDRESS, self.EXT_SENS_DATA_00)
        except:
            print('whoAmI MAG read failed')
            return

        if (whoAmI == 0x48):
            # Connection is good! Begin the true initialization
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_ADDR, self.AK8963_ADDRESS);     # Set the I2C slave address of AK8963 and set for write.
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_REG, self.AK8963_CNTL2);        # I2C slave 0 register address from where to begin data transfer
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_DO, 0x01);                      # Reset AK8963
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_CTRL, 0x81);                    # Enable I2C and write 1 byte
            time.sleep(0.05)

            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_ADDR, self.AK8963_ADDRESS);     # Set the I2C slave address of AK8963 and set for write.
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_REG, self.AK8963_CNTL);         # I2C slave 0 register address from where to begin data transfer
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_DO, 0x00);                      # Power down magnetometer
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_CTRL, 0x81);                    # Enable I2C and transfer 1 byte
            time.sleep(0.05)

            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_ADDR, self.AK8963_ADDRESS);     # Set the I2C slave address of AK8963 and set for write.
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_REG, self.AK8963_CNTL);         # I2C slave 0 register address from where to begin data transfer
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_DO, 0x0F);                      # Enter fuze mode
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_CTRL, 0x81);                    # Enable I2C and write 1 byte
            time.sleep(0.05)

            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_ADDR, self.AK8963_ADDRESS | 0x80);   # Set the I2C slave address of AK8963 and set for read.
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_REG, self.AK8963_ASAX);              # I2C slave 0 register address from where to begin data transfer
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_CTRL, 0x83);                         # Enable I2C and read 3 bytes
            time.sleep(0.05)

            # Read the x, y, and z axis calibration values
            try:
                rawData = self.bus.read_i2c_block_data(self.MPU9250_ADDRESS, self.EXT_SENS_DATA_00, 3)
            except:
                print('Reading MAG x y z calibration values failed')
                return

            # Convert values to something more usable
            self.magXcal =  float(rawData[0] - 128)/256.0 + 1.0;
            self.magYcal =  float(rawData[1] - 128)/256.0 + 1.0;
            self.magZcal =  float(rawData[2] - 128)/256.0 + 1.0;

            # Flush the sysem
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_ADDR, self.AK8963_ADDRESS);     # Set the I2C slave address of AK8963 and set for write.
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_REG, self.AK8963_CNTL);         # I2C slave 0 register address from where to begin data transfer
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_DO, 0x00);                      # Power down magnetometer
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_CTRL, 0x81);                    # Enable I2C and write 1 byte

            # Configure the settings for the mag
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_ADDR, self.AK8963_ADDRESS);     # Set the I2C slave address of AK8963 and set for write.
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_REG, self.AK8963_CNTL);         # I2C slave 0 register address from where to begin data transfer
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_DO, self.magHex);               # Set magnetometer for 14 or 16 bit continous 100 Hz sample rates
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_CTRL, 0x81);                    # Enable I2C and transfer 1 byte
            time.sleep(0.05)

            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_ADDR, self.AK8963_ADDRESS | 0x80);    # Set the I2C slave address of AK8963 and set for read.
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_REG, self.AK8963_CNTL);               # I2C slave 0 register address from where to begin data transfer
            self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_CTRL, 0x81);                          # Enable I2C and transfer 1 byte
            time.sleep(0.05)

            # Display results to user
            print("MAG set up:")
            print("\tMagnetometer: " + hex(self.magHex) + " " + str(round(self.magScaleFactor,3)) + "\n")
        else:
            # Bad connection or something went wrong
            print("MAG WHO_AM_I was: " + hex(whoAmI) + ". Should have been " + hex(0x48))

    def eightBit2sixteenBit(self, l, h):
        # Shift the low and high byte into a 16 bit number
        val = (h << 8) + l

        # Make 16 bit unsigned value to signed value (0 to 65535) to (-32768 to +32767)
        if (val >= 0x8000):
            return -((65535 - val) + 1)
        else:
            return val

    def readRawIMU(self): 
        # Read 14 raw values [High Low] as temperature falls between the accelerometer and gyro registries
        try:
            rawData = self.bus.read_i2c_block_data(self.MPU9250_ADDRESS, self.ACCEL_XOUT_H, 14)
        except:
            print('Read raw IMU data failed')

        # Convert the raw values to something a little more useful (middle value is temperature)
        self.ax = self.eightBit2sixteenBit(rawData[1], rawData[0])
        self.ay = self.eightBit2sixteenBit(rawData[3], rawData[2])
        self.az = self.eightBit2sixteenBit(rawData[5], rawData[4])

        self.gx = self.eightBit2sixteenBit(rawData[9], rawData[8])
        self.gy = self.eightBit2sixteenBit(rawData[11], rawData[10])
        self.gz = self.eightBit2sixteenBit(rawData[13], rawData[12])
        
    def readRawAcel(self):
        # Read 6 raw values [High Low] as accelerometer
        try:
            rawData = self.bus.read_i2c_block_data(self.MPU9250_ADDRESS, self.ACCEL_XOUT_H, 6)
        except:
            print('Read raw IMU data failed')

        # Convert the raw values to something a little more useful (middle value is temperature)
        self.ax = self.eightBit2sixteenBit(rawData[1], rawData[0])
        self.ay = self.eightBit2sixteenBit(rawData[3], rawData[2])
        self.az = self.eightBit2sixteenBit(rawData[5], rawData[4])

    def readRawTemp(self):
        # Read 2 raw values [High Low] as temperature
        try:
            rawData = self.bus.read_i2c_block_data(self.MPU9250_ADDRESS, self.TEMP_OUT_H, 2)
        except:
            print('Read raw IMU data failed')

        # Convert the raw values to something a little more useful (middle value is temperature)
        self.temp = self.eightBit2sixteenBit(rawData[1], rawData[0])
        
    def readRawGyro(self):
        # Read 6 raw values [High Low] gyro registries
        try:
            if self.Interrupt is True:
                if self.GPIO.wait_for_edge(self.RPi_GPIO, self.GPIO.RISING, timeout=100) is None:
                    print('tiempo agotado para interrupcion')

            rawData = self.bus.read_i2c_block_data(self.MPU9250_ADDRESS, self.GYRO_XOUT_H, 6)
        except:
            print('Read raw IMU data failed')

        self.gx = self.eightBit2sixteenBit(rawData[1], rawData[0])
        self.gy = self.eightBit2sixteenBit(rawData[3], rawData[2])
        self.gz = self.eightBit2sixteenBit(rawData[5], rawData[4])

    def readRawMag(self):
        # Prepare to request values
        self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_ADDR, self.AK8963_ADDRESS | 0x80);    # Set the I2C slave address of AK8963 and set for read.
        self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_REG, self.AK8963_XOUT_L);             # I2C slave 0 register address from where to begin data transfer
        self.bus.write_byte_data(self.MPU9250_ADDRESS, self.I2C_SLV0_CTRL, 0x87);                          # Enable I2C and read 7 bytes
        time.sleep(0.02)

        # Read 7 values [Low High] and one more byte (overflow check)
        try:
            rawData = self.bus.read_i2c_block_data(self.MPU9250_ADDRESS, self.EXT_SENS_DATA_00, 7)
        except:
            print('Read raw MAG data failed')

        # If overflow check passes convert the raw values to something a little more useful
        if not (rawData[6] & 0x08):
            self.mx = self.eightBit2sixteenBit(rawData[0], rawData[1])
            self.my = self.eightBit2sixteenBit(rawData[2], rawData[3])
            self.mz = self.eightBit2sixteenBit(rawData[4], rawData[5])

    def calibrateGyro(self, N):
        # Display message
        print("Calibrating gyro with " + str(N) + " points. Do not move!")
        '''valores encntrados a mano armar una funcion que minimice la deriva'''
        self.bus.write_i2c_block_data(self.MPU9250_ADDRESS, self.XG_OFFSET_H, [0,0,0,0,0,0])
        

        # Take N readings for each coordinate and add to itself
        for ii in range(N):
            self.readRawGyro()
            self.gyroXcal += self.gx
            self.gyroYcal += self.gy
            self.gyroZcal += self.gz

        # Find average offset value ¿para corregir la deriva tempotal?
        self.gyroXcal /= N
        self.gyroYcal /= N
        self.gyroZcal /= N

        # Display message and restart timer for comp filter
        print("Calibration complete")
        print("\tX axis offset: " + str(round(self.gyroXcal,1)))
        print("\tY axis offset: " + str(round(self.gyroYcal,1)))
        print("\tZ axis offset: " + str(round(self.gyroZcal,1)) + "\n")

        # Start the timer
        self.dtTimer = time.perf_counter()

    def calibrateMag(self, N):
        # Local calibration variables
        magBias = [0, 0, 0]
        magScale = [0, 0, 0]
        magMin = [32767, 32767, 32767]
        magMax = [-32767, -32767, -32767]
        magTemp = [0, 0, 0]

        # Take N readings of mag data
        for ii in range(N):
            # Read fresh values and assign to magTemp
            self.readRawMag()
            magTemp = [self.mx, self.my, self.mz]

            # Adjust the max and min points based off of current reading
            for jj in range(3):
                if (magTemp[jj] > magMax[jj]):
                    magMax[jj] = magTemp[jj]
                if (magTemp[jj] < magMin[jj]):
                    magMin[jj] = magTemp[jj]

            # Display some info to the user
            print(str(self.mx)+','+str(self.my)+','+str(self.mz))

            # Small delay before next loop (data available every 10 ms or 100 Hz)
            time.sleep(0.012)

        # Get hard iron correction
        self.magXbias = ((magMax[0] + magMin[0])/2) * self.magScaleFactor * self.magXcal
        self.magYbias = ((magMax[1] + magMin[1])/2) * self.magScaleFactor * self.magYcal
        self.magZbias = ((magMax[2] + magMin[2])/2) * self.magScaleFactor * self.magZcal

        # Get soft iron correction estimate
        magXchord = (magMax[0] - magMin[0])/2
        magYchord = (magMax[1] - magMin[1])/2
        magZchord = (magMax[2] - magMin[2])/2

        avgChord = (magXchord + magYchord + magZchord)/3

        self.magXscale = avgChord/magXchord
        self.magYscale = avgChord/magYchord
        self.magZscale = avgChord/magZchord

    def calibrateMagGuide(self):
        # Display message
        print("Magnetometer calibration. Wave and rotate device in a figure eight until notified.\n")
        time.sleep(3)

        # Run the first calibration
        self.calibrateMag(3000)

        # Display results to user
        print("\nCalibration complete:")
        print("\tmagXbias = " + str(round(self.magXbias,3)))
        print("\tmagYbias = " + str(round(self.magYbias,3)))
        print("\tmagZbias = " + str(round(self.magZbias,3)) + "\n")

        print("\tmagXscale = " + str(round(self.magXscale,3)))
        print("\tmagYscale = " + str(round(self.magYscale,3)))
        print("\tmagZscale = " + str(round(self.magZscale,3)) + "\n")

        # Give more instructions to the user
        print("Place above values in magCalVisualizer.py and magCalSlider.py")
        print("Recording additional 1000 data points to verify the calibration")
        print("Repeat random figure eight pattern and rotations...\n")
        time.sleep(3)

        # Run the scecond calibration
        self.calibrateMag(1000)

        # Provide final instructions
        print("\nCopy the raw values into data.txt")
        print("Run magCalVisualizer.py and magCalSlider.py to validate calibration success")
        print("See the README for more information")
        print("Also compare the second calibration to the first:")

        # Display the second results
        print("\tmagXbias = " + str(round(self.magXbias,3)))
        print("\tmagYbias = " + str(round(self.magYbias,3)))
        print("\tmagZbias = " + str(round(self.magZbias,3)) + "\n")

        print("\tmagXscale = " + str(round(self.magXscale,3)))
        print("\tmagYscale = " + str(round(self.magYscale,3)))
        print("\tmagZscale = " + str(round(self.magZscale,3)) + "\n")

        # End program
        print("Terminating program now!")
        quit()

    def setMagCalibration(self, bias, scale):
        # Set the bias variables in all 3 axis
        self.magXbias = bias[0]
        self.magYbias = bias[1]
        self.magZbias = bias[2]

        # Set the scale variables in all 3 axis
        self.magXscale = scale[0]
        self.magYscale = scale[1]
        self.magZscale = scale[2]

    def processValues(self):
        # Update the raw data
        self.readRawIMU()
        self.readRawMag()

        # Subtract the offset calibration values for the gyro
        self.gx -= self.gyroXcal
        self.gy -= self.gyroYcal
        self.gz -= self.gyroZcal

        # Convert the gyro values to degrees per second
        self.gx /= self.gyroScaleFactor
        self.gy /= self.gyroScaleFactor
        self.gz /= self.gyroScaleFactor

        # Convert the accelerometer values to g force
        self.ax /= self.accScaleFactor
        self.ay /= self.accScaleFactor
        self.az /= self.accScaleFactor

        # Process mag values in milliGauss
        # Include factory calibration per data sheet and user environmental corrections
        self.mx = self.mx * self.magScaleFactor * self.magXcal - self.magXbias
        self.my = self.my * self.magScaleFactor * self.magYcal - self.magYbias
        self.mz = self.mz * self.magScaleFactor * self.magZcal - self.magZbias

        self.mx *= self.magXscale
        self.my *= self.magYscale
        self.mz *= self.magZscale

    def processGyro(self):
        # Update the raw data
        self.readRawGyro()

        # Subtract the offset calibration values for the gyro
        self.gx -= self.gyroXcal
        self.gy -= self.gyroYcal
        self.gz -= self.gyroZcal

        # Convert the gyro values to degrees per second
        self.gx /= self.gyroScaleFactor
        self.gy /= self.gyroScaleFactor
        self.gz /= self.gyroScaleFactor


    def integartorFilter(self):
        # Get the processed values from Gyro
        self.processGyro()

        # Get delta time and record time for next call
        dt = time.perf_counter() - self.dtTimer
        self.dtTimer = time.perf_counter()

        # Gyro integration angle
        self.gyroRoll += self.gx * dt
        self.gyroPitch += self.gy * dt
        self.gyroYaw += self.gz * dt
