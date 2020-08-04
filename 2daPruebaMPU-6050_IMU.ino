///////////////////////////////////////////////////////////////////////////////////////
//Connections
///////////////////////////////////////////////////////////////////////////////////////
/*Power (5V) is provided to the Arduino pro mini by the FTDI programmer

Gyro - Arduino pro mini
VCC  -  5V
GND  -  GND
SDA  -  A4
SCL  -  A5

LCD  - Arduino pro mini
VCC  -  5V
GND  -  GND
SDA  -  A4
SCL  -  A5
*//////////////////////////////////////////////////////////////////////////////////////

#include <Wire.h>

//Declaring some global variables
int gyro_x, gyro_y, gyro_z;
float q1=1.0, q2=0.0, q3=0.0, q4=0.0;
float q1_buffer, q2_buffer, q3_buffer, q4_buffer;
long acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
float gyro_x_cal = 0, gyro_y_cal = 0, gyro_z_cal = 0;
long gyro_imu_x_cal = 0, gyro_imu_y_cal = 0, gyro_imu_z_cal = 0;
long loop_timer, serial_timer;
int loop_time_max, loop_time_min, loop_time;
int serial_loop_counter = 0;
float angle_pitch, angle_roll, angle_yaw;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc, angle_yaw_acc;
float angle_pitch_output, angle_roll_output, angle_yaw_output;
float angle_pitch_buffer, angle_roll_buffer, angle_yaw_buffer;

char MPU9250_ADDRESS    = 0x68;
char AK8963_ADDRESS     = 0x0C;
char XG_OFFSET_H        = 0x13;       //6 Bytes: XG_OFFSET_H, XG_OFFSET_L, YG_OFFSET_H, YG_OFFSET_L, ZG_OFFSET_H, ZG_OFFSET_L
char SMPLRT_DIV         = 0x19;
char CONFIG             = 0x1A;       //GYRO_LPF
char GYRO_CONFIG        = 0x1B;
char ACCEL_CONFIG       = 0x1C;
char ACCEL_CONFIG2      = 0x1d;
char FIFO_EN            = 0x23;
char I2C_MST_CTRL       = 0x24;
char I2C_SLV0_ADDR      = 0x25;
char I2C_SLV0_REG       = 0x26;
char I2C_SLV0_CTRL      = 0x27;
char INT_PIN_CFG        = 0x37;
char INT_ENABLE         = 0x38;
char INT_STATUS         = 0x3A;
char ACCEL_XOUT_H       = 0x3B;       //6 Bytes: ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L, ACCEL_ZOUT_H, ACCEL_ZOUT_L
char TEMP_OUT_H         = 0x41;       //2 Bytes: TEMP_OUT_H, TEMP_OUT_L
char GYRO_XOUT_H        = 0x43;       //6 Bytes: GYRO_XOUT_H, GYRO_XOUT_L, GYRO_YOUT_H, GYRO_YOUT_L, GYRO_ZOUT_H, GYRO_ZOUT_L
char EXT_SENS_DATA_00   = 0x49;
char I2C_SLV0_DO        = 0x63;
char I2C_MST_DELAY_CTRL = 0X67; 
char SIGNAL_PATH_RESET  = 0x68;
char USER_CTRL          = 0x6A;
char PWR_MGMT_1         = 0x6B;
char PWR_MGMT_2         = 0x6C;
char FIFO_COUNTH        = 0x72;       //2 Bytes: FIFO_COUNTH, FIFO_COUNTL
char FIFO_R_W           = 0x74;
char WHO_AM_I_MPU9250   = 0x75;

char WHO_AM_I_AK8963    = 0x00;
char AK8963_XOUT_L      = 0x03;       //6 Bytes: HXL, HXH, HYH, HZL, HZH
char AK8963_CNTL        = 0x0A;
char AK8963_CNTL2       = 0x0B;
char AK8963_ASAX        = 0x10;       //3 Bytes: ASAX, ASAY, ASAZ

char MPU9250_SAMPLERATE_MIN = 5;      // samples per second is the lowest
char MPU9250_SAMPLERATE_MAX = 32000;  //  samples per second is the absolute maximum

char MPU9250_COMPASSRATE_MIN = 1;     // samples per second is the lowest
char MPU9250_COMPASSRATE_MAX = 100;   // samples per second is maximum

int retardo = 2000;                   //tiempo en uSeg que demora el bucle principal.
int retardo_serial = 100;             //tiempo en mSeg que demora para enviar una cadena de informacion por serie.
int FS_SEL;
int pushButton = 3;
float gyro_scale_factor_X_delta_T;
float gyro_scale_factor_X_delta_T_to_rad;
float delta_t;
int first_cal_n = 100;
int second_cal_n = 2000;

void setup() {
  Wire.begin();                                                        //Start I2C as master
  Wire.setClock(500000);                                               //500kHz 

  Serial.begin(115200);                                                //Use only for debugging
  pinMode(13, OUTPUT);                                                 //Set output 13 (LED) as output
  pinMode(pushButton, INPUT_PULLUP);
  
  digitalWrite(13, HIGH);                                              //Set digital output 13 high to indicate startup

  setup_mpu_6050_registers();                                          //Setup the registers of the MPU-6050 (500dfs / +/-8g) and start the gyro
  
  calibrate_mpu_6050();

  angle_pitch = 0;                                                     //Set the gyro angle equal to the accelerometer pitch angle 
  angle_roll = 0;                                                      
  angle_yaw = 0;
  
  loop_timer = micros();                                               //Reset the loop timer
  serial_timer = millis();
  serial_loop_counter = 0;
  digitalWrite(13, LOW);                                               //All done, turn the LED off
}

void loop(){
  //read_mpu_6050_data();                                              //Read the raw acc and gyro data from the MPU-6050
  read_mpu_6050_gyro();                                                //Read the raw gyro data from the MPU-6050

  gyro_x -= gyro_x_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
  gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
  gyro_z -= gyro_z_cal;                                                //Subtract the offset calibration value from the raw gyro_z value
  
  //Gyro angle calculations
  
  angle_pitch += gyro_x * gyro_scale_factor_X_delta_T;                 //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll  += gyro_y * gyro_scale_factor_X_delta_T;                              
  angle_yaw   += gyro_z * gyro_scale_factor_X_delta_T;

  //angle_pitch += angle_roll * sin(gyro_z * gyro_scale_factor_X_delta_T_to_rad);               //If the IMU has yawed transfer the roll angle to the pitch angel
  //angle_roll -= angle_pitch * sin(gyro_z * gyro_scale_factor_X_delta_T_to_rad);               //If the IMU has yawed transfer the pitch angle to the roll angel

  update_quaternion();
  
  if(digitalRead(pushButton) == false){                                                              
    angle_pitch = 0;                                      //Set the gyro angle equal to 0 
    angle_roll = 0;
    angle_yaw = 0;    
  }

  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value
  angle_yaw_output = angle_yaw_output * 0.9 + angle_yaw * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value
    
  write_serial();
  
  loop_time = micros() - loop_timer;
  if(loop_time > loop_time_max) loop_time_max = loop_time;
  if(loop_time < loop_time_min) loop_time_min = loop_time;

  while(micros() - loop_timer < retardo);                              //Wait until the loop_timer reaches retardous before starting the next loop
  loop_timer = micros();                                               //Reset the loop timer
}

void update_quaternion(){
  float Q1=q1 ,Q2=q2 ,Q3=q3, Q4=q4;
  float qDot1=q1, qDot2=q2, qDot3=q3, qDot4=q4;
  qDot1 = (-Q2 * gyro_x - Q3 * gyro_y - Q4 * gyro_z);
  qDot2 = ( Q1 * gyro_x + Q3 * gyro_z - Q4 * gyro_y);
  qDot3 = ( Q1 * gyro_y - Q2 * gyro_z + Q4 * gyro_x);
  qDot4 = ( Q1 * gyro_z + Q2 * gyro_y - Q3 * gyro_x);

  Q1 += qDot1 * gyro_scale_factor_X_delta_T;
  Q2 += qDot2 * gyro_scale_factor_X_delta_T;
  Q3 += qDot3 * gyro_scale_factor_X_delta_T;
  Q4 += qDot4 * gyro_scale_factor_X_delta_T;
  float norm = sqrt(pow(Q1,2) + pow(Q2,2) + pow(Q3,2) + pow(Q4,2));
  q1= Q1 / norm;
  q2= Q2 / norm;
  q3= Q3 / norm;
  q4= Q4 / norm;
  
  }

void write_serial(){                                                      //Subroutine for writing the LCD
  //Writing multiple characters is taking to much time
  if (millis() - serial_timer > retardo_serial){
    serial_loop_counter = 0;                                              //Reset the counter after n microseg
    serial_timer = millis();
  }
  switch (serial_loop_counter) {
    case 0:
    angle_pitch_buffer = angle_pitch_output;
    angle_roll_buffer = angle_roll_output;
    angle_yaw_buffer = angle_yaw_output;
    q1_buffer = q1;
    q2_buffer = q2;
    q3_buffer = q3;
    q4_buffer = q4;
    break;
    /*
    case 1:
    Serial.print("Pitch: ");
    break;
    
    case 2:
    Serial.print(angle_pitch_buffer,2);
    break;
    
    case 3:
    Serial.print(" Roll: "); 
    break;
    
    case 4:
    Serial.print(angle_roll_buffer, 2);
    break;
    
    case 5:
    Serial.print(" Yaw: ");
    break;
    
    case 6:
    Serial.println(angle_yaw_buffer, 2);
    break;
    
    case 7:
    Serial.print(" Time_min: ");
    break;
    
    case 8:
    Serial.print(loop_time_min);
    loop_time_min = retardo;
    break;
    
    case 9:
    Serial.print(" Time_max: ");
    break;
  
    case 10:
    Serial.println(loop_time_max);
    loop_time_max = 0;
    break;
    */
    case 11:
    Serial.print("q1: ");
    break;
    
    case 12:
    Serial.print(q1_buffer,2);
    break;
    
    case 13:
    Serial.print(" q2: "); 
    break;
    
    case 14:
    Serial.print(q2_buffer, 2);
    break;
    
    case 15:
    Serial.print(" q3: ");
    break;
    
    case 16:
    Serial.print(q3_buffer, 2);
    break;
    
    case 17:
    Serial.print(" q4: ");
    break;
    
    case 18:
    Serial.println(q4_buffer, 2);
    break;
    
    }
  serial_loop_counter ++;                                              //Increase the counter
}

void read_mpu_6050_data(){                                             //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  acc_x = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_x variable
  acc_y = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_y variable
  acc_z = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_z variable
  temperature = Wire.read()<<8|Wire.read();                            //Add the low and high byte to the temperature variable
  gyro_x = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_x variable
  gyro_y = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_y variable
  gyro_z = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_z variable

}

void read_mpu_6050_gyro(){                                             //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(GYRO_XOUT_H);                                             //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68,6);                                            //Request 6 bytes from the MPU-6050
  while(Wire.available() < 6);                                         //Wait until all the bytes are received
  gyro_x = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_x variable
  gyro_y = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_y variable
  gyro_z = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_z variable
}

void calibrate_mpu_6050(){
  Serial.println("Calibrating gyro with N points. Do not move!");
  //''' Ver registro 27 Register Map '''
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68,1);                                            //
  while(Wire.available() < 1);                                         //Wait until all the bytes are received
  FS_SEL = Wire.read()>>3;
  delta_t = retardo * pow(10,-6);
  /*
  FS_SEL Full Scale Range LSB Sensitivity
  0 ± 250 °/s 131 LSB/°/s
  1 ± 500 °/s 65.5 LSB/°/s
  2 ± 1000 °/s 32.8 LSB/°/s
  3 ± 2000 °/s 16.4 LSB/°/s
  */
  switch (FS_SEL){
    case 0:
      gyro_scale_factor_X_delta_T = delta_t / 131;
      break;
    case 1:
      gyro_scale_factor_X_delta_T = delta_t / 65.5;
      break;
    case 2:
      gyro_scale_factor_X_delta_T = delta_t / 32.8;
      break;
    case 3:
      gyro_scale_factor_X_delta_T = delta_t / 16.4;
      break;
    default:
      Serial.println("Error FS_SEL Read");
      break;
    }
  gyro_scale_factor_X_delta_T_to_rad = gyro_scale_factor_X_delta_T * DEG_TO_RAD;
  Serial.print("gyro_scale_factor_X_delta_T: "); Serial.println(gyro_scale_factor_X_delta_T,24);

  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x13); //
  Wire.write(0x00); //XG_OFFSET_H
  Wire.write(0x00); //XG_OFFSET_L
  Wire.write(0x00); //YG_OFFSET_H
  Wire.write(0x00); //YG_OFFSET_L
  Wire.write(0x00); //ZG_OFFSET_H
  Wire.write(0x00); //ZG_OFFSET_L
  Wire.endTransmission();                                              //End the transmission
  delay(100);
  loop_timer = micros();                                               //Reset the loop timer
  
  for (int cal_int = 0; cal_int < first_cal_n ; cal_int ++){                  //Run this code 2000 times
    read_mpu_6050_gyro();                                              //Read the raw acc and gyro data from the MPU-6050
    gyro_imu_x_cal += gyro_x;                                          //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_imu_y_cal += gyro_y;                                          //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_imu_z_cal += gyro_z;                                          //Add the gyro z-axis offset to the gyro_z_cal variable
    while(micros() - loop_timer < retardo);                            //Wait until the loop_timer reaches retardous (250Hz) before starting the next loop
    loop_timer = micros();                                             //Reset the loop timer
  }

  gyro_imu_x_cal /= first_cal_n*4/pow(2,FS_SEL);                              //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_imu_y_cal /= first_cal_n*4/pow(2,FS_SEL);                              //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_imu_z_cal /= first_cal_n*4/pow(2,FS_SEL);                              //Divide the gyro_z_cal variable by 2000 to get the avarage offset
  
  Serial.println("First Calibration");
  Serial.println(gyro_imu_x_cal);
  Serial.println(gyro_imu_y_cal);
  Serial.println(gyro_imu_z_cal);

  byte hx,lx,hy,ly,hz,lz;
  
  if (-32768 < int(gyro_imu_x_cal) < 3276){
      lx = int(-gyro_imu_x_cal) & 0x00ff;
      hx = int(-gyro_imu_x_cal) >> 8;
  }
  else{
      Serial.println("X Overflow");
  }
  if (-32768 < int(gyro_imu_y_cal) < 3276){
      ly = int(-gyro_imu_y_cal) & 0x00ff;
      hy = int(-gyro_imu_y_cal) >> 8;
  }
  else{
      Serial.println("Y Overflow");
  }
  if (-32768 < int(gyro_imu_z_cal) < 3276){
      lz = int(-gyro_imu_z_cal) & 0x00ff;
      hz = int(-gyro_imu_z_cal) >> 8;
  }
  else{
      Serial.println("Z Overflow");
  }

  Wire.beginTransmission(0x68);                                 //Start communicating with the MPU-6050
  Wire.write(0x13); //
  Wire.write(hx); //XG_OFFSET_H
  Wire.write(lx); //XG_OFFSET_L
  Wire.write(hy); //YG_OFFSET_H
  Wire.write(ly); //YG_OFFSET_L
  Wire.write(hz); //ZG_OFFSET_H
  Wire.write(lz); //ZG_OFFSET_L
  Wire.endTransmission();

  loop_timer = micros();                                               //Reset the loop timer
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++){                  //Run this code 2000 times
    read_mpu_6050_gyro();                                              //Read the raw acc and gyro data from the MPU-6050
    gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
    while(micros() - loop_timer < retardo);                            //Wait until the loop_timer reaches retardo us (250Hz) before starting the next loop
    loop_timer = micros();                                             //Reset the loop timer
  }
  gyro_x_cal /= 2000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal /= 2000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal /= 2000;                                                  //Divide the gyro_z_cal variable by 2000 to get the avarage offset
 
  Serial.println("Second Calibration");
  Serial.println(gyro_x_cal,24);
  Serial.println(gyro_y_cal,24);
  Serial.println(gyro_z_cal,24);
}

void setup_mpu_6050_registers(){
  //Reset the MPU-6050
  Wire.beginTransmission(MPU9250_ADDRESS);                             //Start communicating with the MPU-6050
  Wire.write(PWR_MGMT_1);                                              //Send the requested starting register
  Wire.write(0x80);                                                    //DEVICE_RESET
  Wire.endTransmission();                                              //End the transmission
  Wire.beginTransmission(MPU9250_ADDRESS);                             //Start communicating with the MPU-6050
  delay(100);
  Wire.write(SIGNAL_PATH_RESET);                                       //Send the requested starting register
  Wire.write(0x03);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  delay(100);
  
  Wire.beginTransmission(MPU9250_ADDRESS);                             //Start communicating with the MPU-6050
  Wire.write(PWR_MGMT_1);                                              //Send the requested starting register
  Wire.write(0x00);                                                    //0 Internal 8MHz oscillator
  Wire.endTransmission();                                              //End the transmission

  //  SAMPLE_RATE = 1kHz;  DLPF_CFG = 20 Hz
  Wire.beginTransmission(MPU9250_ADDRESS);                             //Start communicating with the MPU-6050
  Wire.write(SMPLRT_DIV);                                              //Send the requested starting register
  Wire.write(0x00);                                                    //SAMPLE_RATE=Internal_Sample_Rate / (1 + SMPLRT_DIV)
  Wire.endTransmission();                                              //End the transmission
  Wire.beginTransmission(MPU9250_ADDRESS);                             //Start communicating with the MPU-6050
  Wire.write(CONFIG);                                                  //Send the requested starting register
  Wire.write(0x04);                                                    //EXT_SYNC_SET 0 Input disabled + DLPF_CFG 20 Hz 
  Wire.endTransmission();                                              //End the transmission

  Wire.beginTransmission(MPU9250_ADDRESS);                             //Start communicating with the MPU-6050
  Wire.write(INT_PIN_CFG);                                             //Send the requested starting register
  Wire.write(0x00);                                                    //
  Wire.endTransmission();                                              //End the transmission
  Wire.beginTransmission(MPU9250_ADDRESS);                             //Start communicating with the MPU-6050
  Wire.write(INT_ENABLE);                                              //Send the requested starting register
  Wire.write(0x01);                                                    //DATA_RDY_EN
  Wire.endTransmission();                                              //End the transmission

  Wire.beginTransmission(MPU9250_ADDRESS);                             //Start communicating with the MPU-6050
  Wire.write(FIFO_EN);                                                 //Send the requested starting register
  Wire.write(0x00);                                                    //Nadie escribe en la FIFO
  Wire.endTransmission();                                              //End the transmission

  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(MPU9250_ADDRESS);                             //Start communicating with the MPU-6050
  Wire.write(ACCEL_CONFIG);                                            //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the gyro (2000dps full scale)
  Wire.beginTransmission(MPU9250_ADDRESS);                             //Start communicating with the MPU-6050
  Wire.write(GYRO_CONFIG);                                             //Send the requested starting register
  Wire.write(0x18);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
}
