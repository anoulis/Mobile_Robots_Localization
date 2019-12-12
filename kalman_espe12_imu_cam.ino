#include <ArduinoJson.h>
#include <MatrixMath.h>
#include <Wire.h>
#include <MPU6050.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#define left_wheel 15
#define right_wheel 13
#define INTERVAL_MESSAGE1 500
#define forward 1
#define right 2
#define left 3
#define stopped 4
unsigned long time_1 = 0;

MPU6050 mpu;
int SCL_PIN=D1;
int SDA_PIN=D2;
const int MPU_addr=0x68;
const char* ssid     = "home";
const char* password = "fiap4202";

//const char* mqttServer = "192.168.0.105";
const char* mqttServer = "192.168.43.112";
const int mqttPort = 1883;
const char* mqttUser = "username";
const char* mqttPassword = "password";
int incomingByte = 0;
boolean camera_flag ;

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long now, lastTime = 0;
int16_t ax, ay, az, gx, gy, gz;             // Accelerometer gyroscope raw data
double aax=0, aay=0,aaz=0, agx=0, agy=0, agz=0;    // Angle variable
double roll=0, pitch=0, yaw=0;
long axo = 0, ayo = 0, azo = 0;             // Accelerometer offset
long gxo = 0, gyo = 0, gzo = 0;             // Gyro offset
int incr =0; String command;
int y=0;
long choice; boolean flag = true;

double distx=0;
double velx=0;

double pi = 3.1415926;
double AcceRatio = 16384.0;            // Accelerometer scale factor (to convert data form g to in m/s^2)
//double AcceRatio = 1670.13;   
double GyroRatio = 131.0;                    // Gyroscope scale factor (to convert data in m/s)


//sigma variables

double sigma2imuacc=0.002642197;
double sigma2imuomega=3.52855e-6;

double sigma2camx=0.00001; //
double sigma2camy=0.00001; //
double sigma2camtheta=0.00001;   ///ATTENTION

//measures variables

double omegaimu;
double accimu;
double ximu, yimu, thetaimu, vimu;
double xcam, ycam, thetacam;
double dt;
double xrobot;
double yrobot;
double thetarobot;
double guardAngle; double upperLimit; double lowerLimit; double turnAngle;

//matrices initialization

double state[4][1];
double Pk[4][4];
double A[4][4];
double B[4][2];
double u[2][1];
double H[3][4];
double Qimu[2][2];
double R[3][3];
double z[3][1];

double I[4][4];

//matrices for kalman calculations inside the loop

double stateknext[4][1];
double Pkpiu1[4][4];

double savestate[4][1];
double savePk[4][4];


void setup() {
  Wire.begin();
  Serial.begin(115200);

  pinMode(right_wheel, OUTPUT);
  pinMode(left_wheel, OUTPUT);
  
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
    if (client.connect("ESP8266Client", mqttUser, mqttPassword )) {
      Serial.println("connected");yield();
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());yield();
      delay(2000);
    }
  }
  client.publish("test", "we are testing the comms");
  //client.subscribe("positions");
  client.subscribe("positions");
 delay(15000);
  Serial.println("Initialize MPU6050");
  while(!mpu.beginSoftwareI2C(SCL_PIN,SDA_PIN,MPU6050_SCALE_250DPS , MPU6050_RANGE_2G,MPU_addr))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
    }

    // Calibration
    unsigned short times = 1000;             // The number of samples
    for(int i=0;i<times;i++)
    {
        Vector RawAccel = mpu.readRawAccel();
        Vector RawGyro = mpu.readRawGyro();
        //accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // Read the six-axis original value
        ax = RawAccel.XAxis;  ay = RawAccel.YAxis;  az = RawAccel.ZAxis;
        gx = RawGyro.XAxis;  gy = RawGyro.YAxis;  gz = RawGyro.ZAxis;
        axo += ax; ayo += ay; azo += az;      // Sampling
        gxo += gx; gyo += gy; gzo += gz;
    }
    axo /= times; ayo /= times; azo /= times; // Calculate accelerometer offset
    gxo /= times; gyo /= times; gzo /= times; // Calculate the gyro offset

//Variables initialization
  xcam=0;
  ycam=0;
  thetacam=0;

  ximu=0;
  yimu=0;
  thetaimu=0;
  vimu=0;
  accimu=0;
  omegaimu=0;
  
  //State initialization
  state[0][0]=xcam;
  state[1][0]=ycam;
  state[2][0]=vimu;
  state[3][0]=thetacam;
  //Matrix.Print((double*)state, 4, 1, "state");
  
  //A matrix initialization
  A[0][0]=1; A[0][1]=0; A[0][2]=cos(state[3][0])*dt; A[0][3]=0;
  A[1][0]=0; A[1][1]=1; A[1][2]=sin(state[3][0])*dt; A[1][3]=0;  
  A[2][0]=0; A[2][1]=0; A[2][2]=1; A[2][3]=0;
  A[3][0]=0; A[3][1]=0; A[3][2]=0; A[3][3]=1;
  //Matrix.Print((double*)A, 4, 4, "A");

  //B matrix initialization
  B[0][0]=0; B[0][1]=0;
  B[1][0]=0; B[1][1]=0;
  B[2][0]=dt; B[2][1]=0; 
  B[3][0]=0; B[3][1]=dt;
  //Matrix.Print((double*)B, 4, 2, "B");
  
  //input vector u initialization
  u[0][0]=accimu;
  u[1][0]=omegaimu;
  //Matrix.Print((double*)u, 2, 1, "u");
  
  //H matrix
  H[0][0]=1; H[0][1]=0; H[0][2]=0; H[0][3]=0;
  H[1][0]=0; H[1][1]=1; H[1][2]=0; H[1][3]=0;
  H[2][0]=0; H[2][1]=0; H[2][2]=0; H[2][3]=1;
  //Matrix.Print((double*)H, 3, 4, "H");

  //Qimu
  Qimu[0][0]=sigma2imuacc; Qimu[0][1]=0;
  Qimu[1][0]=0; Qimu[1][1]=sigma2imuomega;
  //Matrix.Print((double*)Qimu, 2, 2, "Qimu");

  //P matrix
  Pk[0][0]=0.1;
  Pk[0][1]=0.1;
  Pk[0][2]=0.1;
  Pk[0][3]=0.1;
  Pk[1][0]=0.1;
  Pk[1][1]=0.1;
  Pk[1][2]=0.1;
  Pk[1][3]=0.1;
  Pk[2][0]=0.1;
  Pk[2][1]=0.1;
  Pk[2][2]=0.1;
  Pk[2][3]=0.1;
  Pk[3][0]=0.1;
  Pk[3][1]=0.1;
  Pk[3][2]=0.1;
  Pk[3][3]=0.1;
  //Matrix.Print((double*)Pk, 4, 4, "Pk");

  //Identity Matrix
  I[0][0]=1;
  I[0][1]=0;
  I[0][2]=0;
  I[0][3]=0;
  I[1][0]=0;
  I[1][1]=1;
  I[1][2]=0;
  I[1][3]=0;
  I[2][0]=0;
  I[2][1]=0;
  I[2][2]=1;
  I[2][3]=0;
  I[3][0]=0;
  I[3][1]=0;
  I[3][2]=0;
  I[3][3]=1;

  //R camera matrix
  R[0][0]=sigma2camx; R[0][1]=0; R[0][2]=0;
  R[1][0]=0; R[1][1]=sigma2camy; R[1][2]=0;
  R[2][0]=0; R[2][1]=0; R[2][2]=sigma2camtheta;
  //Matrix.Print((double*)R, 3, 3, "R");
  
  //z camera measures
  z[0][0]=xcam;
  z[1][0]=ycam;
  z[2][0]=thetacam;
  //Matrix.Print((double*)z, 3, 1, "z");

  //INITIALIZATION
  Matrix.Copy((double*)state, 4, 1, (double*)savestate);
  Matrix.Copy((double*)Pk, 4, 4, (double*)savePk);
  //Serial.println("In50");
 
  upperLimit=pi; 
  lowerLimit=0; 
  turnAngle=pi;

}

void loop() {
  client.loop();
  kalman();
  walkerMoves();
  guardMoves();
  //circlemove();
  //moveAround();
  //moves();
}

void kalman(){
//matrices for calculations
double Astate[4][1]; 
double Bu[4][1]; 
double statekpiu1meno[4][1];
double Atranspose[4][4];
double APk[4][4];
double APkAtr[4][4];
double Btranspose[2][4];
double BQimu[4][2];
double BQimuBtr[4][4];
double Pkpiu1meno[4][4];
double Htranspose[4][3];
double HPk[3][4];
double HPkHtr[3][3];
double Skpiu1[3][3];
double PkHtr[4][3];      
double Skpiu1inv[3][3];
double Wkpiu1[4][3];
double Hstate[3][1];
double zminusHst[3][1];
double WkzminusH[4][1];
double WkH[4][4];
double IminusWkH[4][4];

    unsigned long now = millis();
    dt = (now - lastTime)/ 1000.0;           // Differential time(s)
    lastTime = now;  

    //taking new measures
    Vector RawGyro = mpu.readRawGyro();
    Vector RawAccel = mpu.readRawAccel();
    ax = RawAccel.XAxis;  ay = RawAccel.YAxis;  az = RawAccel.ZAxis;
    gx = RawGyro.XAxis;  gy = RawGyro.YAxis;  gz = RawGyro.ZAxis;
    omegaimu=((gz-gzo)/GyroRatio)/360*2*pi;
    accimu=(ax-axo)/AcceRatio;


    //updating state and Pk
    Matrix.Copy((double*)savestate, 4, 1, (double*)state);
    Matrix.Copy((double*)savePk, 4, 4, (double*)Pk);
    
    //updating matrices

    A[0][2]=cos(state[3][0])*dt;
    A[1][2]=sin(state[3][0])*dt;
    B[2][0]=dt;
    B[3][1]=dt;
    u[0][0]=accimu;
    u[1][0]=omegaimu;
    z[0][0]=xcam;
    z[1][0]=ycam;
    z[2][0]=thetacam;
    
    //PREDICTION STEP
   
    //First row statekpiu1meno
    
    //Matrix.Print((double*)A, 4, 4, "A");
    //Matrix.Print((double*)B, 4, 2, "B");
    //Matrix.Print((double*)u, 2, 1, "u");
  
    Matrix.Multiply((double*) A, (double*) state, 4, 4, 1, (double*) Astate);
    //Matrix.Print((double*)Astate, 4, 1, "Astate");
    
    Matrix.Multiply((double*) B, (double*) u, 4, 2, 1, (double*) Bu);
    //Matrix.Print((double*)Bu, 4, 1, "Bu");
    
    Matrix.Add((double*) Astate, (double*) Bu, 4, 1, (double*) statekpiu1meno);
    //Matrix.Print((double*)statekpiu1meno, 4, 1, "statekpiu1meno");

    //Second row Pkpiu1meno
    
    Matrix.Transpose((double*) A, 4, 4, (double*) Atranspose);
    //Matrix.Print((double*)Atranspose, 4, 4, "Atranspose");
    
    Matrix.Transpose((double*) B, 4, 2, (double*) Btranspose);
    //Matrix.Print((double*)Btranspose, 2, 4, "Btranspose");

    Matrix.Multiply((double*) A, (double*) Pk, 4, 4, 4, (double*) APk);
    //Matrix.Print((double*)APk, 4, 4, "APk");
    
    Matrix.Multiply((double*) APk, (double*) Atranspose, 4, 4, 4, (double*) APkAtr);
    //Matrix.Print((double*)APkAtr, 4, 4, "APkAtr");

    Matrix.Multiply((double*) B, (double*) Qimu, 4, 2, 2, (double*) BQimu);
    //Matrix.Print((double*)BQimu, 4, 2, "BQimu");
    
    Matrix.Multiply((double*) BQimu, (double*) Btranspose, 4, 2, 4, (double*) BQimuBtr);
    //Matrix.Print((double*)BQimuBtr, 4, 4, "BQimuBtr");

    Matrix.Add((double*) APkAtr, (double*) BQimuBtr, 4, 4, (double*) Pkpiu1meno);
    //Matrix.Print((double*)Pkpiu1meno, 4, 4, "Pkpiu1meno");

    //UPDATE STEP
    
    //Third row Skpiu1
    
    Matrix.Transpose((double*) H, 3, 4, (double*) Htranspose);
    //Matrix.Print((double*)Htranspose, 4, 3, "Htranspose");
    
    Matrix.Multiply((double*) H, (double*) Pkpiu1meno, 3, 4, 4, (double*) HPk);
    //Matrix.Print((double*)HPk, 3, 4, "HPk");
    
    Matrix.Multiply((double*) HPk, (double*) Htranspose, 3, 4, 3, (double*) HPkHtr);
    //Matrix.Print((double*)HPkHtr, 3, 3, "HPkHtr");

    Matrix.Add((double*) HPkHtr, (double*) R, 3, 3, (double*) Skpiu1);
    //Matrix.Print((double*)Skpiu1, 3, 3, "Skpiu1");

    //Fourth row Wkpiu1
       
    Matrix.Multiply((double*) Pkpiu1meno, (double*) Htranspose, 4, 4, 3, (double*) PkHtr);
    //Matrix.Print((double*)PkHtr, 4, 3, "PkHtr");

    //Matrix.Copy((double*)Skpiu1, 3, 3, (double*)Skpiu1inv);
    //Matrix.Print((double*)Skpiu1inv, 3, 3, "Skpiu1inv"); 
    //Matrix.Invert((double*) Skpiu1inv, 3);
    //Matrix.Print((double*)Skpiu1inv, 3, 3, "Skpiu1inv");

    
Skpiu1inv[0][0] = (Skpiu1[1][1] * Skpiu1[2][2] - Skpiu1[1][2] * Skpiu1[2][1]) / (Skpiu1[0][0] * Skpiu1[1][1] * Skpiu1[2][2] - Skpiu1[0][0] * Skpiu1[1][2] * Skpiu1[2][1] - Skpiu1[1][0] * Skpiu1[0][1] * Skpiu1[2][2] + Skpiu1[2][0] * Skpiu1[0][1] * Skpiu1[1][2] + Skpiu1[1][0] * Skpiu1[0][2] * Skpiu1[2][1] - Skpiu1[2][0] * Skpiu1[0][2] * Skpiu1[1][1]);
Skpiu1inv[0][1] = (-Skpiu1[0][1] * Skpiu1[2][2] + Skpiu1[0][2] * Skpiu1[2][1]) / (Skpiu1[0][0] * Skpiu1[1][1] * Skpiu1[2][2] - Skpiu1[0][0] * Skpiu1[1][2] * Skpiu1[2][1] - Skpiu1[1][0] * Skpiu1[0][1] * Skpiu1[2][2] + Skpiu1[2][0] * Skpiu1[0][1] * Skpiu1[1][2] + Skpiu1[1][0] * Skpiu1[0][2] * Skpiu1[2][1] - Skpiu1[2][0] * Skpiu1[0][2] * Skpiu1[1][1]);
Skpiu1inv[0][2] = (Skpiu1[0][1] * Skpiu1[1][2] - Skpiu1[0][2] * Skpiu1[1][1]) / (Skpiu1[0][0] * Skpiu1[1][1] * Skpiu1[2][2] - Skpiu1[0][0] * Skpiu1[1][2] * Skpiu1[2][1] - Skpiu1[1][0] * Skpiu1[0][1] * Skpiu1[2][2] + Skpiu1[2][0] * Skpiu1[0][1] * Skpiu1[1][2] + Skpiu1[1][0] * Skpiu1[0][2] * Skpiu1[2][1] - Skpiu1[2][0] * Skpiu1[0][2] * Skpiu1[1][1]);
Skpiu1inv[1][0] = (-Skpiu1[1][0] * Skpiu1[2][2] + Skpiu1[1][2] * Skpiu1[2][0]) / (Skpiu1[0][0] * Skpiu1[1][1] * Skpiu1[2][2] - Skpiu1[0][0] * Skpiu1[1][2] * Skpiu1[2][1] - Skpiu1[1][0] * Skpiu1[0][1] * Skpiu1[2][2] + Skpiu1[2][0] * Skpiu1[0][1] * Skpiu1[1][2] + Skpiu1[1][0] * Skpiu1[0][2] * Skpiu1[2][1] - Skpiu1[2][0] * Skpiu1[0][2] * Skpiu1[1][1]);
Skpiu1inv[1][1] = (Skpiu1[0][0] * Skpiu1[2][2] - Skpiu1[0][2] * Skpiu1[2][0]) / (Skpiu1[0][0] * Skpiu1[1][1] * Skpiu1[2][2] - Skpiu1[0][0] * Skpiu1[1][2] * Skpiu1[2][1] - Skpiu1[1][0] * Skpiu1[0][1] * Skpiu1[2][2] + Skpiu1[2][0] * Skpiu1[0][1] * Skpiu1[1][2] + Skpiu1[1][0] * Skpiu1[0][2] * Skpiu1[2][1] - Skpiu1[2][0] * Skpiu1[0][2] * Skpiu1[1][1]);
Skpiu1inv[1][2] = (-Skpiu1[0][0] * Skpiu1[1][2] + Skpiu1[0][2] * Skpiu1[1][0]) / (Skpiu1[0][0] * Skpiu1[1][1] * Skpiu1[2][2] - Skpiu1[0][0] * Skpiu1[1][2] * Skpiu1[2][1] - Skpiu1[1][0] * Skpiu1[0][1] * Skpiu1[2][2] + Skpiu1[2][0] * Skpiu1[0][1] * Skpiu1[1][2] + Skpiu1[1][0] * Skpiu1[0][2] * Skpiu1[2][1] - Skpiu1[2][0] * Skpiu1[0][2] * Skpiu1[1][1]);
Skpiu1inv[2][0] = (Skpiu1[1][0] * Skpiu1[2][1] - Skpiu1[1][1] * Skpiu1[2][0]) / (Skpiu1[0][0] * Skpiu1[1][1] * Skpiu1[2][2] - Skpiu1[0][0] * Skpiu1[1][2] * Skpiu1[2][1] - Skpiu1[1][0] * Skpiu1[0][1] * Skpiu1[2][2] + Skpiu1[2][0] * Skpiu1[0][1] * Skpiu1[1][2] + Skpiu1[1][0] * Skpiu1[0][2] * Skpiu1[2][1] - Skpiu1[2][0] * Skpiu1[0][2] * Skpiu1[1][1]);
Skpiu1inv[2][1] = (-Skpiu1[0][0] * Skpiu1[2][1] + Skpiu1[0][1] * Skpiu1[2][0]) / (Skpiu1[0][0] * Skpiu1[1][1] * Skpiu1[2][2] - Skpiu1[0][0] * Skpiu1[1][2] * Skpiu1[2][1] - Skpiu1[1][0] * Skpiu1[0][1] * Skpiu1[2][2] + Skpiu1[2][0] * Skpiu1[0][1] * Skpiu1[1][2] + Skpiu1[1][0] * Skpiu1[0][2] * Skpiu1[2][1] - Skpiu1[2][0] * Skpiu1[0][2] * Skpiu1[1][1]);
Skpiu1inv[2][2] = (Skpiu1[0][0] * Skpiu1[1][1] - Skpiu1[0][1] * Skpiu1[1][0]) / (Skpiu1[0][0] * Skpiu1[1][1] * Skpiu1[2][2] - Skpiu1[0][0] * Skpiu1[1][2] * Skpiu1[2][1] - Skpiu1[1][0] * Skpiu1[0][1] * Skpiu1[2][2] + Skpiu1[2][0] * Skpiu1[0][1] * Skpiu1[1][2] + Skpiu1[1][0] * Skpiu1[0][2] * Skpiu1[2][1] - Skpiu1[2][0] * Skpiu1[0][2] * Skpiu1[1][1]);
  
   
    Matrix.Multiply((double*) PkHtr, (double*) Skpiu1inv, 4, 3, 3, (double*) Wkpiu1);
    //Matrix.Print((double*)Wkpiu1, 4, 3, "Wkpiu1");

    //Fifth row stateknext

    Matrix.Multiply((double*) H, (double*) statekpiu1meno, 3, 4, 1, (double*) Hstate);
    //Matrix.Print((double*)Hstate, 3, 1, "Hstate");

    Matrix.Subtract((double*) z, (double*) Hstate, 3, 1, (double*) zminusHst);
    //Matrix.Print((double*)zminusHst, 3, 1, "zminusHst");
    
    Matrix.Multiply((double*) Wkpiu1, (double*) zminusHst, 4, 3, 1, (double*) WkzminusH);
    //Matrix.Print((double*)WkzminusH, 4, 1, "WkzminusH");

    Matrix.Add((double*) statekpiu1meno, (double*) WkzminusH, 4, 1, (double*) stateknext);
    //Matrix.Print((double*)stateknext, 4, 1, "stateknext"); 

    //Sixth row Pkpiu1
   
    Matrix.Multiply((double*) Wkpiu1, (double*) H, 4, 3, 4, (double*) WkH);
    //Matrix.Print((double*)WkH, 4, 4, "WkH");

    Matrix.Copy((double*)WkH, 4, 4, (double*)IminusWkH);
    Matrix.SubID((double*)IminusWkH, 4, 4);
    //Matrix.Print((double*)IminusWkH, 4, 4, "IminusWkH");
    
    Matrix.Multiply((double*) IminusWkH, (double*) Pkpiu1meno, 4, 4, 4, (double*) Pkpiu1);
    //Matrix.Print((double*)Pkpiu1, 4, 4, "Pkpiu1");

    //saving the state
    Matrix.Copy((double*)stateknext, 4, 1, (double*)savestate);  
    //Saving the matrix P
    Matrix.Copy((double*)Pkpiu1meno, 4, 4, (double*)savePk);
    //Matrix.Copy((double*)Pkpiu1, 4, 4, (double*)savePk);

    
    xrobot=stateknext[0][0];
    yrobot=stateknext[1][0];
    thetarobot=stateknext[3][0];

    //thetarobot=thetarobot*360/(2*pi);

    //Serial.print(xrobot,10);Serial.print("     ");Serial.print(yrobot,10);Serial.print("     ");Serial.println(thetarobot,10);

    //Serial.println();
    //Serial.println();
  //  if(camera_flag){
   
      String message="";
      message = String(xrobot,8) + " " + String(yrobot,8) + " " + String(thetarobot,8) ;
      //message = String(xrobot,8) + " " + String(yrobot,8) + " " + String(thetarobot,8) +" " + String(666) + " " + String(666) + " " + String(666);
      //message = String(666) + " " + String(666) + " " + String(666) +" " + String(xrobot,8) + " " + String(yrobot,8) + " " + String(thetarobot,8);
      char copy [message.length()];
      message.toCharArray(copy, message.length());
      
      //client.publish("kalman", copy);
      int ret = client.publish("kalman", copy);
      if(ret==0){
        Serial.println(copy);
      }else {
        Serial.println("kalman");
        client.publish("kalman", copy);
      }
      camera_flag = false;
    //}

}

//}

void callback(char* topic, byte* payload, unsigned int length) {
  
  Serial.print("Message arrived in topic: ");
  String input;
  //Serial.println(topic);
  //Serial.print("Message:");
  for (int i = 0; i < length; i++) {
    //Serial.write((char)payload[i]);
    //Serial.print((char)payload[i]);
    input +=(char)payload[i];
  }
  
  //Serial.println();
  Serial.println(input);
  DynamicJsonBuffer jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(input);
  String json;
  xcam = root["x"];
  ycam = root["y"];
  thetacam = root["theta"];
  ///// Walker /////
  guardAngle = root["gA"];
  upperLimit = root["ul"];
  lowerLimit = root["ll"];
  ///// Guard /////
  turnAngle = root["tA"];
  camera_flag = true;
  
  
}

void walkerMoves(){
  if( guardAngle <= upperLimit && guardAngle >= lowerLimit){
    gostraight();
  }else{
    block();
  }
  
}

void guardMoves(){
  if( thetarobot<= turnAngle ){
    turnright();
  }
}


//////////// Moving Functions /////////////
void turnright()
{
     analogWrite(right_wheel, 0);
      analogWrite(left_wheel, 1023); 
}
void turnleft()
{
     analogWrite(right_wheel, 1023);
      analogWrite(left_wheel, 0);   
}
void block()
{
      analogWrite(right_wheel, 0);
      analogWrite(left_wheel, 0);    
}
void gostraight()
{
      analogWrite(right_wheel, 1023);
     analogWrite(left_wheel,  1023);

}

void stayinside()
{ 
    // left edge
    if(xrobot<0.2 && cos(thetarobot)<0.5 && sin(thetarobot)<0)
    {turnleft();}
    if(xrobot<0.2 && cos(thetarobot)<0.5 && sin(thetarobot)>0)
    {turnright();}

    //right edge
    if(xrobot>1.8 && cos(thetarobot)>-0.5 && sin(thetarobot)<0)
    {turnright();}
    if(xrobot>1.8 && cos(thetarobot)>-0.5 && sin(thetarobot)>0)
    {turnleft();}

    //lower edge
    if(yrobot<0.2 && cos(thetarobot)<0 && sin (thetarobot)<0.5)
    {turnright();}
    if(yrobot<0.2 && cos(thetarobot)>0 && sin (thetarobot)<0.5)
    {turnleft();}

    //upper edge
    if(yrobot>2.3 && cos(thetarobot)<0 && sin (thetarobot)>-0.5)
    {turnleft();}
    if(yrobot>2.3 && cos(thetarobot)>0 && sin (thetarobot)>-0.5)
    {turnright();}
}

void circlemove()
{ 
    
    if(incr<=100)
    {
      gostraight();
      //command = "1";
      //Serial.println(command);
      incr++;
    }

       
    if(incr>100 && incr<=150)
    {
      block();
      //command = "2";
     //Serial.println(command);
      incr++;
    }

       
    if(incr>150 && incr<=250)         
    {
      turnleft();
       //     command = "3";
      //Serial.println(command);
       incr++;
    }

        if(incr>250 &&  incr<=450)
    {
      block();
       //     command = "4";
     // Serial.println(command);
       incr++;
    }
        if( incr>450)
    {
         //   command = "5";
     // Serial.println(command);
      incr=0;  
    }
}



void moveAround(){
  // Serial.println("paok");
   if( flag){
    choice= random(1, 4);
    flag = false;
   }
    if(millis() >time_1+INTERVAL_MESSAGE1){
        choice= random(1, 5);
        time_1 = millis();
        Serial.println(choice);
        Serial.println(time_1/1000);
    }
    switch (choice) {
    case 1:
    //checkBorders();
    gostraight();
      break;
    case 2:
    turnright();
      break;
    case 3:
    turnleft;
      break;
    case 4:
    block;
      break;
    default:
      // if nothing else matches, do the default
      // default is optional
      break;
  }
}
