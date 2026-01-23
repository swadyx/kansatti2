#include <iostream>
#include <cmath>
#include <fstream>
#include <chrono>
#include <thread>
#include <vector>

using namespace std;

// Structs
struct vector3 {
    double x, y, z;
};
struct quaternion {
    double w, x, y, z;
};
struct PIDResult {
    double output;
    double lastError;
    double integral;
    unsigned long previousMillis;
};
struct State {
    quaternion q;   // orientation
};

struct Covariance {
    double p[3][3];
};
// Utility functions
const double LAT_TO_METERS = 111320.0;
double lonToMeters(double latDeg) { return 111320.0 * cos(latDeg * M_PI/180.0); }
vector3 normalize(vector3 v) {
    double n = sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
    if (n < 1e-9) return {0,0,0};
    return { v.x/n, v.y/n, v.z/n };
}
quaternion normalizeQ(quaternion q) {
    double n = sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if (n < 1e-9) return {1,0,0,0};
    return { q.w/n, q.x/n, q.y/n, q.z/n };
}
quaternion qMul(const quaternion &a, const quaternion &b) {
    return {
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
    };
}
auto clamp = [](double a, double min, double max){
    if (a < min) a = min;
    if (a > max) a = max;
    return a;
};
double alpha = .69;
double sIIR(double newData, double oldData) {
    return oldData+alpha*(newData-oldData);
}; //simppeli IIR : y[n] = y[n-1]+a*(x[n]-y[n-1])

// extended kalman voids
void predict(State &x, vector3 gyro, double dt) {
    quaternion wq = {0, gyro.x, gyro.y, gyro.z};

    quaternion dq = {
        0.5 * (-x.q.x*wq.x - x.q.y*wq.y - x.q.z*wq.z),
        0.5 * ( x.q.w*wq.x + x.q.y*wq.z - x.q.z*wq.y),
        0.5 * ( x.q.w*wq.y - x.q.x*wq.z + x.q.z*wq.x),
        0.5 * ( x.q.w*wq.z + x.q.x*wq.y - x.q.y*wq.x)
    };

    x.q.w += dq.w * dt;
    x.q.x += dq.x * dt;
    x.q.y += dq.y * dt;
    x.q.z += dq.z * dt;

    x.q = normalizeQ(x.q);
}
void predictCov(Covariance &P, double noise, double dt) {
    double q = noise * dt * dt;
    for (int i = 0; i < 3; i++)
        P.p[i][i] += q;
}
vector3 expectedAccel(const quaternion &q) {
    return {
        2*(q.x*q.z - q.w*q.y),
        2*(q.w*q.x + q.y*q.z),
        q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z
    };
}
void jacobianH(const State &x, double H[3][3]) {
    H[0][0] = 1; H[0][1] = 0; H[0][2] = 0;
    H[1][0] = 0; H[1][1] = 1; H[1][2] = 0;
    H[2][0] = 0; H[2][1] = 0; H[2][2] = 1;
}
// PID update function
PIDResult pidUpdate(double input, double setpoint,
                    double kp, double ki, double kd,
                    PIDResult s){
    //unsigned long now = millis(); 
    double dt = 0.02; // fixed dt for simulation
    
    if (input < 1e-5) input = 0.001;

    double err = setpoint - input;

    s.integral += err * dt;
    s.integral = clamp(s.integral, -0.3, 0.3); // Anti-windup
    double derivative = (err - s.lastError) / dt;

    s.output = kp * err + ki * s.integral + kd * derivative;

    s.lastError = err;
    //s.previousMillis = now;
    return s;
}

// Mitta data
vector3 gyroData  = {0,0,0}; // rad/s
vector3 accelData = {0,0,0}; // m/s^2
//vector3 magData   = {0,0,0};
vector3 gpsData   = {0,0,0}; // deg, deg, m (above sea)

// INIT variable 
vector3 calculatedPosition = {0, 0, 0};
vector3 measuredPos        = {0, 0, 0};
vector3 lastPosition       = {0, 0, 0};
vector3 startPos     = {0, 0, 0};
vector3 targetPosition     = {0, 0 ,0};
PIDResult rollPID  = {0,0,0,0};
PIDResult pitchPID = {0,0,0,0};
PIDResult yawPID   = {0,0,0,0};
PIDResult xPID = {0,0,0,0};
PIDResult yPID = {0,0,0,0};
double descentFallVelocity = 0.0; // Kuinka nopeesti halutaan pudota
string curState = "FLIGHT"; // STABILIZE --> FLIGHT --> LAND
double thrust = 0.45; // base thrust

void flightcontrollerLoop(double measuredRoll, double measuredPitch, double measuredYaw) {
    double roll  = measuredRoll; // rad, IMU measurement
    double pitch = measuredPitch; 
    double yaw   = measuredYaw;

    vector3 measuredPos = measuredPos;
    vector3 targetPosition = startPos;
    vector3 currentVelocity = {0,0,0};
    // -------------------------------------------

    double motor1_pwm, motor2_pwm, motor3_pwm, motor4_pwm;

    if (curState != "STABILIZE")
    {
        // claculate PID for position control
        xPID = pidUpdate(measuredPos.x, targetPosition.x, 0.2, 0.0, 0.01, xPID);
        yPID = pidUpdate(measuredPos.y, targetPosition.y, 0.2, 0.0, 0.01, yPID);

        double rollSet  = xPID.output;     // oikee / vase
        double pitchSet = -yPID.output;    // loput
        double yawSet   = 0.0; // älä pyöri

        // Clamp ettei vedä flipoi
        rollSet  = clamp(rollSet, -0.5, 0.5);
        pitchSet = clamp(pitchSet, -0.5, 0.5);

        // Päivitä PIDit
        rollPID  = pidUpdate(roll,  rollSet, 0.05, 0.002, 0.001, rollPID);
        pitchPID = pidUpdate(pitch, pitchSet,0.05, 0.002, 0.001, pitchPID);
        yawPID   = pidUpdate(yaw,   yawSet,  0.05, 0.002, 0.001, yawPID);

        // Clamp PID outputit
        rollPID.output = clamp(rollPID.output, -0.3, 0.3);
        pitchPID.output = clamp(pitchPID.output, -0.3, 0.3);
        yawPID.output = clamp(yawPID.output, -0.2, 0.2);

        // ------------------------------------------- VAIHA TÄHÄN VAIK ALTIDUTA PID ------------------------------------------
        // Jokaisessa tilassa pitää laskeutua
        if(abs(lastPosition.y - measuredPos.y) >= 0.01) {
            if ((lastPosition.y - measuredPos.y) >= 0) { // Jos me nousaan ylöspäin ( ei pitäis olla mahdollista )
                thrust += (measuredPos.y < (targetPosition.y + 15)) ? (curState=="FLIGHT") ? 0.001 : -0.001 : -0.001; // jos ollaan liian alhaalla ja lento tilassa nosta muulloin laske
            }else{ // Pudotaan
                if (currentVelocity.y > descentFallVelocity) { // Jos pudotaan hitaammin kuin halutaan pudotaan nopeemmin
                    thrust += (measuredPos.y < (targetPosition.y + 15)) ? -0.001 : (curState=="FLIGHT") ? 0.001 : -0.001; // Jos ollaan liian alhaalla nosta ellei oo lakseutumis tila
                }else{
                    if (descentFallVelocity  > currentVelocity.y) { // Nyt pudotaan jo liian kovaa, hidastus
                        thrust += 0.001;
                    }
                }
            }
        }  // ------------------------------------------- VAIHA TÄHÄN VAIK ALTIDUTA PID ------------------------------------------


        // kun tarpeeks lähellä targettia, vaihda "LAND" ei oteta korkeutta huomioon koska voidaan joko olla helveti korkeel tai just ja just siin raja korkeudes

        lastPosition = measuredPos; // Vaihetaan arvot ens looppia varten
        thrust = clamp(thrust, 0 , .5); // Ei thrusti yli- tai aliammu
    }else{ // Jos STABILIZE tila eli ei vielä liikuta halutaan vaa suoraa raketista ittensä pysty suoraks
        rollPID  = pidUpdate(roll,  0, 0.05, 0.002, 0.001, rollPID);
        pitchPID = pidUpdate(pitch, 0, 0.05, 0.002, 0.001, pitchPID);
        yawPID   = pidUpdate(yaw,   0, 0.05, 0.002, 0.001, yawPID);
    }

    // laksetaa PID
    double m1 = thrust - rollPID.output + pitchPID.output + yawPID.output;
    double m2 = thrust + rollPID.output - pitchPID.output + yawPID.output;
    double m3 = thrust + rollPID.output + pitchPID.output - yawPID.output;
    double m4 = thrust - rollPID.output - pitchPID.output - yawPID.output;

    // Clamp ja otetaa tarpeeks isot
    motor1_pwm = clamp(m1 * 1000 + 1000, 1000, 2000);
    motor2_pwm = clamp(m2 * 1000 + 1000, 1000, 2000);
    motor3_pwm = clamp(m3 * 1000 + 1000, 1000, 2000);
    motor4_pwm = clamp(m4 * 1000 + 1000, 1000, 2000);

    // printtaa output  
    system("cls");
        
    cout << "Motor PWMs: "
        << motor1_pwm << ", "
        << motor2_pwm << ", "
        << motor3_pwm << ", "
        << motor4_pwm << endl;
        cout << "\n";
}

int main() {
    // testi tiedosto
    ifstream inFile("C:/Users/Ninon/AppData/LocalLow/DefaultCompany/gyro/sensor_log.txt");
    if (!inFile) {
        cout << "Unable to open file\n";
        return 1;
    }
    vector3 lastGPS = {0,0,0};
    double previousYaw = 0;
    double currentYaw = 0;
    // EKF init
    State x = { {1,0,0,0} };

    Covariance P = {{
        {3,0,0},
        {0,3,0},
        {0,0,3}
    }};
    double accelNoise = 0.01;
    double gyroNoise  = 0.02;

    auto lastTime = chrono::high_resolution_clock::now();

    while (true) {
        
        inFile.clear();
        inFile.seekg(0, ios::beg);

        // unity -> file -> here
        vector<double> s;
        double v;
        while (inFile >> v) s.push_back(v);
        if (s.size() < 9) continue;

        // aseta data - pass trhough simple onepoint iir
        vector3 n_gyroData  =  { sIIR(s[0],gyroData.x)
                    ,    sIIR(s[1],gyroData.y)
                    ,    sIIR(s[2],gyroData.z) };

        vector3 n_accelData  = { sIIR(s[3],accelData.x)
                    ,    sIIR(s[4],accelData.y)
                    ,    sIIR(s[5],accelData.z) };

        vector3 n_gpsData  =   { sIIR(s[9],gpsData.x)
                    ,    sIIR(s[10],gpsData.y)
                    ,    sIIR(s[11],gpsData.z) };

        /*/ vector3 n_magData  =   { sIIR(s[6],magData.x)
                    ,    sIIR(s[7],magData.y)
                    ,    sIIR(s[8],magData.z) }; /*/
                    
        // Set data to new filtered data
        gyroData = n_gyroData;
        accelData = normalize(n_accelData);
        gpsData = n_gpsData;
        //magData = n_magData;

        auto now = chrono::high_resolution_clock::now();
        double dt = chrono::duration<double>(now - lastTime).count();

        lastTime = now;

        predict(x, gyroData, dt);
        predictCov(P, gyroNoise, dt);

        vector3 h = expectedAccel(x.q);
        vector3 y = { accelData.x - h.x,
                    accelData.y - h.y,
                    accelData.z - h.z };

        double H[3][3];
        jacobianH(x, H);

        double K[3] = {
            P.p[0][0] / (P.p[0][0] + accelNoise),
            P.p[1][1] / (P.p[1][1] + accelNoise),
            P.p[2][2] / (P.p[2][2] + accelNoise)
        };

        vector3 dtheta = { K[0]*y.x, K[1]*y.y, K[2]*y.z };
        double theta = sqrt(dtheta.x*dtheta.x + dtheta.y*dtheta.y + dtheta.z*dtheta.z);

        quaternion dq;
        if(theta < 1e-6) {
            dq = {1.0, 0.5*dtheta.x, 0.5*dtheta.y, 0.5*dtheta.z};
        } else {
            double halfTheta = theta * 0.5;
            double sinHalf = sin(halfTheta) / theta;
            dq.w = cos(halfTheta);
            dq.x = dtheta.x * sinHalf;
            dq.y = dtheta.y * sinHalf;
            dq.z = dtheta.z * sinHalf;
        }

        double gpsYaw;
        if (!isnan(gpsData.x) && !isnan(gpsData.y)) {
            double dx = (gpsData.x - lastGPS.x) * LAT_TO_METERS;
            double dy = (gpsData.y - lastGPS.y) * lonToMeters(gpsData.x);
            lastGPS = gpsData;

            if (dx != 0 || dy != 0)
                gpsYaw = atan2(dy, dx);
            else
                gpsYaw = previousYaw; // no movement
        } else {
            gpsYaw = previousYaw; // jos NaN ei oo gps
        }
        double alphaYaw = 0.1;
        currentYaw = currentYaw + alphaYaw * (gpsYaw - currentYaw);

        double yawError = gpsYaw - currentYaw;
        quaternion dq_yaw = { cos(0.5*yawError), 0, 0, sin(0.5*yawError) };
        x.q = qMul(x.q, dq_yaw);
        x.q = normalizeQ(x.q);

        if (x.q.w < 0) { x.q.w=-x.q.w; x.q.x=-x.q.x; x.q.y=-x.q.y; x.q.z=-x.q.z; } // untiy quaternion flipperi

        for (int i=0; i<3; i++) P.p[i][i] *= (1.0 - K[i]);
        std::string unityFilePath = "C:/Users/Ninon/AppData/LocalLow/DefaultCompany/gyro/orientation_log.txt";

        {
            std::ofstream ofs(unityFilePath, std::ios::trunc);
            if (ofs.is_open()) {
                ofs << x.q.w << " "
                    << x.q.x << " "
                    << x.q.y << " "
                    << x.q.z << "\n";
                ofs.close();
            } else {
                std::cerr << "unity onkelma" << std::endl;
            }
        }

        double roll  = atan2( 2*(x.q.w*x.q.x + x.q.y*x.q.z),
                            1 - 2*(x.q.x*x.q.x + x.q.y*x.q.y) );
        double pitch = asin( 2*(x.q.w*x.q.y - x.q.z*x.q.x) );
        double yaw   = atan2( 2*(x.q.w*x.q.z + x.q.x*x.q.y),
                            1 - 2*(x.q.y*x.q.y + x.q.z*x.q.z) );
        flightcontrollerLoop(roll,pitch,yaw);
    }
    return 0;
}
