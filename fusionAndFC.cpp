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

// Utility functions
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

// Parameters
double beta = 0.13;
double descentFallVelocity = 0.0; // Kuinka nopeesti halutaan pudota

// Mitta data
vector3 gyroData = {0,0,0};   // rad/s
vector3 accelData = {0,0,0};  // m/s^2
vector3 magData = {0,0,0};

// INIT variables ja variablet joita ei saa muokata ennen
string curState = "FLIGHT"; // STABILIZE --> FLIGHT --> LAND
double thrust = 0.45; // base thrust
vector3 measuredPos  = {0, 0, 0};
vector3 startPos     = {0, 0, 0};
vector3 lastPosition = {0,0,0};

quaternion fusedOrientation = {1,0,0,0};
PIDResult rollPID  = {0,0,0,0};
PIDResult pitchPID = {0,0,0,0};
PIDResult yawPID   = {0,0,0,0};
PIDResult xPID = {0,0,0,0};
PIDResult yPID = {0,0,0,0};

void flightcontrollerLoop(double measuredRoll, double measuredPitch, double measuredYaw) {
    double roll  = measuredRoll; // rad, IMU measurement
    double pitch = measuredPitch; 
    double yaw   = measuredYaw;

    vector3 curPosition = measuredPos;
    vector3 targetPosition = startPos;
    vector3 currentVelocity = {0,0,0};
    // -------------------------------------------

    double motor1_pwm, motor2_pwm, motor3_pwm, motor4_pwm;

    if (curState != "STABILIZE")
    {
        // claculate PID for position control
        xPID = pidUpdate(curPosition.x, targetPosition.x, 0.2, 0.0, 0.01, xPID);
        yPID = pidUpdate(curPosition.y, targetPosition.y, 0.2, 0.0, 0.01, yPID);

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
        if(abs(lastPosition.y - curPosition.y) >= 0.01) {
            if ((lastPosition.y - curPosition.y) >= 0) { // Jos me nousaan ylöspäin ( ei pitäis olla mahdollista )
                thrust += (curPosition.y < (targetPosition.y + 15)) ? (curState=="FLIGHT") ? 0.001 : -0.001 : -0.001; // jos ollaan liian alhaalla ja lento tilassa nosta muulloin laske
            }else{ // Pudotaan
                if (currentVelocity.y > descentFallVelocity) { // Jos pudotaan hitaammin kuin halutaan pudotaan nopeemmin
                    thrust += (curPosition.y < (targetPosition.y + 15)) ? -0.001 : (curState=="FLIGHT") ? 0.001 : -0.001; // Jos ollaan liian alhaalla nosta ellei oo lakseutumis tila
                }else{
                    if (descentFallVelocity  > currentVelocity.y) { // Nyt pudotaan jo liian kovaa, hidastus
                        thrust += 0.001;
                    }
                }
            }
        }  // ------------------------------------------- VAIHA TÄHÄN VAIK ALTIDUTA PID ------------------------------------------


        // kun tarpeeks lähellä targettia, vaihda "LAND" ei oteta korkeutta huomioon koska voidaan joko olla helveti korkeel tai just ja just siin raja korkeudes

        lastPosition = curPosition; // Vaihetaan arvot ens looppia varten
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
    // ei oo millisii pakko leikkii
    auto nextTick = chrono::steady_clock::now();

    // ikune looppi
    while (true) {
        double dt = 0.02; // Deltatime on 0.02 simulaatiota varte

        inFile.clear();
        inFile.seekg(0, ios::beg);

        // nappaa tiedostosta datat jotka unity tuottaa
        vector<double> s;
        double v;
        while (inFile >> v) s.push_back(v);
        if (s.size() < 9) continue;

        // aseta data
        gyroData  = { s[0], s[1], s[2] };
        accelData = { s[3], s[4], s[5] };
        magData   = { s[6], s[7], s[8] };

        quaternion q = fusedOrientation;

        // Normalize magnetometer
        vector3 mNorm = normalize(magData);
        // Normalize accelerometer
        vector3 aNorm = normalize(accelData);

        // lyhennä muuttujat
        double ax = aNorm.x;
        double ay = aNorm.y;
        double az = aNorm.z;
        double mx = mNorm.x;
        double my = mNorm.y;
        double mz = mNorm.z;
        double q0 = q.w;
        double q1 = q.x;
        double q2 = q.y;
        double q3 = q.z;
        
        // SUORAAN GPT mut kyl luin läpi ja ymmärsin jotai
        // Rotate magnetometer into earth frame
        double hx = 2*mx*(0.5 - q2*q2 - q3*q3)
                + 2*my*(q1*q2 - q0*q3)
                + 2*mz*(q1*q3 + q0*q2);

        double hy = 2*mx*(q1*q2 + q0*q3)
                + 2*my*(0.5 - q1*q1 - q3*q3)
                + 2*mz*(q2*q3 - q0*q1);

        double hz = 2*mx*(q1*q3 - q0*q2)
                + 2*my*(q2*q3 + q0*q1)
                + 2*mz*(0.5 - q1*q1 - q2*q2);

        // Reference direction of Earth's field
        double bx = sqrt(hx*hx + hy*hy);
        double bz = hz;

        double f4 = 2*bx*(0.5 - q2*q2 - q3*q3)
                + 2*bz*(q1*q3 - q0*q2) - mx;

        double f5 = 2*bx*(q1*q2 - q0*q3)
                + 2*bz*(q0*q1 + q2*q3) - my;

        double f6 = 2*bx*(q0*q2 + q1*q3)
                + 2*bz*(0.5 - q1*q1 - q2*q2) - mz;
        // TÄS LOPPUU GPT

        //Gyrö integrointi
        quaternion qDot = qMul(q, {0, gyroData.x, gyroData.y, gyroData.z});
        qDot.w *= 0.5;
        qDot.x *= 0.5;
        qDot.y *= 0.5;
        qDot.z *= 0.5;

        // Accelerometer validity gate aka älä ota iha vittu maisii heilahuskii
        double accelMag = sqrt(
            accelData.x*accelData.x +
            accelData.y*accelData.y +
            accelData.z*accelData.z
        );


        if (fabs(accelMag - 9.81) < 1.5) {
            
            quaternion grad = {0,0,0,0};

            // Joku gradient decent
            double f1 = 2*(q.x*q.z - q.w*q.y) - ax;
            double f2 = 2*(q.w*q.x + q.y*q.z) - ay;
            double f3 = 2*(0.5 - q.x*q.x - q.y*q.y) - az;

            // Gradient
            grad.w =
                (-2*q2)*f1 + ( 2*q1)*f2
            + (-2*bz*q2)*f4 + (-2*bx*q3 + 2*bz*q1)*f5 + (2*bx*q2)*f6;

            grad.x =
                ( 2*q3)*f1 + ( 2*q0)*f2 - (4*q1)*f3
            + (2*bz*q3)*f4 + (2*bx*q2 + 2*bz*q0)*f5
            + (2*bx*q3 - 4*bz*q1)*f6;

            grad.y =
                (-2*q0)*f1 + ( 2*q3)*f2 - (4*q2)*f3
            + (-4*bx*q2 - 2*bz*q0)*f4
            + (2*bx*q1 + 2*bz*q3)*f5
            + (2*bx*q0 - 4*bz*q2)*f6;

            grad.z =
                ( 2*q1)*f1 + ( 2*q2)*f2
            + (-4*bx*q3 + 2*bz*q1)*f4
            + (-2*bx*q0 + 2*bz*q2)*f5
            + (2*bx*q1)*f6;


            // normalisoi gradient
            double gn = sqrt(
                grad.w*grad.w +
                grad.x*grad.x +
                grad.y*grad.y +
                grad.z*grad.z
            );
            if (gn > 1e-5) {
                grad.w /= gn;
                grad.x /= gn;
                grad.y /= gn;
                grad.z /= gn;
            }
        }
        // qDot korjaus
        qDot.w -= beta * grad.w;
        qDot.x -= beta * grad.x;
        qDot.y -= beta * grad.y;
        qDot.z -= beta * grad.z;

        // q päivitys
        q.w += qDot.w * dt;
        q.x += qDot.x * dt;
        q.y += qDot.y * dt;
        q.z += qDot.z * dt;

        fusedOrientation = normalizeQ(q);

        // Eulerit
        double roll  = atan2(2*(q.w*q.x + q.y*q.z),
                            1 - 2*(q.x*q.x + q.y*q.y));
        double pitch = asin(clamp(2*(q.w*q.y - q.z*q.x), -1.0, 1.0)); // Jos menee ohi [-1,1] nii NaN
        double yaw   = atan2(2*(q.w*q.z + q.x*q.y),
                            1 - 2*(q.y*q.y + q.z*q.z));

        nextTick += chrono::milliseconds(20);
        // Laske flight controller
        flightcontrollerLoop(roll, pitch, yaw);
        this_thread::sleep_until(nextTick);
    }
    return 0;
}
