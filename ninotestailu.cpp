#include <iostream>
#include <cmath>

using namespace std;
struct vector3 {
    double x;
    double y;
    double z;
};
struct PIDResult {
    double output;
    double lastError;
    double integral;
    unsigned long previousMillis;
};
// Output is rad/s
PIDResult pidUpdate(double input, double setpoint,
                    double kp, double ki, double kd,
                    PIDResult s){
    unsigned long now = s.previousMillis + 5; 
    double dt = (now - s.previousMillis) / 1000.0;
    if (dt <= 0) dt = 0.001;

    double err = setpoint - input;

    s.integral += err * dt;
    double derivative = (err - s.lastError) / dt;

    s.output = kp * err + ki * s.integral + kd * derivative;

    s.lastError = err;
    s.previousMillis = now;
    return s;
}

string curState = "FLIGHT"; // HEITETTY --> STABILIZE --> FLIGHT --> LAND
double thrust = 0.80; // base thrust

// INIT variables
double measuredRoll  = 0.0;
double measuredPitch = 0.0; 
double measuredYaw   = 0.0; 
vector3 measuredPos  = {0, 0, 0};
vector3 startPos     = {0, 10, 0};
vector3 lastPosition = {0,0,0};

PIDResult rollPID  = {0,0,0,0};
PIDResult pitchPID = {0,0,0,0};
PIDResult yawPID   = {0,0,0,0};
PIDResult xPID = {0,0,0,0};
PIDResult yPID = {0,0,0,0};

int main() {
    auto clamp = [](double a, double min, double max){
        if (a < min) a = min; // EI ikinä moottoreita nollaan!!
        if (a > max) a = max;
        return a;
    };

    // NÄÄ KORVATAAN MITTA DATAL
    double roll  = measuredRoll; // rad, IMU measurement
    double pitch = measuredPitch; 
    double yaw   = measuredYaw;
    vector3 curPosition = measuredPos;
    vector3 targetPosition = startPos;
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

        rollPID  = pidUpdate(roll,  rollSet, 0.05, 0.002, 0.001, rollPID);
        pitchPID = pidUpdate(pitch, pitchSet,0.05, 0.002, 0.001, pitchPID);
        yawPID   = pidUpdate(yaw,   yawSet,  0.05, 0.002, 0.001, yawPID);

        motor1_pwm = clamp(thrust - rollPID.output + pitchPID.output + yawPID.output, 0.1, 1)*2000;
        motor2_pwm = clamp(thrust + rollPID.output - pitchPID.output + yawPID.output, 0.1, 1)*2000;
        motor3_pwm = clamp(thrust + rollPID.output + pitchPID.output - yawPID.output, 0.1, 1)*2000;
        motor4_pwm = clamp(thrust - rollPID.output - pitchPID.output - yawPID.output, 0.1, 1)*2000;

        if ((lastPosition.y - curPosition.y) >= 0.01) { // nousu
            thrust += -0.001; // AINA LASKEUDUTAAN MIKÄÄN TILA EI NOSTA
        } else if ((curPosition.y - lastPosition.y) >= 0.01) { // lasku
            thrust += (curState == "FLIGHT") ? +0.001 : 0; // jos laskeudutaan flight tilassa nosta muissa jatka laskeutumista
        } else{  // leijunta
            thrust += (curState == "FLIGHT") ? +0.001 : 0; // jos leijutaan flight tilassa nosta muissa jatka laskeutumista
        }

        // kun tarpeeks lähellä targettia, vaihda "LAND"
        if (curState == "FLIGHT") {
            double distance = sqrt(pow(targetPosition.x - curPosition.x, 2) +
                                    pow(targetPosition.y - curPosition.y, 2) +);
            if (distance < 0.5) {
                curState = "LAND";
            }
        }

        vector3 lastPosition = curPosition;
        thrust = clamp(thrust, 0.1, 1.0);
    }else{ // Jos STABILIZE tila
        rollPID  = pidUpdate(roll,  rollSet, 0.05, 0.002, 0.001, rollPID);
        pitchPID = pidUpdate(pitch, pitchSet,0.05, 0.002, 0.001, pitchPID);
        yawPID   = pidUpdate(yaw,   yawSet,  0.05, 0.002, 0.001, yawPID);

        motor1_pwm = 1000 + clamp(thrust - rollPID.output + pitchPID.output + yawPID.output, 0.1, 1)*2000;
        motor2_pwm = 1000 + clamp(thrust + rollPID.output - pitchPID.output + yawPID.output, 0.1, 1)*2000;
        motor3_pwm = 1000 + clamp(thrust + rollPID.output + pitchPID.output - yawPID.output, 0.1, 1)*2000;
        motor4_pwm = 1000 + clamp(thrust - rollPID.output - pitchPID.output - yawPID.output, 0.1, 1)*2000;
    }

    
    cout << "| Roll: " << rollPID.output << " ";
    cout << "| Pitch: " << pitchPID.output << " ";
    cout << "| Yaw: " << yawPID.output << endl;
        
    cout << "Motor PWMs: "
        << motor1_pwm << ", "
        << motor2_pwm << ", "
        << motor3_pwm << ", "
        << motor4_pwm << endl;
        cout << "\n";

    return 0;
}
