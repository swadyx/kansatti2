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

int main() {
    // NÄÄ KORVATAAN MITTA DATAL
    double roll  = 0.2; // rad, IMU measurement
    double pitch = -0.1; 
    double yaw   = 0.0;
    vector3 curPosition = {0, 0, 0};
    vector3 targetPosition = {0, 10, 0};
    // __________________________________________________

    double thrust = 0.80; // base thrust
    double motor1_pwm, motor2_pwm, motor3_pwm, motor4_pwm;

    PIDResult rollPID  = {0,0,0,0};
    PIDResult pitchPID = {0,0,0,0};
    PIDResult yawPID   = {0,0,0,0};
    PIDResult xPID = {0,0,0,0};
    PIDResult yPID = {0,0,0,0};

    // claculate PID for position control
    xPID = pidUpdate(curPosition.x, targetPosition.x, 0.2, 0.0, 0.01, xPID);
    yPID = pidUpdate(curPosition.y, targetPosition.y, 0.2, 0.0, 0.01, yPID);

    double rollSet  = xPID.output;     // oikee / vase
    double pitchSet = -yPID.output;    // loput
    double yawSet   = 0.0; // älä pyöri

    // Clamp ettei vedä flipoi
    auto clampAngle = [](double a){
        if (a > 0.5) a = 0.5;
        if (a < -0.5) a = -0.5;
        return a;
    };

    rollSet  = clampAngle(rollSet);
    pitchSet = clampAngle(pitchSet);

    for (int i = 0; i < 100; i++) {
        rollPID  = pidUpdate(roll,  rollSet, 0.05, 0.002, 0.001, rollPID);
        pitchPID = pidUpdate(pitch, pitchSet,0.05, 0.002, 0.001, pitchPID);
        yawPID   = pidUpdate(yaw,   yawSet,  0.05, 0.002, 0.001, yawPID);

        cout << "Step " << i << ": ";
        cout << "| Roll: " << rollPID.output << " ";
        cout << "| Pitch: " << pitchPID.output << " ";
        cout << "| Yaw: " << yawPID.output << endl;

        // Simulate drone movement
        double ax = sin(roll)  * thrust * 2.0;
        double ay = sin(pitch) * thrust * 2.0;

        curPosition.x += ax * 0.05;
        curPosition.y += ay * 0.05;

        cout << "Position: (" << curPosition.x << ", " << curPosition.y << ")" << endl;


        auto clamp = [](double v){
            if (v < 0.1) v = 0.1; // EI ikinä moottoreita nollaan!!
            if (v > 1) v = 1;
            return v;
        };

        motor1_pwm = clamp(thrust - rollPID.output + pitchPID.output + yawPID.output)*2000;
        motor2_pwm = clamp(thrust + rollPID.output - pitchPID.output + yawPID.output)*2000;
        motor3_pwm = clamp(thrust + rollPID.output + pitchPID.output - yawPID.output)*2000;
        motor4_pwm = clamp(thrust - rollPID.output - pitchPID.output - yawPID.output)*2000;

        cout << "Motor PWMs: "
            << motor1_pwm << ", "
            << motor2_pwm << ", "
            << motor3_pwm << ", "
            << motor4_pwm << endl;
            cout << "\n";
    }

    return 0;
}
