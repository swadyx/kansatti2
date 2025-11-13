#include <iostream>
#include <cmath>
using namespace std;

// Structs
struct vector3 {
  double x;
  double y;
  double z;
};
struct quaternion{
  double w;
  double x;
  double y;
  double z;
};
struct PIDResult {
    double output;
    double lastError;
    unsigned long previousMillis;
};

// Helpers
vector3 normalize(vector3 p){
    float length = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);

    if (length < 1e-6) // EI MAJUROIDA
        return {0.0f, 0.0f, 0.0f};

    vector3 result;
    result.x = p.x / length;
    result.y = p.y / length;
    result.z = p.z / length;

    return result;
}
quaternion qConjugate(const quaternion &q) {
    quaternion qc;
    qc.w = q.w;     // scalar stays the same
    qc.x = -q.x;    // flip vector part
    qc.y = -q.y;
    qc.z = -q.z;
    return qc;
}
quaternion qMul(const quaternion &a, const quaternion &b) {
    return {
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
    };
}
vector3 cross(vector3 a, vector3 b) {
    vector3 result;
    result.x = a.y * b.z - a.z * b.y;
    result.y = a.z * b.x - a.x * b.z;
    result.z = a.x * b.y - a.y * b.x;
    return result;
}
vector3 rotateVectorByQuaternion(const vector3 &v, const quaternion &q) { //
    quaternion qv = {0, v.x, v.y, v.z};
    quaternion q_conj = qConjugate(q);
    quaternion rotated = qMul(qMul(q, qv), q_conj);

    vector3 result = { rotated.x, rotated.y, rotated.z };
    return result;
}
double dot(const vector3 &a, const vector3 &b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}



// input = measured value , Setpoint = target calculated value
PIDResult calculatePID(
  double input, double p, double i,  
  double d, double setPoint, double lastError,
  double integral, unsigned long& previousMillis){

    unsigned long curMills = 15;
    double sinceLast = double(curMills-previousMillis) / 1000; // Time since last loop in seconds

    double curError = setPoint-input;
    integral += curError * sinceLast; // Integral of error * time
    double derivative = (curError - lastError) / sinceLast;

    double output = p * curError + i * integral + d * derivative; // Calculate output
    previousMillis = curMills;

    // return values
    PIDResult result;
    result.output = output;
    result.lastError = curError;
    result.previousMillis = previousMillis;
    return result;
}

quaternion radToQ(double roll, double, pitch, double yaw){
  // roll, sin_roll , cos_roll
  // pitch, sin_pitch, cos_pitch 
  // yaw, sin_yaw, cos_yaw 
  double rotMatrix[9] = { 
    0, 0, 0,
    0, 0, 0,
    0, 0, 0
  };

  rotMatrix[0] = roll; // Roll - X
  rotMatrix[3] = pitch; // Pitch - Y
  rotMatrix[6] = yaw; // Yaw - Z

  rotMatrix[1] = sin(rotMatrix[0] /2); // sin_roll
  rotMatrix[2] = cos(rotMatrix[0] /2); // cos_roll

  rotMatrix[4] = sin(rotMatrix[3] /2); // sin_pitch 
  rotMatrix[5] = cos(rotMatrix[3] /2); // cos_pitch 

  rotMatrix[7] = sin(rotMatrix[6] /2); // sin_yaw 
  rotMatrix[8] = cos(rotMatrix[6] /2); // cos_yaw

  double w,x,y,z; // new calculated quaternion
  quaternion q;

  q.w = rotMatrix[2]*rotMatrix[5]*rotMatrix[8] + rotMatrix[1]*rotMatrix[4]*rotMatrix[7];
  q.x = rotMatrix[1]*rotMatrix[5]*rotMatrix[8] - rotMatrix[2]*rotMatrix[4]*rotMatrix[7];
  q.y = rotMatrix[2]*rotMatrix[4]*rotMatrix[8] + rotMatrix[1]*rotMatrix[5]*rotMatrix[7];
  q.z = rotMatrix[2]*rotMatrix[5]*rotMatrix[7] - rotMatrix[1]*rotMatrix[4]*rotMatrix[8];

  return q;
}

quaternion calculatePIDTargetAngle(vector3 position, vector3 targetPosition, quaternion rotation){
    vector3 toTarget = {
        targetPosition.x - position.x,
        targetPosition.y - position.y,
        targetPosition.z - position.z
    };
    vector3 tWorld = normalize(toTarget);

    vector3 fBody; // kehon "eteenpäin meno suunta" yksikkövektorina
    fBody.x = 0;
    fBody.y = 0;
    fBody.z = 1.0;

    vector3 fWorld = rotateVectorByQuaternion(fBody, rotation); // worldspace suunta dronen "keholla"

    vector3 rotationAxis = cross(fWorld, tWorld);
    double d = dot(fWorld, tWorld);

    //NaN-proof metodi
    if (d > 1.0) d = 1.0;
    if (d < -1.0) d = -1.0;

    double s = sqrt((1 + d) * 2);

    quaternion r;
    if (s < 1e-6) { // vectors are opposite or almost opposite
        // eri rotation axisien valinta riippuen vectoreiden 
        rotationAxis = {1, 0, 0};
        if (fabs(fWorld.x) > 0.9) rotationAxis = {0, 1, 0};
        r.w = 0;
        r.x = rotationAxis.x;
        r.y = rotationAxis.y;
        r.z = rotationAxis.z;
    } else {
        r.w = 0.5 * s;
        r.x = rotationAxis.x / s;
        r.y = rotationAxis.y / s;
        r.z = rotationAxis.z / s;
    }

    // quaternioonin normalisointi
    double mag = sqrt(r.w*r.w + r.x*r.x + r.y*r.y + r.z*r.z);
    r.w /= mag;
    r.x /= mag;
    r.y /= mag;
    r.z /= mag;

    quaternion targetQ = qMul(r, rotation);
    return targetQ;
}


int main() {
  quaternion q;
  quaternion rot;
  vector3 pos;
  vector3 tPos;
  pos.x = 20;
  pos.y = 20;
  pos.z = 0;
  tPos.x = 10;
  tPos.y = 10;
  tPos.z = 0;
  rot = radToQ(0,0,1,570);

  q = calculatePIDTargetAngle(pos, tPos, rot);
  cout << q.w << " "<< q.x << " "<< q.y << " "<< q.z << " ";
  return 0;

} 
