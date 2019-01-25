#include <Eigen/Dense>

using namespace Eigen;

Matrix3f vectorToCrossMatrix(Vector3f a)
{
    Matrix3f S;
    S.setZero();
    S(0,1) = -1*a(2);
    S(0,2) = a(1);
    S(1,0) = a(2);
    S(1,2) = -1*a(0);
    S(2,0) = -1*a(1);
    S(2,1) = a(0);
    return S;
}

Quaternionf eulerToQuaternion(float roll, float pitch, float yaw)
{
    Quaternionf q;
    q = AngleAxisf(yaw, Vector3f::UnitZ()) 
         * AngleAxisf(pitch, Vector3f::UnitY())
         * AngleAxisf(roll, Vector3f::UnitX());
    return q;
}

Matrix3f eulerToRotationMatrix(float roll, float pitch, float yaw)
{
    Matrix3f m;
    m = AngleAxisf(yaw, Vector3f::UnitZ())
        *AngleAxisf(pitch, Vector3f::UnitY())
        *AngleAxisf(roll, Vector3f::UnitX());
    return m;
}

Vector3f quaterniontoEulerAngle(const Quaternionf q)
{
    float roll, pitch, yaw;
	// roll (x-axis rotation)
	double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
	double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
	roll = atan2(sinr_cosp, cosr_cosp);

	// pitch (y-axis rotation)
	double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
	if (fabs(sinp) >= 1)
		pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		pitch = asin(sinp);

	// yaw (z-axis rotation)
	double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
	double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());  
	yaw = atan2(siny_cosp, cosy_cosp);
    return Vector3f(roll, pitch, yaw);
}