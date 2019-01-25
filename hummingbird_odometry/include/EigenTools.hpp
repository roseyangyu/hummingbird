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

Matrix3f eulerToRotationMatrix(float roll, float pitch, float yaw)
{
    Matrix3f m;
    m = AngleAxisf(yaw, Vector3f::UnitZ())
        *AngleAxisf(pitch, Vector3f::UnitY())
        *AngleAxisf(roll, Vector3f::UnitX());
    return m;
}