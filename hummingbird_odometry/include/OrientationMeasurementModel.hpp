#ifndef KALMAN_EXAMPLES_ROBOT1_ORIENTATIONMEASUREMENTMODEL_HPP_
#define KALMAN_EXAMPLES_ROBOT1_ORIENTATIONMEASUREMENTMODEL_HPP_

#include <kalman/LinearizedMeasurementModel.hpp>

namespace KalmanExamples
{
namespace Robot1
{

/**
 * @brief Measurement vector measuring an orientation.
 * Vector order is ROLL, PITCH, YAW from partner frame to base_link frame.
 *
 * @param T Numeric scalar type
 */
template<typename T>
class OrientationMeasurement : public Kalman::Vector<T, 3>
{
public:
    KALMAN_VECTOR(OrientationMeasurement, T, 3)
};

/**
 * @brief Measurement model for measuring orientation of a 3DOF robot
 *
 * This is the measurement model for measuring the orientation of our
 * planar robot. This could be realized by a compass / magnetometer-sensor.
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class OrientationMeasurementModel : public Kalman::LinearizedMeasurementModel<State<T>, OrientationMeasurement<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
    typedef KalmanExamples::Robot1::State<T> S;
    
    //! Measurement type shortcut definition
    typedef  KalmanExamples::Robot1::OrientationMeasurement<T> M;
    
    OrientationMeasurementModel()
    {
        // Output of this measurement is roll, pitch, yaw
        // which are the last 3 elements of x
        this->H.setZero();
        this->H(0, 6) = 1;
        this->H(1, 7) = 1;
        this->H(2, 8) = 1;
        // Set noise
        this->V.setIdentity();
    }
    
    /**
     * @brief Definition of (possibly non-linear) measurement function
     *
     * This function maps the system state to the measurement that is expected
     * to be received from the sensor assuming the system is currently in the
     * estimated state.
     *
     * @param [in] x The system state in current time-step
     * @returns The (predicted) sensor measurement for the system state
     */
    M h(const S& x) const
    {
        M measurement;
        measurement(0) = x(6);
        measurement(1) = x(7);
        measurement(2) = x(8); 
        return measurement;
    }
};

} // namespace Robot
} // namespace KalmanExamples

#endif