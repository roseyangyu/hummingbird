#ifndef KALMAN_EXAMPLES1_ROBOT_SYSTEMMODEL_HPP_
#define KALMAN_EXAMPLES1_ROBOT_SYSTEMMODEL_HPP_

#include <kalman/LinearizedSystemModel.hpp>
#include "EigenTools.hpp"

namespace KalmanExamples
{
namespace Robot1
{

using namespace Eigen;

/**
 * @brief System state vector-type.
 * 
 * Vector is (X,Y,Z,VX,VY,VZ,ROLL,PITCH,YAW)
 * 
 * X,Y,Z is partner position in base_link frame
 * VX,VY,VZ is base_link velocity in base_link frame
 * ROLL,PITCH,YAW is rotation from partner frame (modelled as an inertial frame) to base_link frame.
 *
 *
 * @param T Numeric scalar type
 */
template<typename T>
class State : public Kalman::Vector<T, 9>
{
public:
    KALMAN_VECTOR(State, T, 9)
};

/**
 * @brief System control-input vector-type for a 3DOF planar robot
 *
 * Input is ax, ay, az, wx, wy, wz
 * 
 * ax,ay,az is acceleration base_link in base_link frame
 * wx,wy,wz is angular rotation in base_link frame
 *
 * @param T Numeric scalar type
 */
template<typename T>
class Control : public Kalman::Vector<T, 6>
{
public:
    KALMAN_VECTOR(Control, T, 6)
};

/**
 * @brief System model for a simple planar 3DOF robot
 *
 * This is the system model defining how our robot moves from one 
 * time-step to the next, i.e. how the system state evolves over time.
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class SystemModel : public Kalman::LinearizedSystemModel<State<T>, Control<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
	typedef KalmanExamples::Robot1::State<T> S;
    
    //! Control type shortcut definition
    typedef KalmanExamples::Robot1::Control<T> C;

    /**
     * @brief Definition of (non-linear) state transition function
     *
     * This function defines how the system state is propagated through time,
     * i.e. it defines in which state \f$\hat{x}_{k+1}\f$ is system is expected to 
     * be in time-step \f$k+1\f$ given the current state \f$x_k\f$ in step \f$k\f$ and
     * the system control input \f$u\f$.
     *
     * @param [in] x The system state in current time-step
     * @param [in] u The control vector input
     * @returns The (predicted) system state in the next time-step
     */
    S f(const S& x, const C& u, float dt) const
    {
        //! Predicted state vector after transition
        S x_dot;
        Vector3f w = u.segment(3,3);
        Matrix3f S = vectorToCrossMatrix(w);
        x_dot.segment(0,3) = -S*x.segment(0,3) - x.segment(3,3);
        Vector3f a = u.segment(0,3);
        x_dot.segment(3,3) = a;
        Vector3f rpy = x.segment(6,3);
        // ROLL_DOT
        x_dot(6) = (w(0)*cos(rpy(1))+w(2)*cos(rpy(0))*sin(rpy(1))+w(1)*sin(rpy(1))*sin(rpy(0)))/cos(rpy(1));
        // PITCH_DOT
        x_dot(7) = w(1)*cos(rpy(0))-w(2)*sin(rpy(0));
        // YAW_DOT
        x_dot(8) = (w(2)*cos(rpy(0))+w(1)*sin(rpy(0)))/cos(rpy(1));
        return x + x_dot*dt;
    }

protected:
    /**
     * @brief Update jacobian matrices for the system state transition function using current state
     *
     * This will re-compute the (state-dependent) elements of the jacobian matrices
     * to linearize the non-linear state transition function \f$f(x,u)\f$ around the
     * current state \f$x\f$.
     *
     * @note This is only needed when implementing a LinearizedSystemModel,
     *       for usage with an ExtendedKalmanFilter or SquareRootExtendedKalmanFilter.
     *       When using a fully non-linear filter such as the UnscentedKalmanFilter
     *       or its square-root form then this is not needed.
     *
     * @param x The current system state around which to linearize
     * @param u The current system control input
     */
    void updateJacobians( const S& x, const C& u, float dt)
    {
        this->F.setZero();
        Vector3f w = u.segment(3,3);
        Matrix3f S = vectorToCrossMatrix(w);
        // parter position wrt partner position
        this->F.block(0,0,3,3) = -1*S;
        // partner position wrt velocity
        this->F.block(0,3,3,3) = -1*MatrixXf::Identity(3, 3);

        Vector3f rpy = x.segment(6,3);
        // roll_dot wrt roll, pitch, yaw
        this->F(6, 6) =  (w(1)*cos(rpy(0))-w(2)*sin(rpy(0)))*sin(rpy(1))/cos(rpy(1));
        this->F(6,7) = (w(2)*cos(rpy(0))+w(1)*sin(rpy(0)))/(cos(rpy(1))*cos(rpy(1)));
        this->F(6,8) = 0;
        // pitch_dot wrt roll, pitch, yaw
        this->F(7,6) = -w(2)*cos(rpy(0))-w(1)*sin(rpy(0));
        this->F(7,7) = 0;
        this->F(7,8) = 0;
        // yaw_dot wrt roll, pitch, yaw
        this->F(8,6) = (w(1)*cos(rpy(0))-w(2)*sin(rpy(0)))/cos(rpy(1));
        this->F(8,7) = (w(2)*cos(rpy(0))+w(1)*sin(rpy(0)))*sin(rpy(1)) / (cos(rpy(1))*cos(rpy(1)));
        this->F(8,8) = 0;
        // df_hat/dx = I + df/dx
        this->F = MatrixXf::Identity(9, 9) + dt*this->F;
        // Model noise
        this->W.setIdentity();
        this->W = dt*(this->W);
    }
};

} // namespace Robot
} // namespace KalmanExamples

#endif