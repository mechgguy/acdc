// Copyright (c) 2022 Institute for Automotive Engineering of RWTH Aachen University

// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.

/*
 * trajectory_planner.cpp
 *
 * ACDC (2022)
 *
 * Author:
 * Guido Küppers, guido.kueppers@ika.rwth-aachen.de
 *
 * Description:
 * This file contains the code to be viewed and modified by the students.
 *
 */

#include "trajectory_planner.hpp"
#include <Eigen/Geometry>

// global variables - added here
std::vector<double> vehicle_position;
std::vector<geometry_msgs::Point> stop_line(2);
double stop_distance;
double width;

// std::vector<PLANNER::TrafficLight> tr_i1;
PLANNER::TrafficLight tr_i1;

// Set control constraints
void PLANNER::getControlConstraints(Eigen::VectorXd &u_lb, Eigen::VectorXd &u_ub, Eigen::VectorXi &sparsity)
{
    sparsity << 1, 1;
    u_lb.resize(2);
    u_ub.resize(2);
    u_lb(0) = -5.0;
    u_ub(0) = 5.0;
    u_lb(1) = -0.4;
    u_ub(1) = 0.4;
}

// Set state constraints
void PLANNER::getStateConstraints(Eigen::VectorXd &x_lb, Eigen::VectorXd &x_ub, Eigen::VectorXi &sparsity)
{
    sparsity << 0, 0, 1, 1, 1, 0, 1;
    x_lb.resize(4);
    x_ub.resize(4);
    x_lb(0) = 0.0;
    x_ub(0) = 999999999;
    x_lb(1) = 0.0;
    x_ub(1) = 999999999;
    x_lb(2) = -3.5;
    x_ub(2) = 3.5;
    x_lb(3) = -0.4956735;
    x_ub(3) = 0.4956735;
}

/*
 *
 * Implements the intermediate cost functions.
 * Derivatives are automatically computed using Algorithmic Differentiation.
 *
 * x[0]-x[6] is the state vector (active variables), the others are passive parameters
 * (weights, reference values, dyn obj coordinates and reference path values [x-y-v-x-y-v-...])
 *
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, class MPC_NODE, typename SCALAR_EVAL, typename SCALAR>
template <typename SC>
SC CostTermIntermediate<STATE_DIM, CONTROL_DIM, MPC_NODE, SCALAR_EVAL, SCALAR>::evalLocal(const Eigen::Matrix<SC, STATE_DIM, 1> &x, const Eigen::Matrix<SC, CONTROL_DIM, 1> &u, const SC &t)
{
    // Find nearest sample in reference path
    const size_t index = MPC_NODE::state_dim + MPC_NODE::params_dim_cfg;
    SC distance = CppAD::sqrt(CppAD::pow(x[index] - x[0], 2) + CppAD::pow(x[index + 1] - x[1], 2));
    SC velocity = x[index + 2];
    for (size_t i = index + 3; i < STATE_DIM - 2; i += 3)
    {
        SC currDist = CppAD::sqrt(CppAD::pow(x[i] - x[0], 2) + CppAD::pow(x[i + 1] - x[1], 2));
        velocity = CppAD::CondExpLt(currDist, distance, x[i + 2], velocity);
        distance = CppAD::CondExpLt(currDist, distance, currDist, distance);
    }

    // Ref path term
    SC pathRef = x[MPC_NODE::WEIGHTS::PATH_REF];
    SC pathCost = distance / pathRef;
    SC pathWeight = x[MPC_NODE::WEIGHTS::PATH];
    SC pathTerm = CppAD::pow(pathCost * pathWeight, 2);

    // START TASK 2 CODE HERE
    // use helping comments from Wiki and README.md

    //System State Vector:
    // x[0]: x -> Position X
    // x[1]: y -> Position Y
    // x[2]: s -> Distance
    // x[3]: v -> Vehicle Velocity
    // x[4]: a -> Vehicle Acceleration
    // x[5]: psi -> Vehicle Heading
    // x[6]: delta -> Steering Angle

    //Control Vector:
    // u[0]: j_lon -> longitudinal jerk
    // u[1]: alpha -> Steering Rate

    // if necessary use CppAD::sin(...), CppAD::cos(...), CppAD::tan(...), CppAD::pow(...), CppAD::sqrt(...)
    // Longitudinal jerk term
    SC jerkRef = x[MPC_NODE::WEIGHTS::JERK_REF];
    SC jerkLonCost  = u[0] / jerkRef; // fill here
    SC jerkLonWeight = x[MPC_NODE::WEIGHTS::JERK];
    SC jerkLonTerm  = CppAD::pow(jerkLonCost * jerkLonWeight,2); // fill here

    // Alpha term
    SC alphaRef = x[MPC_NODE::WEIGHTS::ALPHA_REF];
    SC alphaCost = u[1] / jerkRef; // fill here
    SC alphaWeight = x[MPC_NODE::WEIGHTS::ALPHA];
    SC alphaTerm = CppAD::pow(alphaCost * alphaWeight,2);

    // Lateral jerk term
    //The vehicles wheel-base is defined by the variable wheelBase
    double wheelBase = MPC_NODE::systemDynamics::wheelBase;
    SC jLat = (1 / wheelBase) * (2 * x[3] * CppAD::tan(x[6]) * x[4] + CppAD::pow(x[3], 2) * (CppAD::pow(CppAD::tan(x[6]), 2) + 1) * u[1]); // fill here
    SC jerkLatCost = jLat / jerkRef; // fill here
    SC jerkLatWeight = x[MPC_NODE::WEIGHTS::JERK];
    SC jerkLatTerm = CppAD::pow(jerkLatCost * jerkLatWeight, 2); // fill here
    // END TASK 2 CODE HERE

    // START TASK 3 CODE HERE
    // Velocity Term
    // if necessary use CppAD::sin(...), CppAD::cos(...), CppAD::tan(...), CppAD::pow(...), CppAD::sqrt(...)
    SC vScale = CppAD::CondExpGt(velocity, SC(10.0 / 3.6), velocity, SC(10.0 / 3.6));
    SC vCost = (velocity - x[3]) / vScale; // fill here
    SC vWeight = x[MPC_NODE::WEIGHTS::VEL];
    SC velTerm = CppAD::pow(vCost * vWeight, 2); // fill here
    // END TASK 3 CODE HERE

    // START TASK 4 CODE HERE
    // Dyn obj
    // if necessary use CppAD::sin(...), CppAD::cos(...), CppAD::tan(...), CppAD::pow(...), CppAD::sqrt(...)
    SC dynObjX = x[MPC_NODE::DYNOBJCOORDS::X];
    SC dynObjY = x[MPC_NODE::DYNOBJCOORDS::Y];
    SC dynObjRef = x[MPC_NODE::WEIGHTS::DYNOBJ_REF];
    SC dynObjDist = CppAD::sqrt(CppAD::pow(dynObjX - x[0], 2) + CppAD::pow(dynObjY - x[1], 2)); // fill here
    SC dynObjCost = CppAD::CondExpLt(dynObjDist, dynObjRef, CppAD::cos(SC(M_PI) * CppAD::pow(dynObjDist, 2) / CppAD::pow(dynObjRef, 2)) + 1, SC(0.0));
    SC dynObjWeight = x[MPC_NODE::WEIGHTS::DYNOBJ];
    SC dynObjTerm = CppAD::pow(dynObjCost * dynObjWeight, 2); //fill here
    // END TASK 4 CODE HERE

    // This cost term is relevant for Section 5
    // Traffic Light
    // Traffic Light
    // traffic light position change, change this point to a line
    // changed here
    SC TrafficLightX1 = x[MPC_NODE::TRAFFICLIGHT::X_TL];
    SC TrafficLightY1 = x[MPC_NODE::TRAFFICLIGHT::Y_TL];

    // changed here
    
    SC TrafficLightX = x[MPC_NODE::TRAFFICLIGHT::X_TL];
    SC TrafficLightY = x[MPC_NODE::TRAFFICLIGHT::Y_TL];
    SC TrafficLightState = x[MPC_NODE::TRAFFICLIGHT::STATE];
    SC TrafficLightRef = x[MPC_NODE::WEIGHTS::TRAFFICLIGHT_REF];
    SC TrafficLightDist = CppAD::sqrt(CppAD::pow(TrafficLightX - x[0], 2) + CppAD::pow(TrafficLightY - x[1], 2));
    SC TrafficLightCost = CppAD::CondExpEq(TrafficLightState,SC(1.0),CppAD::CondExpLt(TrafficLightDist, TrafficLightRef, CppAD::cos(SC(M_PI) * CppAD::pow(TrafficLightDist, 2) / CppAD::pow(TrafficLightRef, 2)) + 1, SC(0.0)),SC(0.0));
    SC TrafficLightWeight = x[MPC_NODE::WEIGHTS::TRAFFICLIGHT];
    SC TrafficLightTerm = CppAD::pow(TrafficLightCost * TrafficLightWeight, 2);

    // Return sum
    return pathTerm + jerkLonTerm + jerkLatTerm + alphaTerm + velTerm + dynObjTerm + TrafficLightTerm;
}

// Implements the final cost functions.
// Derivatives are automatically computed using Algorithmic Differentiation.
template <size_t STATE_DIM, size_t CONTROL_DIM, class MPC_NODE, typename SCALAR_EVAL, typename SCALAR>
template <typename SC>
SC CostTermFinal<STATE_DIM, CONTROL_DIM, MPC_NODE, SCALAR_EVAL, SCALAR>::evalLocal(const Eigen::Matrix<SC, STATE_DIM, 1> &x, const Eigen::Matrix<SC, CONTROL_DIM, 1> &u, const SC &t)
{
    //
    // Yaw deviation
    //

    // Find nearest sample in reference path and approx its orientation
    const size_t index = MPC_NODE::state_dim + MPC_NODE::params_dim_cfg + 3;
    SC distance = CppAD::sqrt(CppAD::pow(x[index] - x[0], 2) + CppAD::pow(x[index + 1] - x[1], 2));
    SC dx = x[index] - x[index - 3];
    SC dy = x[index + 1] - x[index - 2];
    for (size_t i = index + 3; i < STATE_DIM - 2; i += 3)
    {
        SC currDist = CppAD::sqrt(CppAD::pow(x[i] - x[0], 2) + CppAD::pow(x[i + 1] - x[1], 2));
        dx = CppAD::CondExpLt(currDist, distance, x[i] - x[i - 3], dx);
        dy = CppAD::CondExpLt(currDist, distance, x[i + 1] - x[i - 2], dy);
        distance = CppAD::CondExpLt(currDist, distance, currDist, distance);
    }

    return CppAD::pow((x[5] - CppAD::atan2(dy, dx)) * x[MPC_NODE::WEIGHTS::YAW], 2);
}

// Implements the system dynamics.
template <typename SCALAR>
void ACDC_VehcicleSystem<SCALAR>::computeControlledDynamics(const StateVector<STATE_DIM, SCALAR> &state, const time_t &t, const ControlVector<CONTROL_DIM, SCALAR> &control, StateVector<STATE_DIM, SCALAR> &derivative)
{
    // START TASK 1 CODE HERE
    // System Dynamics
    // use helping comments from Wiki

    
    derivative(0) = state(3) * cos(state(5)); // derivative of x
    derivative(1) = state(3) * sin(state(5)); // derivative of y
    derivative(2) = state(3); // derivative of s
    derivative(3) = state(4); // derivative of v
    derivative(4) = control(0); // derivative of a
    derivative(5) = (state(3)/wheelBase) * tan(state(6)); // derivative of psi
    derivative(6) = control(1); // derivative of delta
    // state(6) = atan(wheelBase/R);
    // state(3) = derivative(6)*R;

    // System State Vector:
    // state(0): x -> Position X
    // state(1): y -> Position Y
    // state(2): s -> Distance
    // state(3): v -> Vehicle Velocity
    // state(4): a -> Vehicle Acceleration
    // state(5): psi -> Vehicle Heading
    // state(6): delta -> Steering Angle

    // Control Vector:
    // control(0): j_lon -> longitudinal jerk
    // control(1): alpha -> Steering Rate

    // The vehicles wheel-base is defined by the class variable wheelBase

    // derivative(0) = state(0)/t ; // derivative of x
    // derivative(1) =  ; // derivative of y
    // derivative(2)  ; // derivative of s
    // derivative(3)  ; // derivative of v
    // derivative(4)  ; // derivative of a
    // derivative(5)  ; // derivative of psi
    // derivative(6)  ; // derivative of delta
    
    // END TASK 1 CODE HERE
    // changed here
    // EVALUATION PARAMETER
    vehicle_position = {state(0), state(1)};
    PLANNER obj;
    stop_line = obj.createStopLine(tr_i1.ingress_lane, 1);

    // stop_line = createStopLine(TrafficLightState.ingress_lane, 1); //added debug, 1 is width
    
    // stop_line = createStopLine(tr_i1.ingress_lane, 1);
    // const std::vector<geometry_msgs::Point> stop_line;
    std::vector<geometry_msgs::Point> stop_line_points;
    for (const auto& point : stop_line) {
        geometry_msgs::Point point_msg;
        // geometry_msgs::Point point;
        double x = point.x;
        double y = point.y;
        point_msg.x = x; // point[0];
        point_msg.y = y; // point[1];
        // Assuming stop_line is a vector of 2D points. Add z if it's 3D.
        stop_line_points.push_back(point_msg);
    }

    // geometry_msgs::Point vehicle_position_point;
    // vehicle_position_point.x = vehicle_position[0];
    // vehicle_position_point.y = vehicle_position[1];
    // Assuming vehicle_position is a 2D point. Add z if it's 3D.
    
    // stop_distance = PLANNER::distanceToStopLine(vehicle_position, stop_line_points);
    stop_distance = obj.distanceToStopLine(vehicle_position, stop_line_points);

    // stop_distance = distanceToStopLine(vehicle_position, stop_line_points);
    // stop_distance = PLANNER::distanceToStopLine(vehicle_position, stop_line_points);
    // changed here
}

// Called with our main frequency. Evaluates the MPC and sends new trajectory to controller.
void PLANNER::runMPC()
{
    // Temporary data for initialization
    StateVectorArray<state_dim> x_ref_init(K + 1);
    FeedbackArray<state_dim, control_dim> u0_fb(K);
    ControlVectorArray<control_dim> u0_ff(K);

    // Check for vehicleData time
    ros::Duration vehcileDataAge = ros::Time::now() - vehicleData.header.stamp;
    if (vehcileDataAge > ros::Duration(1.0 / cfg.mainFrequency))
    {
        ROS_WARN("Outdated vehicle data!");
        bReset = true;
    }

    // changed here
    // call createStopLine function
    width = 1.0;

    
    // stop_line = createStopLine(TrafficLight.ingress_lane, width);
    
    // stop_line = createStopLine(trafficlights.ingress_lane, width);
    // vehicle_position = {statex, statey};
    stop_distance = distanceToStopLine(vehicle_position, stop_line);
    // move this all to hpp maybe
    // changed here

    // We have a valid MPC solution from the last iteration
    if (!bReset)
    {
        // Adjust last trajectories according to odometry
        for (size_t i = 0; i < x_ref_init.size(); i++)
        {
            // Gather interpolation parameters
            size_t index;
            double factor;
            getInterpolationParameters(lastValidTrajectory.getReferenceStateTrajectory().getDataArray(), lastValidTrajectory.getReferenceStateTrajectory().getDataArray()[i][2] + ds, index, factor);

            // Interpolate state
            interpolateValues(lastValidTrajectory.getReferenceStateTrajectory().getDataArray(), factor, index, x_ref_init[i]);

            // Transform state into new frame
            x_ref_init[i][0] -= dx;
            x_ref_init[i][1] -= dy;
            x_ref_init[i][5] -= dYaw;
            const double ds_tmp = std::sqrt(x_ref_init[i][0] * x_ref_init[i][0] + x_ref_init[i][1] * x_ref_init[i][1]);
            const double theta = std::atan2(x_ref_init[i][1], x_ref_init[i][0]);
            x_ref_init[i][0] = ds_tmp * std::cos(theta - dYaw);
            x_ref_init[i][1] = ds_tmp * std::sin(theta - dYaw);

            // Interpolate feedback and feedforward controls
            if (i < u0_fb.size())
            {
                index = std::min(index, u0_fb.size() - 2);
                interpolateValues(lastValidTrajectory.getFeedbackTrajectory().getDataArray(), factor, index, u0_fb[i]);
                interpolateValues(lastValidTrajectory.getFeedforwardTrajectory().getDataArray(), factor, index, u0_ff[i]);
            }
        }

        // Recompute s
        x_ref_init.front()[2] = 0.0;
        for (size_t i = 1; i < x_ref_init.size(); i++)
        {
            x_ref_init[i][2] = x_ref_init[i - 1][2] + std::sqrt(std::pow(x_ref_init[i - 1][0] - x_ref_init[i][0], 2) + std::pow(x_ref_init[i - 1][1] - x_ref_init[i][1], 2));
        }

        // Check for too high deviations to odometry
        bool bLatReinit = false;
        bool bLonReinit = false;
        if (std::abs(x_ref_init.front()[3] - vehicleData.velocity) > cfg.deviationMaxV / 3.6 || std::abs(x_ref_init.front()[4] - vehicleData.acceleration) > cfg.deviationMaxA)
        {
            x_ref_init.front()[3] = vehicleData.velocity;
            x_ref_init.front()[4] = vehicleData.acceleration;
            bLonReinit = true;
        }
        if (std::abs(x_ref_init.front()[1]) > std::abs(cfg.deviationMaxY) || std::abs(x_ref_init.front()[5]) > std::abs(cfg.deviationMaxYaw * M_PI / 180.0))
        {
            x_ref_init.front()[1] = 0.0;
            x_ref_init.front()[5] = 0.0;
            x_ref_init.front()[6] = vehicleData.steering_angle;
            bLatReinit = true;
        }
        else if (std::abs(x_ref_init.front()[6] - vehicleData.steering_angle) > cfg.deviationMaxDelta * M_PI / 180.0)
        {
            x_ref_init.front()[6] = vehicleData.steering_angle;
        }
        x_ref_init.front()[0] = 0.0;
        x_ref_init.front()[2] = 0.0;

        bReset = bLonReinit && bLatReinit;
    }

    // In case of reset, we initialize everything with zeros
    if (bReset)
    {
        x_ref_init.setConstant(StateVector<state_dim>::Zero());
        u0_fb.setConstant(FeedbackMatrix<state_dim, control_dim>::Zero());
        u0_ff.setConstant(ControlVector<control_dim>::Zero());

        // Set vehicle data
        x_ref_init.front()[3] = vehicleData.velocity;
        x_ref_init.front()[4] = vehicleData.acceleration;
        x_ref_init.front()[6] = vehicleData.steering_angle;
    }

    // Time measurement start
    auto start_time = std::chrono::high_resolution_clock::now();

    // Update the environment data
    updateEnvironmentData();

    // Run real-time iteration loop for this timestep
    NLOptConSolver<state_dim, control_dim>::Policy_t initialGuess(x_ref_init, u0_ff, u0_fb, ilqr_settings_mpc.dt);
    ilqr_mpc->setInitialGuess(initialGuess);
    ilqr_mpc->prepareMPCIteration();
    ilqr_mpc->changeInitialState(x_ref_init.front());
    bool success;
    for (size_t i = 0; i < cfg.mpcIterations; i++)
    {
        success = ilqr_mpc->finishMPCIteration();
        if (success && i < cfg.mpcIterations - 1)
            ilqr_mpc->prepareMPCIteration();
        else
            break;
    }

    // Time measurement stop
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

    if (!success || ilqr_mpc->getBackend()->getSummary().totalCosts.back() > cfg.objectiveTolerance || std::isnan(ilqr_mpc->getSolution().getReferenceStateTrajectory().getDataArray()[1][0]))
    {
        ROS_WARN_STREAM("Trajectory optimization FAILED after " << duration << "s !!!");
        if (success)
            ROS_INFO_STREAM("Final cost value is " << ilqr_mpc->getBackend()->getSummary().totalCosts.back());
        bReset = true;
    }
    else
    {
        ROS_INFO_STREAM("Trajectory optimization SUCCESSFUL after " << duration << "s.");
        lastValidTrajectory = ilqr_mpc->getSolution();
        bReset = false;
    }

    publishTrajectory();
}

// changed here-added
std::vector<geometry_msgs::Point> PLANNER::createStopLine(const std::vector<geometry_msgs::Point>& ingress_lane, double width=1) 
{
    std::vector<geometry_msgs::Point> stop_line(2);

    if (ingress_lane.size() < 2) {
        // Not enough points to define a line
        return stop_line;
    }

    // Calculate the direction of the ingress lane
    geometry_msgs::Point dir;
    dir.x = ingress_lane.back().x - ingress_lane[ingress_lane.size() - 2].x;
    dir.y = ingress_lane.back().y - ingress_lane[ingress_lane.size() - 2].y;

    // Calculate the orthogonal direction
    geometry_msgs::Point ortho_dir;
    ortho_dir.x = -dir.y;
    ortho_dir.y = dir.x;

    // Normalize the orthogonal direction
    double length = std::sqrt(ortho_dir.x * ortho_dir.x + ortho_dir.y * ortho_dir.y);
    ortho_dir.x /= length;
    ortho_dir.y /= length;

    // Scale by the desired width
    ortho_dir.x *= width / 2;
    ortho_dir.y *= width / 2;

    // Calculate the stop line points
    stop_line[0].x = ingress_lane.back().x + ortho_dir.x;
    stop_line[0].y = ingress_lane.back().y + ortho_dir.y;
    stop_line[1].x = ingress_lane.back().x - ortho_dir.x;
    stop_line[1].y = ingress_lane.back().y - ortho_dir.y;
    
    return stop_line;
}

double PLANNER::calculateDistance(std::vector<double>& point1, std::vector<double>& point2) 
{
    double dx = point2[0] - point1[0];
    double dy = point2[1] - point1[1];
    // std::cout << "Inside calculateDistance in trajectory_planner.cpp";
    return std::sqrt(dx * dx + dy * dy);
}

double PLANNER::distanceToStopLine(std::vector<double>& vehicle_position, const std::vector<geometry_msgs::Point>& stop_line) 
{
    std::vector<double> stop_line_point1 = {stop_line[0].x, stop_line[0].y};
    std::vector<double> stop_line_point2 = {stop_line[1].x, stop_line[1].y};
    double distance1 = calculateDistance(vehicle_position, stop_line_point1);
    double distance2 = calculateDistance(vehicle_position, stop_line_point2);
    // double distance1 = calculateDistance(vehicle_position, stop_line[0]);   
    // double distance2 = calculateDistance(vehicle_position, stop_line[1]);
    // std::cout << "Inside distanceToStopLine in trajectory_planner.cpp";
    return std::min(distance1, distance2);
}

double dotProduct(const geometry_msgs::Point& vec1, const geometry_msgs::Point& vec2) {
    // std::cout << "Inside dotProduct in trajectory_planner.cpp";
    return vec1.x * vec2.x + vec1.y * vec2.y;
}

bool isMovingTowardsStopLine(const geometry_msgs::Point& vehicle_position, const geometry_msgs::Point& vehicle_direction, const std::vector<geometry_msgs::Point>& stop_line) {
    // Calculate the direction of the stop line
    geometry_msgs::Point stop_line_dir;
    stop_line_dir.x = stop_line[1].x - stop_line[0].x;
    stop_line_dir.y = stop_line[1].y - stop_line[0].y;

    // Calculate the dot product of the vehicle's direction and the stop line's direction
    double dot = dotProduct(vehicle_direction, stop_line_dir);
    // std::cout << "Inside isMovingTowardsStopLine in trajectory_planner.cpp";
    // If the dot product is positive, the vehicle is moving towards the stop line
    return dot > 0;
}
//changed here - added
/*
 *
 * Entry point of the ROS node.
 *
 */
int main(int argc, char **argv)
{
    PLANNER mpc(argc, argv);
    mpc.init();

    return 0;
}
