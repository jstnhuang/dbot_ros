

#pragma once

#include <boost/thread/mutex.hpp>

#include <Eigen/Dense>

#include <vector>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <dbot/rao_blackwell_coordinate_particle_filter.hpp>
#include <dbot/models/process_models/brownian_object_motion_model.hpp>
#include <dbot/models/observation_models/kinect_image_observation_model_cpu.hpp>
#ifdef BUILD_GPU
#include <dbot/models/observation_models/kinect_image_observation_model_gpu/kinect_image_observation_model_gpu.hpp>
#endif

#include <fl/model/process/linear_state_transition_model.hpp>
#include <fl/model/process/interface/state_transition_function.hpp>

#include <osr/pose_vector.hpp>
#include <osr/composed_vector.hpp>

namespace bot
{


/**
 * \brief RbcParticleFilterObjectTracker
 *
 * Yaml config file:
 * \code
 * object:
 *  package:
 *  directory:
 *  files: [  ]
 *  sampling_blocks: [ [0, 1, 2], [3, 4], [6, 7] ]
 *
 * \endcode
 */
class RbcParticleFilterObjectTracker
{
public:
    typedef Eigen::VectorXd StateVector;
    typedef osr::PoseBlock<StateVector> StateBlock;

    typedef osr::FreeFloatingRigidBodiesState<> State;
    typedef State::Scalar Scalar;

    typedef Eigen::Matrix<fl::Real, -1, 1> Input;

    typedef fl::StateTransitionFunction<State, State, Input> StateTransition;

    typedef fl::LinearStateTransitionModel<State, Input> NewStateTransition;

    typedef dbot::BrownianObjectMotionModel<State> OldStateTransition;

    typedef dbot::KinectImageObservationModelCPU<Scalar, State>
        ObservationModelCPUType;

#ifdef BUILD_GPU
    typedef dbot::KinectImageObservationModelGPU<State> ObservationModelGPUType;
#endif

    typedef ObservationModelCPUType::Base ObservationModel;
    typedef ObservationModelCPUType::Observation Observation;

    typedef dbot::RBCoordinateParticleFilter<StateTransition, ObservationModel>
        FilterType;

    typedef typename Eigen::Transform<fl::Real, 3, Eigen::Affine> Affine;

    RbcParticleFilterObjectTracker();

    void Reset(std::vector<Eigen::VectorXd> initial_states,
               const sensor_msgs::Image& ros_image);

    void Initialize(std::vector<Eigen::VectorXd> initial_states,
                    const sensor_msgs::Image& ros_image,
                    Eigen::Matrix3d camera_matrix);

    Eigen::VectorXd Filter(const sensor_msgs::Image& ros_image);

private:
    Scalar last_measurement_time_;

    boost::mutex mutex_;
    ros::NodeHandle node_handle_;
    ros::Publisher object_publisher_;

    boost::shared_ptr<FilterType> filter_;

    // parameters
    //    std::string object_model_uri_;
    //    std::string object_model_path_;
    std::vector<std::string> object_names_;
    int downsampling_factor_;

    std::vector<Eigen::Vector3d> centers_;

    std::vector<Affine> default_poses_;
};
}
