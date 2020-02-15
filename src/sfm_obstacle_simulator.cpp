#include <random>

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <Eigen/Dense>

int NUM_OBSTACLE;
double NEIGHBER_RANGE = 3.0;
double LAMBDA = 2.0;
double GAMMA = 0.35;
double N_PRIME = 3.0;
double N = 2.0;
double MAX_VELOCITY = 1.5;
double DESIRED_FORCE_FACTOR;
double SOCIAL_FORCE_FACTOR;
double HZ;
double INITIAL_KEEP_OUT_POSITION_X;
double INITIAL_KEEP_OUT_POSITION_Y;
double INITIAL_KEEP_OUT_RANGE;


std::string ROBOT_FRAME;
std::string WORLD_FRAME;
std::string OBS_FRAME;
double SIMULATION_SQUARE_LENGTH;

class SFMObstacle
{
public:
    SFMObstacle(void);

    unsigned int id;
    Eigen::Vector3d pose;
    Eigen::Vector3d velocity;
    Eigen::Vector3d current_goal;
    double preferred_speed;
    bool dodging_right;
    Eigen::Vector3d last_desired_force;
    Eigen::Vector3d last_social_force;
private:
};

SFMObstacle::SFMObstacle(void)
{
    id = 0;
    pose = Eigen::Vector3d::Zero();
    velocity = Eigen::Vector3d::Zero();
    current_goal = Eigen::Vector3d::Zero();
    preferred_speed = 1.0;
    dodging_right = true;
    last_desired_force = Eigen::Vector3d::Zero();
    last_social_force = Eigen::Vector3d::Zero();
}

std::ostream& operator<<(std::ostream& out, const SFMObstacle& sfmo)
{
    out << "id: " << sfmo.id << "\n"
        << "pose: " << sfmo.pose.transpose() << "\n"
        << "velocity: " << sfmo.velocity.transpose() << "\n"
        << "current speed: " << sfmo.velocity.norm() << "\n"
        << "current goal: " << sfmo.current_goal.transpose() << "\n"
        << "last desired force: " << sfmo.last_desired_force.transpose() << "\n"
        << "last social force: " << sfmo.last_social_force.transpose();
    return out;
}

Eigen::Vector3d get_next_goal(const SFMObstacle& agent)
{
    Eigen::Vector3d new_goal = Eigen::Vector3d::Zero();
    while(1){
        new_goal = Eigen::Vector3d::Random() * SIMULATION_SQUARE_LENGTH * 0.5;
        double distance_from_agent = (new_goal - agent.pose).norm();
        if(distance_from_agent > SIMULATION_SQUARE_LENGTH * 0.5){
            return new_goal;
        }
    }
}

Eigen::Vector3d get_social_force(const SFMObstacle& agent, const std::vector<SFMObstacle>& obstacles)
{
    Eigen::Vector3d force = Eigen::Vector3d::Zero();

    for(auto obstacle : obstacles){
        if(agent.id == obstacle.id){
            continue;
        }

        Eigen::Vector3d diff_vector = obstacle.pose - agent.pose;
        double distance = diff_vector.norm();
        if (distance > NEIGHBER_RANGE){
            continue;
        }
        Eigen::Vector3d diff_direction = diff_vector.normalized();

        double other_angle = atan2(diff_direction(1), diff_direction(0));
        double agent_angle = atan2(agent.velocity(1), agent.velocity(0));
        double angle_fov = other_angle - agent_angle;
        angle_fov = atan2(sin(angle_fov), cos(angle_fov));
        if(fabs(angle_fov) > M_PI / 6.0){
            continue;
        }

        Eigen::Vector3d velocity_diff = agent.velocity - obstacle.velocity;

        Eigen::Vector3d interaction_vector = LAMBDA * velocity_diff + diff_direction;
        Eigen::Vector3d interaction_direction = interaction_vector.normalized();

        double interaction_angle = atan2(interaction_direction(1), interaction_direction(0));
        double theta_angle = other_angle - interaction_angle;
        theta_angle = atan2(sin(theta_angle), cos(theta_angle));

        double sign_of_theta = (fabs(theta_angle) < 1e-2) ? 0.0 : theta_angle / fabs(theta_angle);

        double b = GAMMA * interaction_vector.norm();

        double force_velocity_amount = -std::exp(-diff_vector.norm() / b - (N_PRIME * b * theta_angle) * (N_PRIME * b * theta_angle));
        double force_angle_amount = -sign_of_theta * std::exp(-diff_vector.norm() / b - (N * b * theta_angle) * (N * b * theta_angle));

        Eigen::Vector3d force_velocity = force_velocity_amount * interaction_direction;

        Eigen::Vector3d interaction_direction_normal;
        if(agent.dodging_right){
            interaction_direction_normal << -interaction_direction(1), interaction_direction(0), interaction_direction(2);
        }else{
            interaction_direction_normal << interaction_direction(1), -interaction_direction(0), interaction_direction(2);
        }

        Eigen::Vector3d force_angle = force_angle_amount * interaction_direction_normal;

        force += force_velocity + force_angle;
    }
    return force;
}

Eigen::Vector3d get_desired_force(const SFMObstacle& agent)
{
    Eigen::Vector3d force = Eigen::Vector3d::Zero();
    force = (agent.current_goal - agent.pose).normalized();
    return force;
}

void set_obs_list(const std::vector<SFMObstacle>& obstacles, std::vector<geometry_msgs::TransformStamped>& obs_list)
{
    obs_list.clear();
    for(auto& obs : obstacles){
        geometry_msgs::TransformStamped tfs;
        tfs.header.stamp = ros::Time::now();
        tfs.header.frame_id = WORLD_FRAME;
        tfs.child_frame_id = OBS_FRAME + std::to_string(obs.id);
        tfs.transform.translation.x = obs.pose(0);
        tfs.transform.translation.y = obs.pose(1);
        tfs.transform.rotation = tf::createQuaternionMsgFromYaw(atan2(obs.velocity(1), obs.velocity(0)));
        obs_list.push_back(tfs);
    }
}

void simulate_one_step(std::vector<SFMObstacle>& obstacles, double dt)
{
    if(dt < 0.0){
        return;
    }
    // std::cout << "simulate one step" << std::endl;
    for(auto& obs : obstacles){
        // std::cout << obs << std::endl;
        Eigen::Vector3d relative_goal = obs.current_goal - obs.pose;
        if(relative_goal.norm() < 0.5){
            obs.current_goal = Eigen::Vector3d::Random() * SIMULATION_SQUARE_LENGTH * 0.5;
            obs.current_goal = get_next_goal(obs);
            obs.current_goal(2) = 0;
        }
        Eigen::Vector3d desired_force = get_desired_force(obs);
        obs.last_desired_force = desired_force;
        // std::cout << obs.last_desired_force.transpose() << std::endl;
        Eigen::Vector3d social_force = get_social_force(obs, obstacles);
        obs.last_social_force = social_force;
        // std::cout << obs.last_social_force.transpose() << std::endl;
        Eigen::Vector3d force = DESIRED_FORCE_FACTOR * desired_force + SOCIAL_FORCE_FACTOR * social_force;
        obs.velocity += obs.velocity + force * dt;
        // std::cout << obs.velocity.transpose() << std::endl;
        double speed = obs.velocity.norm();
        if(speed > obs.preferred_speed){
            obs.velocity = obs.velocity.normalized() * obs.preferred_speed;
        }
        obs.pose += obs.velocity * dt;
        // std::cout << obs.pose.transpose() << std::endl;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sfm_obstacle_simulator");
    std::cout << "=== sfm_obstacle_simulator ===" << std::endl;
    ros::NodeHandle nh;

    ros::NodeHandle local_nh("~");

    local_nh.param<std::string>("/dynamic_avoidance/ROBOT_FRAME", ROBOT_FRAME, {"base_link"});
    local_nh.param<std::string>("/dynamic_avoidance/WORLD_FRAME", WORLD_FRAME, {"map"});
    local_nh.param<std::string>("/dynamic_avoidance/OBSTACLES_FRAME", OBS_FRAME, {"obs"});
    local_nh.param<double>("HZ", HZ, {20.0});
    local_nh.param<int>("NUM_OBSTACLE", NUM_OBSTACLE, {20});
    local_nh.param<double>("DESIRED_FORCE_FACTOR", DESIRED_FORCE_FACTOR, {20.0});
    local_nh.param<double>("SOCIAL_FORCE_FACTOR", SOCIAL_FORCE_FACTOR, {100.0});
    local_nh.param<double>("SIMULATION_SQUARE_LENGTH", SIMULATION_SQUARE_LENGTH, {20.0});
    local_nh.param<double>("INITIAL_KEEP_OUT_POSITION_X", INITIAL_KEEP_OUT_POSITION_X, {-10.0});
    local_nh.param<double>("INITIAL_KEEP_OUT_POSITION_Y", INITIAL_KEEP_OUT_POSITION_Y, {0.0});
    local_nh.param<double>("INITIAL_KEEP_OUT_RANGE", INITIAL_KEEP_OUT_RANGE, {3.0});

    std::cout << "HZ: " << HZ << std::endl;
    std::cout << "NUM_OBSTACLE: " << NUM_OBSTACLE << std::endl;
    std::cout << "DESIRED_FORCE_FACTOR: " << DESIRED_FORCE_FACTOR << std::endl;
    std::cout << "SOCIAL_FORCE_FACTOR: " << SOCIAL_FORCE_FACTOR << std::endl;
    std::cout << "SIMULATION_SQUARE_LENGTH: " << SIMULATION_SQUARE_LENGTH << std::endl;
    std::cout << "INITIAL_KEEP_OUT_POSITION_X: " << INITIAL_KEEP_OUT_POSITION_X << std::endl;
    std::cout << "INITIAL_KEEP_OUT_POSITION_Y: " << INITIAL_KEEP_OUT_POSITION_Y << std::endl;
    std::cout << "INITIAL_KEEP_OUT_RANGE: " << INITIAL_KEEP_OUT_RANGE << std::endl;

    ros::Rate loop_rate(HZ);

    tf::TransformBroadcaster obs_broadcaster;

    srand((unsigned int)time(0));// for Eigen
    std::random_device rnd;
    std::mt19937 mt(rnd());
    std::uniform_real_distribution<> dist(1.0, MAX_VELOCITY);
    std::uniform_int_distribution<> dist_bool(0, 1);

    std::vector<SFMObstacle> obstacles;
    obstacles.clear();
    // initialize
    Eigen::Vector3d init_pos(INITIAL_KEEP_OUT_POSITION_X, INITIAL_KEEP_OUT_POSITION_Y, 0.0);
    for(int i=0;i<NUM_OBSTACLE;i++){
        SFMObstacle o;
        o.id = i;
        while(1){
            o.pose = Eigen::Vector3d::Random() * SIMULATION_SQUARE_LENGTH * 0.5;
            o.pose(2) = 0;
            if((o.pose - init_pos).norm() > INITIAL_KEEP_OUT_RANGE){
                break;
            }
        }
        o.velocity = Eigen::Vector3d::Random();
        o.velocity(2) = 0;
        o.current_goal = Eigen::Vector3d::Random() * SIMULATION_SQUARE_LENGTH * 0.5;
        o.current_goal(2) = 0;
        o.preferred_speed = dist(mt);
        o.dodging_right = dist(mt);
        obstacles.push_back(o);
    }

    tf::TransformListener listener;

    while(ros::ok()){
        bool robot_added_flag = false;
        try{
            static bool first_tf_flag = true;
            static double last_tf_time = 0;
            static Eigen::Vector3d last_robot_pose = Eigen::Vector3d::Zero();
            Eigen::Vector3d robot_velocity = Eigen::Vector3d::Zero();
            Eigen::Vector3d robot_pose = Eigen::Vector3d::Zero();

            geometry_msgs::PoseStamped robot_p;
            robot_p.header.frame_id = ROBOT_FRAME;
            robot_p.header.stamp = ros::Time(0);
            robot_p.pose.orientation = tf::createQuaternionMsgFromYaw(0);
            listener.transformPose(WORLD_FRAME, robot_p, robot_p);
            robot_pose(0) = robot_p.pose.position.x;
            robot_pose(1) = robot_p.pose.position.y;
            if(!first_tf_flag){
                double dt = robot_p.header.stamp.toSec() - last_tf_time;
                if(dt > 1e-4){
                    robot_velocity = (robot_pose - last_robot_pose) / dt;
                }else{
                    robot_velocity = Eigen::Vector3d::Zero();
                }
            }else{
                first_tf_flag = false;
            }
            SFMObstacle robot_o;
            robot_o.id = obstacles.size();
            robot_o.pose = robot_pose;
            robot_o.velocity = robot_velocity;
            robot_o.current_goal = robot_o.pose + robot_o.velocity * 5.0;// 5[s] after
            obstacles.push_back(robot_o);
            std::cout << "robot is added" << std::endl;
            std::cout << robot_o << std::endl;
            std::cout << robot_p.header.stamp.toSec() - last_tf_time << std::endl;
            std::cout << last_robot_pose.transpose() << std::endl;

            robot_added_flag = true;
            last_robot_pose = robot_pose;
            last_tf_time = robot_p.header.stamp.toSec();
        }catch(tf::TransformException& ex){
            std::cout << ex.what() << std::endl;
        }

        simulate_one_step(obstacles, 1 / HZ);

        if(robot_added_flag){
            obstacles.pop_back();
        }

        std::vector<geometry_msgs::TransformStamped> obs_list;
        obs_list.clear();
        set_obs_list(obstacles, obs_list);

        std::cout << "===" << std::endl;
        for(const auto obs : obstacles){
            // std::cout << obs << std::endl;
        }

        obs_broadcaster.sendTransform(obs_list);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
};
