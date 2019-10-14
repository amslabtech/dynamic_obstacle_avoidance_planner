#include "dynamic_obstacle_avoidance_planner/obstacle_tracker_kf.h"

KalmanFilter::KalmanFilter(void)
{
    sigma_a = 0.05;
}

Eigen::Matrix4d KalmanFilter::get_f(double dt)
{
    Eigen::Matrix4d f;
    f << 1.0, 0.0,  dt, 0.0,
         0.0, 1.0, 0.0,  dt,
         0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 1.0;
    return f;
}

Eigen::Matrix4d KalmanFilter::get_q(double dt)
{
    Eigen::Matrix<double, 4, 2> g;
    g << dt * dt / 2.0,           0.0,
                   0.0, dt * dt / 2.0,
                    dt,           0.0,
                   0.0,            dt;
    Eigen::Matrix4d q = sigma_a * sigma_a * g * g.transpose();
    return q;
}

Obstacle::Obstacle(void)
{
    x = Eigen::Vector4d::Zero();

    p << 1e2, 0.0, 0.0, 0.0,
         0.0, 1e2, 0.0, 0.0,
         0.0, 0.0, 1e2, 0.0,
         0.0, 0.0, 0.0, 1e2;

    h << 1.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0;

    r << 1e-1,  0.0,
          0.0, 1e-1;

    last_time = ros::Time::now().toSec();
    likelihood = 1.0;
    lifetime = 10;
    age = 0;
    not_observed_time = 0;
}

Obstacle::Obstacle(const Obstacle& obstacle)
{
    x = obstacle.x;
    p = obstacle.p;
    r = obstacle.r;
    h = obstacle.h;
    likelihood = obstacle.likelihood;
    last_time = obstacle.last_time;
    lifetime = obstacle.lifetime;
    age = obstacle.age;
    not_observed_time = obstacle.not_observed_time;
}

Obstacle::Obstacle(const Eigen::Vector2d& position)
{
    x << position(0), position(1), 0.0, 0.0;

    p << 1e2, 0.0, 0.0, 0.0,
         0.0, 1e2, 0.0, 0.0,
         0.0, 0.0, 1e2, 0.0,
         0.0, 0.0, 0.0, 1e2;

    h << 1.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0;

    r << 1e-1,  0.0,
          0.0, 1e-1;

    last_time = ros::Time::now().toSec();
    likelihood = 1.0;
    lifetime = 10;
    age = 0;
    not_observed_time = 0;
}

Eigen::Vector2d Obstacle::get_position(void)
{
    return x.segment(0, 2);
}

void Obstacle::update(const Eigen::Vector2d& z)
{
    // std::cout << "update" << std::endl;
    // std::cout << "Z:\n" << z << std::endl;
    Eigen::Vector2d e = z - h * x;
    Eigen::Matrix2d s = h * p * h.transpose() + r;
    Eigen::Matrix<double, 4, 2> k = p * h.transpose() * s.inverse();
    // std::cout << "K:\n" << k << std::endl;
    x = x + k * e;
    // std::cout << "X:\n" << x << std::endl;
    p = (Eigen::Matrix4d::Identity() - k * h) * p;
    // std::cout << "P:\n" << p << std::endl;

    not_observed_time = 0;
}

void Obstacle::predict(void)
{
    // std::cout << "predict" << std::endl;
    double current_time = ros::Time::now().toSec();
    double dt = current_time - last_time;
    last_time = current_time;
    age += dt;
    not_observed_time += dt;
    // std::cout << "age: " << age << std::endl;
    // std::cout << "not_observed_time: " << not_observed_time << std::endl;

    Eigen::Matrix4d f = kf.get_f(dt);
    x = f * x;
    // std::cout << "X:\n" << x << std::endl;;
    Eigen::Matrix4d q = kf.get_q(dt);
    p = f * p * f.transpose() + q;
    // std::cout << "P:\n" << p << std::endl;;
}

void Obstacle::predict(double dt)
{
    // std::cout << "predict" << std::endl;
    age += dt;
    not_observed_time += dt;
    // std::cout << "age: " << age << std::endl;
    // std::cout << "not_observed_time: " << not_observed_time << std::endl;

    Eigen::Matrix4d f = kf.get_f(dt);
    x = f * x;
    // std::cout << "X:\n" << x << std::endl;;
    Eigen::Matrix4d q = kf.get_q(dt);
    p = f * p * f.transpose() + q;
    // std::cout << "P:\n" << p << std::endl;;
}

double Obstacle::calculate_likelihood(void)
{
    Eigen::Matrix2d m = p.block<2, 2>(0, 0);// up left 2x2
    Eigen::EigenSolver<Eigen::Matrix2d> es(m);
    if(!es.info()){
        Eigen::Vector2d e_values = es.eigenvalues().real();
        const double CHI2 = 9.21034;// chi-square, 99%
        double a, b;// ellipse parameter
        if(e_values(0) > e_values(1)){
            a = sqrt(CHI2 * e_values(0));
            b = sqrt(CHI2 * e_values(1));
        }else{
            a = sqrt(CHI2 * e_values(1));
            b = sqrt(CHI2 * e_values(0));
        }
        if(a * b > 1e-5){
            likelihood = 1.0 / (a * b);
        }else{
            likelihood = 1e3;
        }
    }else{
        std::cout << "Eigen solver error: " << es.info() << std::endl;
    }
    return likelihood;
}

ObstacleTrackerKF::ObstacleTrackerKF(void)
:SAME_OBSTACLE_THRESHOLD(0.8), ERASE_LIKELIHOOD_THREHSOLD(0.8)
, NOT_OBSERVED_TIME_THRESHOLD(5.0), DEFAULT_LIFE_TIME(10.0)
{
    obstacles.clear();
}

void ObstacleTrackerKF::set_obstacles_pose(const geometry_msgs::PoseArray& pose_array)
{
    std::cout << "tracking " << obstacles.size() << " obstacles" << std::endl;
    std::cout << "observed " << pose_array.poses.size() << " obstacles" << std::endl;
    std::vector<Eigen::Vector2d> observed_obstacles;
    observed_obstacles.reserve(pose_array.poses.size());
    for(const auto& pose : pose_array.poses){
        Eigen::Vector2d position;
        position << pose.position.x, pose.position.y;
        observed_obstacles.push_back(position);
    }
    associate_obstacles(observed_obstacles);
    update_tracking(observed_obstacles);

    std::cout << "--- predict ---" << std::endl;
    auto it = obstacles.begin();
    while(it != obstacles.end()){
        it->second.predict();
        std::cout << "not observed: " << it->second.not_observed_time << std::endl;
        std::cout << "age: " << it->second.age << std::endl;
        std::cout << "likelihood: " << it->second.likelihood << std::endl;
        if((it->second.likelihood > ERASE_LIKELIHOOD_THREHSOLD) && (it->second.not_observed_time < NOT_OBSERVED_TIME_THRESHOLD)){
            it->second.lifetime = DEFAULT_LIFE_TIME;
            ++it;
        }else{
            if(it->second.lifetime > it->second.age){
                // ???
                ++it;
            }else{
                std::cout << "\033[31mobstacle " << it->first << " was erased\033[0m" << std::endl;
                it = obstacles.erase(it);
            }
        }
    }
}

void ObstacleTrackerKF::get_velocities(std::vector<Eigen::Vector3d>& velocities)
{
    for(auto it=obstacles.begin();it!=obstacles.end();++it){
        Eigen::Vector3d v;
        v << it->second.x(2), it->second.x(3), 0;
        velocities.push_back(v);
    }
}

void ObstacleTrackerKF::get_poses(std::vector<Eigen::Vector3d>& poses)
{
    for(auto it=obstacles.begin();it!=obstacles.end();++it){
        Eigen::Vector3d p;
        p << it->second.x(0), it->second.x(1), 0;
        poses.push_back(p);
    }
}

void ObstacleTrackerKF::associate_obstacles(const std::vector<Eigen::Vector2d>& observed_obstacles)
{
    std::cout << "--- associate obstacles ---" << std::endl;
    int cluster_num = obstacles.size();
    int observed_obstacles_num = observed_obstacles.size();

    int matrix_size = cluster_num + observed_obstacles_num;

    Eigen::MatrixXi association_matrix(matrix_size, matrix_size);
    for(int i=0;i<matrix_size;i++){
        for(int j=0;j<matrix_size;j++){
            if(i < cluster_num && j < observed_obstacles_num){
                association_matrix(i, j) = get_distance(obstacles[get_id_from_index(i)], observed_obstacles[j]) * 100;
            }else if(i < cluster_num && !(j < observed_obstacles_num)){
                association_matrix(i, j) = SAME_OBSTACLE_THRESHOLD * 100;
            }else if(!(i < cluster_num) && j < observed_obstacles_num){
                association_matrix(i, j) = SAME_OBSTACLE_THRESHOLD * 100;
            }else{
                association_matrix(i, j) = SAME_OBSTACLE_THRESHOLD * 100;
            }
        }
    }
    solve_hungarian_method(association_matrix);
}

double ObstacleTrackerKF::get_distance(const Obstacle& obstacle, const Eigen::Vector2d& position)
{
    std::cout << "--- get distance ---" << std::endl;
    Obstacle copied_obstacle(obstacle);
    copied_obstacle.update(position);
    copied_obstacle.predict();
    double likelihood = copied_obstacle.calculate_likelihood();
    return likelihood;
}

int ObstacleTrackerKF::get_id_from_index(int index)
{
    int index_ = 0;
    for(auto it=obstacles.begin();it!=obstacles.end();++it){
        if(index_ == index){
            return it->first;
        }
        index_++;
    }
    return -1;
}

int ObstacleTrackerKF::get_new_id(void)
{
    int new_id = 0;
    int obstacles_num = obstacles.size();
    for(int i=0;i<obstacles_num;i++){
        bool used_id_flag = false;
        for(auto it=obstacles.begin();it!=obstacles.end();++it){
            if(new_id == it->first){
                used_id_flag = true;
                break;
            }
        }
        if(!used_id_flag){
            return new_id;
        }
        new_id++;
    }
    return new_id;
}

void ObstacleTrackerKF::solve_hungarian_method(Eigen::MatrixXi& matrix)
{
    // reference: http://www.prefield.com/algorithm/math/hungarian.html
    const double inf = 1e6;
    int n = matrix.rows(), p, q;
    std::vector<int> fx(n, inf), fy(n, 0);
    std::vector<int> x(n, -1), y(n, -1);
    for(int i = 0;i < n;++i){
        for(int j = 0;j < n;++j){
            fx[i] = std::max(fx[i], matrix(i, j));
        }
    }
    int count = 0;
    for(int i = 0;i < n;){
        if(count>1000){
            obstacles.clear();
            candidates.clear();
            std::cout << "after 1000 times loop,  break!!!!!" << std::endl;
            return;
        }
        std::vector<int> t(n, -1), s(n+1, i);
        for(p = q = 0;p <= q && x[i] < 0;++p){
            for(int k = s[p], j = 0;j < n && x[i] < 0;++j){
                if(fx[k] + fy[j] == matrix(k, j) && t[j] < 0){
                    s[++q] = y[j];
                    t[j] = k;
                    if(s[q] < 0){
                        for(p = j;p >= 0;j = p){
                            y[j] = k = t[j];
                            p = x[k];
                            x[k] = j;
                        }
                    }
                }
            }
        }
        if(x[i] < 0){
            int d = inf;
            for(int k = 0;k <= q;++k){
                for(int j = 0;j < n;++j){
                    if(t[j] < 0){
                        d = std::min(d, fx[s[k]] + fy[j] - matrix(s[k], j));
                    }
                }
                for(int j = 0;j < n;++j){
                    fy[j] += (t[j] < 0 ? 0 : d);
                }
                for(int k = 0;k <= q;++k){
                    fx[s[k]] -= d;
                }
            }
        }else{
            ++i;
        }
        i += 1;
    }
    candidates.resize(n);
    for(int i = 0;i < n;++i){
        candidates[i] = get_id_from_index(y[i]);
    }
}

void ObstacleTrackerKF::update_tracking(const std::vector<Eigen::Vector2d>& observed_obstacles)
{
    std::cout << "--- update tracking ---" << std::endl;
    for(auto it=observed_obstacles.begin();it!=observed_obstacles.end();++it){
        int id = candidates[it - observed_obstacles.begin()];
        auto obstacle_it = obstacles.find(id);
        if(obstacle_it != obstacles.end()){
            // this obstacle has already been tracked
            obstacle_it->second.update(*it);
        }else{
            // new obstacle
            Obstacle obstacle(*it);
            int new_id = get_new_id();
            obstacles[new_id] = obstacle;
            std::cout << "new obstacle (id:" << new_id << ") was added" << std::endl;
        }
    }
}
