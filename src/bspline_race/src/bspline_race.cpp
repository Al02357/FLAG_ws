#include "bspline_race/bspline_race.h"

namespace FLAG_Race

{
    UniformBspline::UniformBspline(const int &p,  const int &n, const double &beta, const int &D, 
                                                 const Eigen::MatrixXd &s_ini, const Eigen::MatrixXd &s_ter)
    {
        initUniformBspline(p, n,beta, D, s_ini, s_ter); 
    }

    UniformBspline::~UniformBspline() {}

    void UniformBspline::initUniformBspline(const int &p,  const int &n, const double &beta, const int &D, 
                                                 const Eigen::MatrixXd &s_ini, const Eigen::MatrixXd &s_ter)
    {
        p_ = p; 
        n_ = n-1;
        beta_ = beta;
        D_ =D;
        m_ = p_+n_+1;
        u_ = Eigen::VectorXd::Zero(m_ + 1); //u0 ~ um 共m+1个
        control_points_ = Eigen::MatrixXd::Zero(n_+1,D_);
        for(int i = 0; i<=m_; i++)
        {
            u_(i) = i;
        }
        s_ini_ = s_ini;
        s_ter_ = s_ter;
        setIniTerMatrix();
        getAvailableSrange();
        getAvailableTrange();
        getInterval();
    }

    void UniformBspline::setIniTerMatrix()
    {
        A_ini.resize(3,3);
        A_ter.resize(3,3);
        A_ini << 1.0/6, 2.0/3, 1.0/6,
                        -1.0/2*beta_, 0.0*beta_, 1.0/2*beta_,
                        1.0*beta_*beta_,-2.0*beta_*beta_,1.0*beta_*beta_;
        A_ter<<1.0/6, 2.0/3, 1.0/6,
                        -1.0/2*beta_, 0.0*beta_, 1.0/2*beta_,
                        1.0*beta_*beta_,-2.0*beta_*beta_,1.0*beta_*beta_;
    }

    void UniformBspline::setControlPoints(const Eigen::MatrixXd &ctrl_points)
    {
        control_points_ = ctrl_points;
    }
    
    Eigen::MatrixXd UniformBspline::getTrajectory(const Eigen::VectorXd &t)
    {
        double u_probe;
        int t_size = t.size();
        Eigen::MatrixXd trajectory(t_size,D_);
        for (size_t i = 0; i < t_size; i++)
        {
            //map t(i) to uniform knot vector
            u_probe = t(i) * beta_ + u_(p_);
            trajectory.row(i) = singleDeboor(u_probe); 
        }
        return trajectory;
    }

     Eigen::Vector2d UniformBspline::singleDeboor(const double &u_probe)//the deboor's algorithm
     {  
        //bound the u_probe
        double u_probe_;
        int k;
        u_probe_ = min(max( u_(p_) , u_probe), u_(m_-p_));
        k = p_;
        while(true)
        {
            if(u_(k+1)>=u_probe_)
                break;
            k = k+1;
        }
        // t(t_ctt) is maped to knot segment u_k ~ u_{k+1}
        // there are at most p+1 basis functions N_k-p,p(u), N_k-p+1,p(u),..., N_k,p(u) non-zero on knot span [uk,uk+1)
        //the effective control points are P_k-p ~ P_k
        // since MATLAB index start from 1 instead of 0
        // The effective control points are
        double alpha;
        Eigen::MatrixXd d(p_+1,2);
        d = control_points_.block(k-p_,0,p_+1,2);// c++这里是从0行0列开始
        for (size_t i = 0; i < p_; i++)
        {
            for (size_t j = p_; j > i; j--)
            {
                alpha = (u_probe_ - u_(j+k-p_)) /(u_(j+1+k-i) - u_(j+k-p_)); 
                d.row(j) = (1 - alpha)*d.row(j-1) + alpha*d.row(j);
            }          
        }

            Eigen::Vector2d value;
            value = d.row(p_);
            return value;
     }

    void UniformBspline::getAvailableSrange()
    {
        s_range = {u_(p_),u_(m_-p_)};
    }

    void UniformBspline::getAvailableTrange()
    {
        t_range = {0/beta_, (u_(m_-p_)-u_(p_))/beta_};
    }

    void UniformBspline::getInterval()
    {
        interval_ = (u_(1) - u_(0))/beta_;
    }

    void UniformBspline::getT(const int &trajSampleRate)
    {
        time_.resize((t_range(1)-t_range(0))*trajSampleRate);
        for (size_t i = 0; i < time_.size(); i++)
        {
            time_(i) = t_range(0) + i*(1.0/trajSampleRate);
        }
    }

    UniformBspline UniformBspline::getDerivative()
    {     
            UniformBspline spline(p_,n_,beta_,D_,s_ini_,s_ter_);
            spline.p_ = spline.p_ -1;
            spline.m_ = spline.p_ +spline.n_ +1;
            spline.u_.resize(u_.size()-2);
            spline.u_ = u_.segment(1,m_-1);//从第2个元素开始的m-1个元素
            spline.control_points_.resize(control_points_.rows()-1,D_);
            for (size_t i = 0; i < spline.control_points_.rows(); i++)
            {
                spline.control_points_.row(i) = spline.beta_*(control_points_.row(i+1) - control_points_.row(i));
            } 
            spline.time_ = time_;
            return spline;
    }

    Eigen::VectorXd UniformBspline::getBoundConstraintb()
    {
        int nm = (n_+1)*D_;
        Eigen::VectorXd b= Eigen::VectorXd::Zero(nm);
        Eigen::MatrixXd tmp1(3,2);//前三个控制点的值
        Eigen::MatrixXd tmp2(3,2);//末尾三个控制点的值
        // solve Ax = b
        tmp1 = A_ini.colPivHouseholderQr().solve(s_ini_);
        tmp2 = A_ter.colPivHouseholderQr().solve(s_ter_);

         for (size_t j = 0; j< D_; j++)
        {
            for (size_t i = 0; i < 3; i++)
            {
                b(i+j*(n_+1)) = tmp1(i,j);
                b((j+1)*(n_+1)-i-1) = tmp2(3-i-1,j);
            }      
        }    
        return b;   
    }

/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/


    bspline_optimizer::bspline_optimizer(const std::vector<Eigen::Vector2d> &path, const int&Dim,  const int&p)
    {
        path_.clear();
        path_ = path;
        Dim_  = Dim;
        p_order_ = p;
        cps_num_ = path.size() + 2*p_order_;
    }

    bspline_optimizer::~bspline_optimizer(){}
    
    void bspline_optimizer::setOptParam(const double lambda1,const double lambda2,const double lambda3,
                                                                                    const double safe_dist)
    {
            lambda1_ = lambda1;
            lambda2_ = lambda2;
            lambda3_ = lambda3;
            safe_distance_ = safe_dist;
    }
    void bspline_optimizer::setVelAcc(const double vel, const double acc)
    {
            max_vel_ = vel;
            max_acc_ = acc;
    }

    void bspline_optimizer::setEsdfMap(const Eigen::MatrixXd &esdf_map)
    {
        esdf_map_ = esdf_map;
    }

    void bspline_optimizer::setMapParam(const double &origin_x,const double &origin_y, const double &map_resolution)
    {
        origin_x_ = origin_x;
        origin_y_ = origin_y;
        map_resolution_ = map_resolution;
    }

    void bspline_optimizer::setSplineParam(const UniformBspline &u)
    {
        u_ = u;
        bspline_interval_  = u.interval_;
        beta_ = u.beta_;
        control_points_.resize(cps_num_,Dim_);
        Eigen::VectorXd beq_bound = u_.getBoundConstraintb();
        
        for (size_t i = 0; i < Dim_; i++)
        {
                for (size_t j = 0; j < p_order_; j++)
                {
                     control_points_(j,i) = beq_bound(i*cps_num_+j);
                     control_points_((1)*cps_num_-j-1,i) = beq_bound((i+1)*cps_num_-j-1);//BUG!!!!
                }
        }
            
        for (size_t i = 0; i < cps_num_-2*p_order_; i++)
        {
            control_points_.row(i+p_order_) = path_[i];

        }
    }

    void bspline_optimizer::calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                                            Eigen::MatrixXd &gradient, bool falg_use_jerk /* = true*/)
    {
        cost = 0.0;
        if (falg_use_jerk)
        {
            Eigen::Vector2d jerk, temp_j;

            for (int i = 0; i < q.cols() - 3; i++)
            {
                /* evaluate jerk */
                jerk = q.col(i + 3) - 3 * q.col(i + 2) + 3 * q.col(i + 1) - q.col(i);
                cost += jerk.squaredNorm();
                temp_j = 2.0 * jerk;
                /* jerk gradient */
                gradient.col(i + 0) += -temp_j;
                gradient.col(i + 1) += 3.0 * temp_j;
                gradient.col(i + 2) += -3.0 * temp_j;
                gradient.col(i + 3) += temp_j;
            }
        }
        else
        {
            Eigen::Vector2d acc, temp_acc;

            for (int i = 0; i < q.cols() - 2; i++)
            {
                /* evaluate acc */
                acc = q.col(i + 2) - 2 * q.col(i + 1) + q.col(i);
                cost += acc.squaredNorm();
                temp_acc = 2.0 * acc;
                /* acc gradient */
                gradient.col(i + 0) += temp_acc;
                gradient.col(i + 1) += -2.0 * temp_acc;
                gradient.col(i + 2) += temp_acc;
            }
        }
    }
  void bspline_optimizer::calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                                                        Eigen::MatrixXd &gradient)
   {
    cost = 0.0;
    /* abbreviation */
    double ts, /*vm2, am2, */ ts_inv2;
    // vm2 = max_vel_ * max_vel_;
    // am2 = max_acc_ * max_acc_;

    ts = bspline_interval_;
    ts_inv2 = 1 / ts / ts;

    /* velocity feasibility */
    for (int i = 0; i < q.cols() - 1; i++)
    {
      Eigen::Vector2d vi = (q.col(i + 1) - q.col(i)) / ts;
      for (int j = 0; j < 2; j++)
      {
        if (vi(j) > max_vel_)
        {
          cost += pow(vi(j) - max_vel_, 2) * ts_inv2; // multiply ts_inv3 to make vel and acc has similar magnitude

          gradient(j, i + 0) += -2 * (vi(j) - max_vel_) / ts * ts_inv2;
          gradient(j, i + 1) += 2 * (vi(j) - max_vel_) / ts * ts_inv2;
        }
        else if (vi(j) < -max_vel_)
        {
          cost += pow(vi(j) + max_vel_, 2) * ts_inv2;

          gradient(j, i + 0) += -2 * (vi(j) + max_vel_) / ts * ts_inv2;
          gradient(j, i + 1) += 2 * (vi(j) + max_vel_) / ts * ts_inv2;
        }
        else
        {
          /* code */
        }
      }
    }

    /* acceleration feasibility */
    for (int i = 0; i < q.cols() - 2; i++)
    {
      Eigen::Vector2d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;

      for (int j = 0; j < 2; j++)
      {
        if (ai(j) > max_acc_)
        {
          cost += pow(ai(j) - max_acc_, 2);

          gradient(j, i + 0) += 2 * (ai(j) - max_acc_) * ts_inv2;
          gradient(j, i + 1) += -4 * (ai(j) - max_acc_) * ts_inv2;
          gradient(j, i + 2) += 2 * (ai(j) - max_acc_) * ts_inv2;
        }
        else if (ai(j) < -max_acc_)
        {
          cost += pow(ai(j) + max_acc_, 2);

          gradient(j, i + 0) += 2 * (ai(j) + max_acc_) * ts_inv2;
          gradient(j, i + 1) += -4 * (ai(j) + max_acc_) * ts_inv2;
          gradient(j, i + 2) += 2 * (ai(j) + max_acc_) * ts_inv2;
        }
        else
        {
          /* code */
        }
      }

    }
    }
    
    void bspline_optimizer::calcEsdfCost(const Eigen::MatrixXd &q, double &cost,
                                                        Eigen::MatrixXd &gradient)
    {
        cost = 0.0;
        double  dist;
        Eigen::Vector2d dist_grad;
        Eigen::Vector2d tmp_vel;

        for (int i = p_order_; i < q.cols()-p_order_; i++) 
        {
            dist = calcDistance(q.col(i));
            dist_grad = calcGrad(q.col(i));
            if (dist_grad.norm() > 1e-4) dist_grad.normalize();
            if (dist < safe_distance_) 
            {
                cost += pow(dist - safe_distance_, 2);
                gradient.col(i) += 2.0 * (dist - safe_distance_) * dist_grad;     
            }
        }   

    }
    double bspline_optimizer::calcDistance(const Eigen::MatrixXd &q)
    {
        double dist;
        Eigen::Vector2d p(q(0,0),q(1,0));//存入两个控制点
        startPoint_x = origin_x_;
        startPoint_y = -origin_y_;
        Eigen::Vector2d diff;
        Eigen::Vector2d sur_pts[2][2];//4个邻居点
        getSurroundPts(p,sur_pts, diff);
        double dists[2][2];
        getSurroundDistance(sur_pts, dists);
        interpolateBilinearDist(dists, diff, dist);
        return dist;
    }

    void bspline_optimizer::interpolateBilinearDist(double values[2][2], const Eigen::Vector2d& diff, 
                                                                                                                                                                double& dist)
    {
        //二线性插值
        double c00 = values[0][0];
        double c01 = values[0][1];
        double c10 = values[1][0];
        double c11 = values[1][1];
        double tx = diff(0);
        double ty = diff(1);
        
        double nx0 = lerp(c00,c10,ty);
        double nx1 = lerp(c01,c11,ty);
        double ny0 = lerp(c00,c01,tx);
        double ny1 = lerp(c10,c11,tx);

        dist = lerp(ny0,ny1,ty);
    }
    Eigen::Vector2d bspline_optimizer::calcGrad(const Eigen::MatrixXd &q)
    {
        Eigen::Vector2d p(q(0,0),q(1,0));//存入两个控制点
        Eigen::Vector2d dist_grad;
        Eigen::Vector2d diff;
        Eigen::Vector2d sur_pts[2][2];//4个邻居点
        getSurroundPts(p,sur_pts, diff);

        double dists[2][2];
        getSurroundDistance(sur_pts, dists);

        interpolateBilinear(dists, diff, dist_grad);

        return dist_grad;
    }

    void bspline_optimizer::getSurroundPts(const Eigen::Vector2d& pos, Eigen::Vector2d pts[2][2], Eigen::Vector2d& diff)
    {
        double dist_x = pos(0) - startPoint_x;
        double dist_y = startPoint_y - pos(1);
        diff(0) = fmod(dist_x,map_resolution_);
        diff(1) = fmod(dist_y,map_resolution_);

        Eigen::Vector2d curr_index;//用这个索引找到最左上角的点，并记为 p(0,0);
        curr_index<< floor(dist_y/map_resolution_),floor(dist_x/map_resolution_);
        for (size_t i = 0; i < 2; i++)
        {
            for (size_t j = 0; j < 2; j++)
            {       
                Eigen::Vector2d tmp_index(curr_index(0)+i,curr_index(1)+j);
                pts[i][j] = tmp_index;
            }
        }
    }
    void bspline_optimizer::getSurroundDistance(Eigen::Vector2d pts[2][2], double dists[2][2])
    {
        for (size_t i = 0; i < 2; i++)
        {
            for (size_t j = 0; j < 2; j++)
            {
                Eigen::Vector2d tmp_index = pts[i][j];
                dists[i][j] = esdf_map_(tmp_index(0),tmp_index(1));   
            }
        }
    }
    void bspline_optimizer::interpolateBilinear(double values[2][2], const Eigen::Vector2d& diff, Eigen::Vector2d& grad)
    {
        //二线性插值
        double c00 = values[0][0];
        double c01 = values[0][1];
        double c10 = values[1][0];
        double c11 = values[1][1];
        double tx = diff(0);
        double ty = diff(1);
        
        double nx0 = lerp(c00,c10,ty);
        double nx1 = lerp(c01,c11,ty);
        double ny0 = lerp(c00,c01,tx);
        double ny1 = lerp(c10,c11,tx);

        grad(0) = (nx1- nx0)/map_resolution_;
        grad(1) = (ny0- ny1)/map_resolution_;

        // grad(0) = (ny0- ny1)/map_resolution_;
        // grad(1) = (nx1- nx0)/map_resolution_;
    }

    void bspline_optimizer::combineCost( const std::vector<double>& x,Eigen::MatrixXd &grad,double &f_combine)
    {
        Eigen::MatrixXd control_points = control_points_.transpose();
        for (size_t j = 0; j < cps_num_-2*p_order_; j++)
            {
              for (size_t i = 0; i < Dim_; i++)
                {
                    control_points(i,j+p_order_) = x[j*Dim_+i];
                } 
            }
        f_combine = 0.0;
        Eigen::MatrixXd grad2D;
        grad2D.resize(Dim_,cps_num_);
        grad2D.setZero(Dim_,cps_num_);//初始化梯度矩阵

        double f_smoothness, f_length, f_distance, f_feasibility;
        Eigen::MatrixXd g_smoothness_ = Eigen::MatrixXd::Zero(Dim_, cps_num_);
        Eigen::MatrixXd g_feasibility_ = Eigen::MatrixXd::Zero(Dim_, cps_num_);
        Eigen::MatrixXd g_distance_ = Eigen::MatrixXd::Zero(Dim_, cps_num_);
        f_smoothness  = f_feasibility =  f_distance = 0.0;

        calcSmoothnessCost(control_points, f_smoothness, g_smoothness_);
        calcFeasibilityCost(control_points,f_feasibility,g_feasibility_);
        calcEsdfCost(control_points,f_distance,g_distance_);

        f_combine = lambda1_ * f_smoothness + lambda2_*f_feasibility + lambda3_*f_distance;
        grad2D = lambda1_*g_smoothness_ + lambda2_ * g_feasibility_ +lambda3_ * g_distance_ ;
        grad = grad2D.block(0,p_order_,Dim_,cps_num_-2*p_order_);//起点  块大小
        cout<<"f_combine is "<< f_combine<< endl;
        cout<<"f_smoothness is "<< f_smoothness<< endl;
        cout<<"f_feasibility is "<< f_feasibility<< endl;
        cout<<"f_distance is "<< f_distance<< endl;
    }

    double bspline_optimizer::costFunction(const std::vector<double>& x, std::vector<double>& grad,
                                                                                         void* func_data)
    {
        bspline_optimizer* opt = reinterpret_cast<bspline_optimizer*>(func_data);
        Eigen::MatrixXd grad_matrix;
        double cost;
        opt->combineCost(x,grad_matrix,cost);
        opt->iter_num_++;

        for (size_t i = 0; i < grad_matrix.cols(); i++)
            {
                for (size_t j = 0; j <opt->Dim_; j++)
                {
                    // grad.push_back(grad_matrix(j,i)) ;
                    grad[i*opt->Dim_+j] = grad_matrix(j,i);
                }    
            } 
        /* save the min cost result */
        if (cost < opt->min_cost_) {
                opt->min_cost_     = cost;
                opt->best_variable_ = x;
            }
        return cost;
    }

     void bspline_optimizer::optimize()
    {
            /* initialize solver */
            iter_num_        = 0;
            min_cost_        = std::numeric_limits<double>::max();
            variable_num = (cps_num_-2*p_order_)*Dim_;
            nlopt::opt opt(nlopt::algorithm(nlopt::LD_LBFGS),variable_num);
            opt.set_min_objective(bspline_optimizer::costFunction,this);
            opt.set_maxeval(200);
            opt.set_maxtime(0.02);
            opt.set_xtol_rel(1e-5);
            vector<double> lb(variable_num), ub(variable_num);
            vector<double> q(variable_num);

            for (size_t j = 0; j < cps_num_-2*p_order_; j++)
            {
               for (size_t i = 0; i < Dim_; i++)
                {
                    q[j*Dim_+i] = control_points_(j+p_order_,i);
                }
            }

            const double  bound = 10.0;
            for (size_t i = 0; i <variable_num; i++)
            {
                    lb[i]  = q[i]-bound;
                    ub[i] = q[i]+bound;      
            }
            opt.set_lower_bounds(lb);
            opt.set_upper_bounds(ub);
        try
        {
            double final_cost;
            nlopt::result result = opt.optimize(q, final_cost);    
        }
        catch(std::exception &e)
         {
            std::cout << "nlopt failed: " << e.what() << std::endl;
        }

          for (size_t j = 0; j < cps_num_-2*p_order_; j++)
            {
              for (size_t i = 0; i < Dim_; i++)
                {
                    control_points_(j+p_order_,i) = best_variable_[j*Dim_+i];
                } 
            }
            cout<< "optimize successfully~"<<endl;
            cout<<"iter num :"<<iter_num_<<endl;
    }

    void bspline_optimizer::optimize_withoutesdf()
    {
           double intial_lambda3 = lambda3_;
            lambda3_  = 0;
            /* initialize solver */
            iter_num_        = 0;
            min_cost_        = std::numeric_limits<double>::max();
            variable_num = (cps_num_-2*p_order_)*Dim_;
            nlopt::opt opt(nlopt::algorithm(nlopt::LD_LBFGS),variable_num);
            opt.set_min_objective(bspline_optimizer::costFunction,this);
            opt.set_maxeval(200);
            opt.set_maxtime(0.02);
            opt.set_xtol_rel(1e-5);
            vector<double> lb(variable_num), ub(variable_num);
            vector<double> q(variable_num);

            for (size_t j = 0; j < cps_num_-2*p_order_; j++)
            {
               for (size_t i = 0; i < Dim_; i++)
                {
                    q[j*Dim_+i] = control_points_(j+p_order_,i);
                }
            }

            const double  bound = 10.0;
            for (size_t i = 0; i <variable_num; i++)
            {
                    lb[i]  = q[i]-bound;
                    ub[i] = q[i]+bound;      
            }
            opt.set_lower_bounds(lb);
            opt.set_upper_bounds(ub);
        try
        {
            double final_cost;
            nlopt::result result = opt.optimize(q, final_cost);    
        }
        catch(std::exception &e)
         {
            std::cout << "nlopt failed: " << e.what() << std::endl;
        }

          for (size_t j = 0; j < cps_num_-2*p_order_; j++)
            {
              for (size_t i = 0; i < Dim_; i++)
                {
                    control_points_(j+p_order_,i) = best_variable_[j*Dim_+i];
                } 
            }
            lambda3_  = intial_lambda3;
    }



/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
   //只用于测试
    plan_manager::plan_manager(ros::NodeHandle &nh)
    {
        setParam(nh);
        TrajPlanning(nh);
    }
    plan_manager::~plan_manager() {}

    void plan_manager::setParam(ros::NodeHandle &nh)
    {
        nh.param("planning/traj_order", p_order_, 3);
        nh.param("planning/dimension", Dim_, -1);
        nh.param("planning/TrajSampleRate", TrajSampleRate, -1);
        nh.param("planning/beta", beta, -1.0);
        nh.param("planning/max_vel", max_vel_, -1.0);
        nh.param("planning/max_acc", max_acc_, -1.0);
        nh.param("planning/start_x", start_x_, -1.0);
        nh.param("planning/start_y", start_y_, -1.0);
        nh.param("planning/goal_x", goal_x_, -1.0);
        nh.param("planning/goal_y", goal_y_, -1.0);
        nh.param("planning/lambda1",lambda1_,-1.0);
        nh.param("planning/lambda2",lambda2_,-1.0);
        nh.param("planning/lambda3",lambda3_,-1.0);
        nh.param("planning/frame",frame_,std::string("odom"));
        nh.param("planning/map_resolution",map_resolution_,-1.0);
        nh.param("planning/origin_x",origin_x_,-1.0);
        nh.param("planning/origin_y",origin_y_,-1.0);
        nh.param("planning/safe_distance",safe_distance_,-1.0);
    }

    void plan_manager::TrajPlanning(ros::NodeHandle &nh)
    {
        //goal_suber = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, & plan_manager::uav_goal_subCallback, this);
        
        //订阅地图
        map_suber = nh.subscribe<std_msgs::Float64MultiArray>("/ESDFmsgs",1,&plan_manager::esdf_map_subCallback,this);
        
        //订阅路径
        path_suber = nh.subscribe<nav_msgs::Path>("/astar_node/grid_twist",1, &plan_manager::astar_subCallback,this);

        //发布轨迹
        Traj_puber = nh.advertise<bspline_race::BsplineTraj>("/bspline_traj", 10);

        //可视化执行的轨迹   
        Traj_vis = nh.advertise<nav_msgs::Path>("/traj_vis", 10);
        Traj_vis1 = nh.advertise<nav_msgs::Path>("/traj_smooth", 10);

        //可视化地图
        Map_puber = nh.advertise<visualization_msgs::Marker>("/esdfmap_slice", 10);

    }

    void plan_manager::uav_goal_subCallback(const geometry_msgs::PoseStampedConstPtr &goal_msg)
    {
        terminal_state(0,0) = goal_msg->pose.position.x;
        terminal_state(0,1) = goal_msg->pose.position.y; 
    }

    void plan_manager::esdf_map_subCallback(const std_msgs::Float64MultiArrayConstPtr &map_msg)
    {
        cout<< "get grid map"<<endl;
        get_map = true;
        map_size_x = map_msg->data[0];
        map_size_y = map_msg->data[1];
        map_size = map_size_x*map_size_y;
        grid_map_.resize(map_size_x,map_size_y);
        esdf_map_.resize(map_size_x,map_size_y);
        double *src,*dst;
        src=(double*)malloc(map_size*sizeof(double));
        dst=(double*)malloc(map_size*sizeof(double));

        for (size_t i = 0; i < map_size_x; i++)
        {
            for (size_t j = 0; j < map_size_y; j++)
            {
                grid_map_(map_size_x-j-1,i) = map_msg->data[i*map_size_x+j+2];
            }
        }

        for (size_t i = 0; i < map_size_x; i++)
        {
            for (size_t j = 0; j < map_size_y; j++)
            {
                //把grid_map里的数据送入src指针
                *(src+i*esdf_map_.rows()+j) = grid_map_(i,j);
            }
        }

        computeEDT(dst,src,map_size_x,map_size_y);

        for (size_t i = 0; i < map_size_x; i++)
        {
            for (size_t j = 0; j < map_size_y; j++)
            {
                esdf_map_(i,j) = (int)sqrt(*(dst+i*esdf_map_.rows()+j));
            }
        }      
        map_slice_output(esdf_map_);
        // map_slice_output(grid_map_);
        free(dst);
        free(src);
        dst = NULL;
        src = NULL;

    }

    void plan_manager::astar_subCallback(const nav_msgs::PathConstPtr &path)
    {
        astar_path_.clear();
        cout<< "get A star path"<<endl;
        get_path = true;
        //读取Astar
        Eigen::Vector2d tmp_point;
        for (size_t i = 0; i < path->poses.size(); i++)
        {
            tmp_point<< path->poses[i].pose.position.x,path->poses[i].pose.position.y;
            //A star路径是正的
            astar_path_.push_back(tmp_point);
        }
        
        //读取首末位置
        Eigen::Vector2d start_point,end_point;
        start_point = *astar_path_.begin(); 
        end_point = *(astar_path_.end()-1);        
        initial_state.resize(3,2);
        terminal_state.resize(3,2);
        initial_state<< start_point(0),start_point(1),
	                                    0.0, 0.0, 
		                                0.0, 0.0;
        terminal_state <<    end_point(0), end_point(1),
		                                        0.0, 0.0,
	                                            0.0, 0.0;
        
        //初始化优化类和 b样条轨迹类
        opt.reset(new bspline_optimizer(astar_path_,Dim_,p_order_));
        u.reset(new UniformBspline(p_order_,opt->cps_num_,Dim_,beta,initial_state,terminal_state));

        opt1.reset(new bspline_optimizer(astar_path_,Dim_,p_order_));
        u1.reset(new UniformBspline(p_order_,opt->cps_num_,Dim_,beta,initial_state,terminal_state));
          if(get_map)
        {
            UniformBspline spline = *u;
            opt->setEsdfMap(esdf_map_) ;
            opt->setOptParam(lambda1_,lambda2_,lambda3_,safe_distance_);
            opt->setMapParam(origin_x_,origin_y_,map_resolution_);
            opt->setVelAcc(max_vel_,max_acc_);
            opt->setSplineParam(spline);
            opt->optimize();

            //计算轨迹
            u->setControlPoints(opt->control_points_);
            
            // cout<<"ctrl_pt:"<<opt->control_points_<<endl;//FIXME 
            u->getT(TrajSampleRate);
            UniformBspline p = *u;
            UniformBspline v = p.getDerivative();
            UniformBspline a = v.getDerivative();

            //生成轨迹
            geometry_msgs::PoseStamped tmp_p,tmp_v,tmp_a;
            geometry_msgs::PoseStamped tmp_vis;
            p_ = p.getTrajectory(p.time_);
            v_ = v.getTrajectory(p.time_);
            a_ = a.getTrajectory(p.time_);
            cout<<"-----------------------------p_-----------------------------------"<<endl;
            // for(int i = 0;i<p_.rows();i++){
            //     for(int j = 0;j<p_.cols();j++){
            //         cout<<p_(i,j)<<endl;
            //     }
            // }
            cout<<p_<<endl;
            cout<<"-----------------------------v_-----------------------------------"<<endl;
            // for(int i = 0;i<v_.rows();i++){
            //     for(int j = 0;j<v_.cols();j++){
            //         cout<<v_(i,j)<<endl;
            //     }
            // }
            cout<<v_<<endl;
            cout<<"------------------------------a_----------------------------------"<<endl;
            // for(int i = 0;i<a_.rows();i++){
            //     for(int j = 0;j<a_.cols();j++){
            //         cout<<a_(i,j)<<endl;
            //     }
            // }
            cout<<a_<<endl;
            traj.position.clear();
            traj.velocity.clear();
            traj.acceleration.clear();
            traj_vis.poses.clear();
            
            for (size_t i = 0; i < p_.rows(); i++)
            {
                int count = p_.rows()-i-1;
                //
                tmp_p.pose.position.x = p_(count,0);tmp_p.pose.position.y =  p_(count,1);tmp_p.pose.position.z = 0;
                tmp_v.pose.position.x = v_(count,0); tmp_v.pose.position.y = v_(count,0);tmp_v.pose.position.z = 0;
                tmp_a.pose.position.x = a_(count,0); tmp_a.pose.position.y = a_(count,1); tmp_a.pose.position.z = 0; 
                tmp_vis.pose.position.x = p_(count,0);tmp_vis.pose.position.y = p_(count,1);tmp_vis.pose.position.z = 0.5;
               
               traj.position.push_back(tmp_p) ;
               traj.velocity.push_back(tmp_v) ;
               traj.acceleration.push_back(tmp_a);
               traj_vis.poses.push_back(tmp_vis);
               traj.header.frame_id = frame_;
               traj_vis.header.frame_id = frame_;
               
            }
        
            UniformBspline spline1 = *u1;
            opt1->setEsdfMap(esdf_map_) ;
            opt1->setOptParam(lambda1_,lambda2_,0.0,safe_distance_);
            opt1->setMapParam(origin_x_,origin_y_,map_resolution_);
            opt1->setVelAcc(max_vel_,max_acc_);
            opt1->setSplineParam(spline);
            opt1->optimize();
            
            //计算轨迹
            u1->setControlPoints(opt1->control_points_);
            u1->getT(TrajSampleRate);
            UniformBspline p1 = *u1;

            //生成纯smooth轨迹，不检测碰撞
            Eigen::MatrixXd p1_;
            geometry_msgs::PoseStamped tmp_vis1;
            p1_ = p1.getTrajectory(p1.time_);
            traj_vis1.poses.clear();
            
            for (size_t i = 0; i < p1_.rows(); i++)
            {
               int count = p1_.rows()-i-1;
               tmp_vis1.pose.position.x = p1_(count,0);tmp_vis1.pose.position.y = p1_(count,1);tmp_vis1.pose.position.z = 0.5;
               traj_vis1.poses.push_back(tmp_vis1);
               traj_vis1.header.frame_id = frame_;
            }

            //发布可视化轨迹（esdf  without esdf）
            Traj_vis.publish(traj_vis);
            Traj_vis1.publish(traj_vis1);

            //发布期望轨迹
            Traj_puber.publish(traj);
            // get_map = false;
        }
        else
        {
            cout<<"no path no path no path"<<endl;
        }
    }



     /*************************************************
 * 测试函数：
 *      输入：
 *          Eigen::MatrixXd
 *      输出：
 *          slice的marker
 *************************************************/
void plan_manager::map_slice_output(const Eigen::MatrixXd &esdf_matrix)
{
    visualization_msgs::Marker marker_result;
    marker_result.header.frame_id = "/t265_odom_frame";
    marker_result.type = visualization_msgs::Marker::POINTS;
    marker_result.action = visualization_msgs::Marker::MODIFY;
    marker_result.scale.x = 0.1;
    marker_result.scale.y = 0.1;
    marker_result.scale.z = 0.1;
    marker_result.pose.orientation.x = 0;
    marker_result.pose.orientation.y = 0;
    marker_result.pose.orientation.z = 0;
    marker_result.pose.orientation.w = 1;

    /* 遍历矩阵的所有元素 */
    for (int i = 0; i < esdf_matrix.rows(); i++)
    {
        for (int j = 0; j < esdf_matrix.cols(); j++)
        {
            double h = esdf_matrix(i, j);
            double max_dist = 20.0;
            if (h < 0 || h > 20.0) continue;
            /* 计算当前栅格中心的真实坐标 */
            double vox_pos_x, vox_pos_y;
            vox_pos_x = (j+0.5)*0.1 - 20;
            vox_pos_y = 20 - (i+0.5)*0.1;
            geometry_msgs::Point pt;
            pt.x = vox_pos_x;
            pt.y = vox_pos_y;
            pt.z = 0.3;
            marker_result.points.push_back(pt);

            /* 计算色彩 */
            std_msgs::ColorRGBA color;
            color.a = 1;
            if (h <= max_dist)
            {
                h = h/max_dist;
            }
            else
            {
                h = 1;
            }
            /* --- 以下复制自fiesta中RainbowColorMap函数 --- */
            double s = 1.0;
            double v = 1.0;
            h -= floor(h);
            h *= 6;
            int ii;
            double m, n, f;
            ii = floor(h);
            f = h - ii;
            if (!(ii & 1))
            f = 1 - f;  // if i is even
            m = v * (1 - s);
            n = v * (1 - s * f);
            switch (ii)
            {
                case 6:
                case 0:color.r = v;
                    color.g = n;
                    color.b = m;
                    break;
                case 1:color.r = n;
                    color.g = v;
                    color.b = m;
                    break;
                case 2:color.r = m;
                    color.g = v;
                    color.b = n;
                    break;
                case 3:color.r = m;
                    color.g = n;
                    color.b = v;
                    break;
                case 4:color.r = n;
                    color.g = m;
                    color.b = v;
                    break;
                case 5:color.r = v;
                    color.g = m;
                    color.b = n;
                    break;
                default:color.r = 1;
                    color.g = 0.5;
                    color.b = 0.5;
                    break;
            }
            marker_result.colors.push_back(color);
        }
    }

    Map_puber.publish(marker_result);
}

bool plan_manager::checkTrajCollision()
{   
    traj_state state;//判断算出来的轨迹是否安全
    state == SAFE;
    Eigen::Vector2i tmp_index;//
    for (size_t i = 0; i < p_.rows(); i++)
    {
        tmp_index = posToIndex(p_.row(i));
        if(grid_map_(tmp_index(0),tmp_index(1))==1)
        {
            state = COLLIDE;
            break;
        }
    }
    
    if(state == COLLIDE)
    {
        return true;//会撞
    }
        
    else
        return false;//不会撞
}

}