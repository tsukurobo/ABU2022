#ifndef NDT_ORIGINAL_H
#define NDT_ORIGINAL_H

#include <fstream>
#include <list>
#include <vector>
#include <nav_msgs/OccupancyGrid.h>
#include "cmatrix.h"

namespace NDT
{
    using namespace custom_math;

    //OccpancyGrid型のメッセージを読んで、NDTGridデータをファイルに書き出す関数
    bool createAndExportNDT(boost::shared_ptr<nav_msgs::OccupancyGrid const> map_p, char* file_path, int npix_per_grid)
    {
        //書き込むファイルを開く
        std::ofstream ofs(file_path);
        //横方向と縦方向の分割数を求める
        //注意：npix_per_gridは、マップの縦横のピクセル数の約数でなければならない
        int map_width = map_p->info.width;
        int map_height = map_p->info.height;
        int ndiv_w = map_width/npix_per_grid;
        int ndiv_h = map_height/npix_per_grid;
        //マップの解像度
        double map_resol = (double)map_p->info.resolution;
        //グリッドの配列を用意
        std::list<Matrix> grids[ndiv_w*ndiv_h];
        //lambda関数
        auto calcAverage = [](std::list<Matrix> pos_list) -> Matrix
                        {
                            Matrix ave(2, 1);
                            if(pos_list.size() > 0)
                            {
                                for(std::list<Matrix>::iterator itr = pos_list.begin(); itr != pos_list.end(); itr++) 
                                {
                                    ave += *itr;
                                }
                                ave *= (1.0/pos_list.size());
                            }
                            return ave;
                        };
        auto calcCovariance = [](std::list<Matrix> pos_list, Matrix ave) -> Matrix
                        {
                            Matrix cov(2, 2);
                            if(pos_list.size() > 0)
                            {
                                for(std::list<Matrix>::iterator itr = pos_list.begin(); itr != pos_list.end(); itr++)
                                {
                                    cov += (*itr - ave)*((*itr - ave).T());
                                }
                                cov *= (1.0/pos_list.size());
                            }
                            return cov;
                        };
        /*auto isZero = [](double x) -> bool
                        {
                            return (-1.0e-8 < x) && (x < 1.0e-8);
                        };*/


        if(ofs)
        {
            Matrix::allocTmpSpace(2, 2);
            /*以下、NDTの作成*/
            for(int i=0; i<ndiv_w*ndiv_h; i++)
            {
                //マップをグリッドで区切ったとき、これからどこのグリッドに対して処理を行うのか求める
                int grid_pos_h = i/ndiv_w;
                int grid_pos_w = i%ndiv_w;
                //std::cout << "h: " << grid_pos_h << ", w: " << grid_pos_w << std::endl;
                for(int j=0; j<npix_per_grid*npix_per_grid; j++)
                {
                    int grid_internal_pos_h = j/npix_per_grid;
                    int grid_internal_pos_w = j%npix_per_grid;
                    
                    //壁と認識されたピクセルに対し、マップ座標系における座標を計算し、
                    //対応するリストに座標値を格納
                    //std::cout << (grid_pos_h + grid_internal_pos_h)*map_width + grid_pos_w*npix_per_grid + grid_internal_pos_w << std::endl;
                    if(map_p->data[(grid_pos_h*npix_per_grid + grid_internal_pos_h)*map_width + grid_pos_w*npix_per_grid + grid_internal_pos_w] >= 60)
                    {
                        double x = (grid_pos_w*npix_per_grid + grid_internal_pos_w)*map_resol + map_resol*0.5;
                        double y = (grid_pos_h*npix_per_grid + grid_internal_pos_h)*map_resol + map_resol*0.5;
                        Matrix p_map(2, 1);
                        
                        p_map(0, 0) = x; p_map(1, 0) = y;
                        grids[i].push_back(p_map);

                        std::cout << "p(" << x << ", " << y  << ") is pushed in grids: " << i << std::endl;
                    }
                }
            }

            //各グリッドの平均・共分散行列を計算し、ファイルに書き出す
            for(int i=0; i<ndiv_w*ndiv_h; i++)
            {
                Matrix average = calcAverage(grids[i]);
                Matrix covariance = calcCovariance(grids[i], average);

                std::cout << "grid: " << i << std::endl;
                std::cout << "average:" << std::endl;
                showMatrix(average);
                std::cout << "covariance:" << std::endl;
                showMatrix(covariance);
                std::cout << std::endl;
                
                Matrix inv_covariance(2, 2);
                //共分散行列が逆行列を持たない場合、行列を少し加工する
                if(isZero(covariance(0, 0))==false || isZero(covariance(0, 1))==false ||
                   isZero(covariance(1, 0))==false || isZero(covariance(1, 1))==false)
                {
                    Matrix eigen_vals = calcEigenVals2x2(covariance);
                    if(fabs(eigen_vals(0, 0)) < fabs(eigen_vals(1, 0))*0.2)
                    {
                        covariance += Matrix::identity(2)*eigen_vals(1, 0)*0.2;
                    } 
                    else if(fabs(eigen_vals(1, 0)) < fabs(eigen_vals(0, 0))*0.2)
                    {
                        covariance += Matrix::identity(2)*eigen_vals(0, 0)*0.2;
                    }
                    //共分散行列の逆行列を計算
                    inv_covariance = calcInverse2x2(covariance);
                }
                //ファイルへの書き出し
                ofs << average(0, 0) << " " << average(1, 0) << " "
                 << inv_covariance(0, 0) << " " << inv_covariance(1, 1) << " " << inv_covariance(0, 1) << std::endl;
                //ofs << average(0, 0) << " " << average(1, 0) << " "
                 //<< covariance(0, 0) << " " << covariance(1, 1) << " " << covariance(0, 1) << std::endl;
            }

            ofs.close();
            Matrix::freeTmpSpace();

            return true;
        }
        else
        {
            return false;
        }
    }


    /*NDTGrid構造体の定義と実装*/
    struct NDTGrid
    {
    public:
        Matrix mean_;
        Matrix cov_inv_;

        NDTGrid()
        : mean_(2, 1), cov_inv_(2, 2)
        {
            mean_(0, 0) = 0; 
            mean_(1, 0) = 0;
            
            cov_inv_(0, 0) = 0;  cov_inv_(0, 1) = 0;
            cov_inv_(1, 0) = 0;  cov_inv_(1, 1) = 0;
        }
        
        NDTGrid(double mean_x, double mean_y, double sig_xx, double sig_yy, double sig_xy)
        : mean_(2, 1), cov_inv_(2, 2)
        {
            mean_(0, 0) = mean_x; 
            mean_(1, 0) = mean_y;
            
            cov_inv_(0, 0) = sig_xx;  cov_inv_(0, 1) = sig_xy;
            cov_inv_(1, 0) = sig_xy;  cov_inv_(1, 1) = sig_yy;
        }
    };

    /*Point2D構造体の定義と実装*/
    struct Point2D
    {
    public:
        Matrix pos_;
        Matrix ndt_mean_;  //その点が属するグリッドの平均
        Matrix ndt_cov_inv_;  //その点が属するグリッドの共分散

        Point2D()
        : pos_(2, 1), ndt_mean_(2, 1), ndt_cov_inv_(2, 2)
        {}
    };

    /*NDTScanMatcherクラスの定義と実装*/
    class NDTScanMatcher
    {
    private:
        std::vector<NDTGrid> grids_;
        std::vector<Point2D> points_map_;
        double itr_eps_ = 0;
        double grid_size_ = 0;
        int itr_n_ = 0;
        int ngrids_col_ = 0;

        void determineCorrespondingGrid(const std::vector<Matrix> &, const Matrix &);
        Matrix gradS(const std::vector<Matrix> &, const Matrix &);
        double score();

    public:
        NDTScanMatcher(int);
        void setCriteria(double, int);
        void setGridAndPointInfo(int, int, double);
        void addGrid(int, double, double, double, double, double);
        void showGrid();
        void align(const std::vector<Matrix> &, Matrix &);
    };

    NDTScanMatcher::NDTScanMatcher(int ngrids)
    : grids_(ngrids)
    {}

    void NDTScanMatcher::setCriteria(double eps, int itr_n)
    {
        itr_eps_ = eps;
        itr_n_ = itr_n;
    }

    void NDTScanMatcher::setGridAndPointInfo(int point_n, int ngrids_col, double grid_size)
    {
        points_map_.resize(point_n);
        ngrids_col_ = ngrids_col;
        grid_size_ = grid_size;
    }

    void NDTScanMatcher::addGrid(int i, double mean_x, double mean_y, double sig_x, double sig_y, double sig_xy)
    {
        NDTGrid grid(mean_x, mean_y, sig_x, sig_y, sig_xy);
        grids_[i] = grid;
    }

    void NDTScanMatcher::showGrid()
    {
        for(int i = 0; i < grids_.size(); i++)
        {
            std::cout << i << std::endl;
            showMatrix(grids_[i].mean_);
        }
    }

    void NDTScanMatcher::align(const std::vector<Matrix> &points_odom, Matrix &odom_to_map)
    {
        //注意：points_odomとpoints_map_の要素数は等しくなるようにすること
        Matrix grad_s(3, 1), grad_s_pre(3, 1), delta_p(3, 1),
                delta_grad(3, 1), odom_to_map_new(3, 1), 
                H = Matrix::identity(3),
                I = Matrix::identity(3);
                //alpha = Matrix::identity(3);
        try
        {
            //スキャンデータをマップ座標系に変換し、対応するグリッドを求める
            //std::cout << "A" << std::endl;
            determineCorrespondingGrid(points_odom, odom_to_map);
            
            // for(int i=0; i<points_map_.size(); i++)
            // {
            //     if(points_map_[i].ndt_mean_(0,0) != 0)
            //     {
            //         showMatrix(points_map_[i].ndt_mean_);
            //         std::cout << std::endl;
            //     }
            // }
            //showMatrix(points_map_[0].ndt_mean_);
            
            //std::cout << "B" << std::endl;
            grad_s_pre = gradS(points_odom, odom_to_map);

            int criteria_counter = 1;
            //std::cout << "C" << std::endl;
            while(1)
            {
                //パラメータ(マップ座標系とオドメトリ座標系の位置関係)の変更分を求める
                delta_p = H*grad_s_pre*(-1.0);
                
                double norm_delta_p = sqrt((delta_p.T()*delta_p)(0, 0));
                if( norm_delta_p > 1.0)
                {
                    delta_p *= 1.0/norm_delta_p;
                    //odom_to_map_new = odom_to_map + delta_p;
                }

                //showMatrix(H);
                //std::cout << std::endl;
                //パラメータの変化分を評価値が減少するように調節する
                double alpha = 0.004/*0.001*/, score_now, score_new, score_delta;
                score_now = score();
                //std::cout << std::endl;
                //std::cout << criteria_counter << std::endl;
                for(int i=0; i<10; i++)
                {
                    odom_to_map_new = odom_to_map + delta_p*alpha;
                    //odom_to_map_new = odom_to_map + alpha*delta_p;
                    determineCorrespondingGrid(points_odom, odom_to_map_new);
                    score_new = score();
                    //std::cout << score_new << std::endl;
                    score_delta = alpha*(grad_s_pre.T()*delta_p)(0, 0);
                    //score_delta = (grad_s_pre.T()*alpha*delta_p)(0, 0);
                    if(score_new < score_now + score_delta) break;
                    else
                    {
                        alpha *= 0.9;
                        //alpha *= 0.9;
                        //alpha *= 0.3;
                        //alpha *= 0.1;
                        // alpha(0, 0) *= 0.5;
                        // alpha(1, 1) *= 0.5;
                        // alpha(2, 2) *= 0.5;
                    }
                }

                //パラメータの更新
                //odom_to_map += delta_p;
                odom_to_map = odom_to_map_new;
                
                //更新されたパラメータに対し、再びgradを計算
                //もしかすると、determineCorrespondingGrid()は要らないかもしれない
                //determineCorrespondingGrid(points_odom, odom_to_map);
                grad_s = gradS(points_odom, odom_to_map);
                
                //gradの変化量が一定値以下になったor繰り返し回数が上限に達した場合は計算終了
                delta_grad = grad_s - grad_s_pre;
                if((delta_grad.T()*delta_grad)(0, 0) < itr_eps_ || criteria_counter == itr_n_)
                    break;
                criteria_counter++;
                
                //ヘシアンの近似逆行列の更新
                // if(isZero(delta_grad(0, 0))==false || isZero(delta_grad(1, 0))==false || isZero(delta_grad(2, 0))==false)
                // {
                //     double inv_y_Txs = 1.0/(delta_grad.T()*delta_p)(0,0);
                //     if(!isZero((delta_grad.T()*delta_p)(0,0)))
                //     {
                //         H = ( I - delta_p*delta_grad.T()*inv_y_Txs)
                //             * H
                //             * ( I - delta_grad*delta_p.T()*inv_y_Txs )
                //             + delta_p*delta_p.T()*inv_y_Txs*alpha;
                //     }
                // }

                grad_s_pre = grad_s;
            }
        }
        catch(Matrix::MatrixCalcException& e)
        {
            std::cerr << e.what() << '\n';
        }
    }

    void NDTScanMatcher::determineCorrespondingGrid(const std::vector<Matrix> &points_odom, const Matrix &odom_to_map)
    {
        //std::cout << "D" << std::endl;
        double inv_grid_size = 1.0/grid_size_;
        Matrix R_map = Matrix::rotation2x2(odom_to_map(2, 0));
        Matrix trans_odom_to_map(2, 1), point_map(2, 1);
        trans_odom_to_map(0, 0) = odom_to_map(0, 0);
        trans_odom_to_map(1, 0) = odom_to_map(1, 0);    

        for(int i=0; i<points_odom.size(); i++)
        {
            //std::cout << "E" << std::endl;
            //スキャンデータをマップ座標系に変換(マップ座標系とグリッド座標系は一致しているものとする)
            point_map = R_map*points_odom[i] + trans_odom_to_map;
            points_map_[i].pos_ = point_map;

            //std::cout << "F" << std::endl;
            //対応しているグリッドを求める
            int grid_index = int(point_map(0, 0)*inv_grid_size) + int(point_map(1, 0)*inv_grid_size)*ngrids_col_;
            //std::cout << grid_index << " ";
            if( (0 < grid_index && grid_index < grids_.size()) &&
                (isZero(grids_[grid_index].mean_(0, 0)) == false || isZero(grids_[grid_index].mean_(1, 0)) == false) )
            {
                //std::cout << "G" << std::endl;
                points_map_[i].ndt_mean_ = grids_[grid_index].mean_;
                points_map_[i].ndt_cov_inv_ = grids_[grid_index].cov_inv_;
            }
            else
            {
                //std::cout << "H" << std::endl;
                points_map_[i].ndt_mean_ = Matrix::zero(2, 1);
                points_map_[i].ndt_cov_inv_ = Matrix::zero(2, 2);
            }
        }
        //showMatrix(points_map_[0].pos_);
        //std::cout << std::endl;
    }

    Matrix NDTScanMatcher::gradS(const std::vector<Matrix> &points_odom, const Matrix &odom_to_map)
    {
        //R_map_d -> オドメトリ座標系をマップ座標系に直すときに使う回転行列を、
        //マップ座標系に対するオドメトリ座標系の回転角度で偏微分したもの
        //p_map_d -> スキャンデータをマップ座標系に変換したものを、
        //マップ座標系に対するオドメトリ座標系の回転角度で偏微分したもの
        Matrix R_map_d(2, 2), /*trans_odom_to_map(2, 1), */p_map_d(2, 1),
            tmp_mat(3, 2), delta_p(2, 1), result(3, 1);

        double sin_theta_map = sin(odom_to_map(2, 0));
        double cos_theta_map = cos(odom_to_map(2, 0));
        R_map_d(0, 0) = -sin_theta_map;  R_map_d(0, 1) = -cos_theta_map;
        R_map_d(1, 0) = cos_theta_map;  R_map_d(1, 1) = -sin_theta_map;
        //trans_odom_to_map(0, 0) = odom_to_map(0, 0);  trans_odom_to_map(1, 0) = odom_to_map(1, 0);
        tmp_mat(0,0) = 1;  tmp_mat(1, 1) = 1;

        for(int i=0; i<points_odom.size(); i++)
        {
            if(isZero(points_map_[i].ndt_mean_(0, 0)) == false || isZero(points_map_[i].ndt_mean_(1, 0)) == false)
            {
                p_map_d = R_map_d*points_odom[i];// + trans_odom_to_map;
                tmp_mat(2, 0) = p_map_d(0, 0);
                tmp_mat(2, 1) = p_map_d(1, 0);
                delta_p = points_map_[i].pos_ - points_map_[i].ndt_mean_;

                double exp_val = -0.5*(delta_p.T()*points_map_[i].ndt_cov_inv_*delta_p)(0, 0);
                result += tmp_mat
                    *(delta_p.T()*points_map_[i].ndt_cov_inv_).T()
                    *exp(exp_val);

                //showMatrix(result);
                //std::cout << std::endl;
            }
        }

        return result;
    }

    double NDTScanMatcher::score()
    {
        double score_sum = 0, exp_val;
        Matrix delta_p(2, 1);

        for(int i=0; i<points_map_.size(); i++)
        {
            if(isZero(points_map_[i].ndt_mean_(0, 0)) == false ||
               isZero(points_map_[i].ndt_mean_(1, 0)) == false)
            {
                delta_p = points_map_[i].pos_ - points_map_[i].ndt_mean_;
                exp_val = -0.5*(delta_p.T()*points_map_[i].ndt_cov_inv_*delta_p)(0, 0);
                score_sum += -exp(exp_val);
            }
        }
        return score_sum;
    }
}

#endif