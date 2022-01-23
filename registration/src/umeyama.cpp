#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

using namespace Eigen;
using namespace std;

double calc_error(const MatrixXd &T,
                  const MatrixXd &src,
                  const MatrixXd &dst)
{
    const double error = (T * src - dst).squaredNorm();
    cout << "error: " << error << "\n\n";
    return error;
}

int main() {
    MatrixXd R(3, 3);
    MatrixXd t(3, 1);
    
    MatrixXd src(4, 3); // homog. coord.
    MatrixXd dst(4, 3); // homog. coord.
    
    src <<  1.,  2.,  3.,
        5.,  4.,  3.,
        0., 0., 0.,
        1,   1,   1;
    
    dst <<  0.,  1.,  2.,
        4.,  3.,  2.,
        -1., -1., -1.,
        1,   1,   1;
    
    MatrixXd src_block = src.block<3,3>(0,0);
    MatrixXd dst_block = dst.block<3,3>(0,0);
    cout << "src_block:\n" << src_block << "\n";
    cout << "dst_block:\n" << dst_block << "\n";
    
    MatrixXd cR_t_umeyama = umeyama(src_block, dst_block).inverse();
    cout << "cR_t_umeyama:\n" << cR_t_umeyama << "\n";
    cout << "error: " << calc_error(cR_t_umeyama, src, dst) << endl;
}

