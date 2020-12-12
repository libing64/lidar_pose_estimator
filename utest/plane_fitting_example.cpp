#include <iostream>
#include <Eigen/Dense>
using namespace Eigen;
using namespace std;

int main()
{
    int n = 100;
    // generate n points in the plane centered in p and spanned bu the u,v vectors.
    MatrixXd points_3D(3, n);
    Vector3d u = Vector3d::Random().normalized();
    Vector3d v = Vector3d::Random().normalized();
    Vector3d p = Vector3d::Random();
    points_3D = p.rowwise().replicate(n) + u * VectorXd::Random(n).transpose() + v * VectorXd::Random(n).transpose();
    MatrixXd initial_points = points_3D;

    Vector3d centroid = points_3D.rowwise().mean();
    points_3D.colwise() -= centroid;
    JacobiSVD<MatrixXd> svd(points_3D, ComputeFullU);
    Vector3d normal = svd.matrixU().col(2);
    double d = -normal.dot(centroid);

    cout << "Plane equation: " << normal.transpose() << " " << d << endl;
    cout << "Distances: " << (normal.transpose() * initial_points).array() + d << endl;
}