#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;

int main()
{
    Eigen::Quaterniond a(1,1,1,0);
    Eigen::Quaterniond b = a;
    b.x() = 2;
    cout << a.vec() << endl << b.vec() << endl;
    return 0; 
}

