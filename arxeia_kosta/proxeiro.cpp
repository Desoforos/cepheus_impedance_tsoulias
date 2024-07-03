#include <iostream>
#include <eigen3/Eigen/Dense>


int main() {
    // Create a 6-dimensional vector initialized to zero
    Eigen::VectorXd myVector = Eigen::VectorXd::Zero(6);

    // Print the vector
    std::cout << "myVector: " << myVector.transpose() << std::endl;

    return 0;
}
