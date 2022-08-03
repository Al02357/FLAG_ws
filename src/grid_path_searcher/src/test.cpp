#include <iostream>
#include <vector>
#include <Eigen/Eigen>

int main(){
    Eigen::Matrix3i test;
                                                test<< 1,2,3,
                                                4,5,6,
                                                7,8,9;
    for(int i = 0;i<9;i++){
        std::cout<<test(i)<<std::endl;
    }
    for(int i = 0;i<3;i++){
        for(int j = 0;j<3;j++){
            std::cout<<test(i,j)<<std::endl;       
             }
    }

    return 0;
}