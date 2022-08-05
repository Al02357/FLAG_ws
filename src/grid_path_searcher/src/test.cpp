#include <iostream>
#include <vector>
#include <Eigen/Eigen>

int main(){
    Eigen::Matrix3i test;
                                                test<< 1,2,3,
                                                4,5,6,
                                                7,8,9;
    std::vector<Eigen::Vector2d> loop;
    Eigen::Vector2d ptr;
    for(int i = 0;i<10;i++)
    {
        ptr << 2*i,2*i+1;
        loop.push_back(ptr);
    }
    int i = 0;
    auto iter = loop.begin();
    do{
        std::cout<<"loop:"<<++i<<std::endl;
        std::cout<<*(iter)<<std::endl;
        loop.erase(iter);
        iter = loop.begin();
    }
    while(iter != loop.end());

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