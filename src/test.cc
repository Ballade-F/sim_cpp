//
// Created by daybeha on 2022/12/13.
//

#include <iostream>
#include "torch/torch.h"

using namespace std;

int main(){
    torch::Tensor tensor = torch::eye(3);
    cout  << tensor <<endl;

    return 0;
}
