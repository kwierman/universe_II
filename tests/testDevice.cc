#include <iostream>

#include "TUVMEDevice.hh"


int main(){
    std::cout<<"Starting unit tests"<<std::endl;
    std:cout<<"Testing TUVMEDevice"<<std::endl;

    TUVMEDevice mydevice(uint32_t(0));
    mydevice.Open();
    std::cout<<"\tDevice Number"<<mydevice.GetDevNumber()<<std::endl;
    std::cout<<"\tVMEAddress"<<mydevice.GetVMEAddress()<<std::endl;
    std::cout<<"\tSizeOfImage"<<mydevice.GetSizeOfImage()<<std::endl;


    std::cout<<"Successfully Completed"<<std::endl;
}
