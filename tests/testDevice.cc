#include <iostream>

#include "TUVMEDevice.hh"


int main(){
    std::cout<<"Starting unit tests"<<std::endl;
    std::cout<<"Testing TUVMEDevice"<<std::endl;

    for(uint32_t i =0; i<9;i++){
        std::cout<<"\tTesting Device: "<<i<<std::endl;
        TUVMEDevice mydevice(i);
        std::cout<<"\tOpen: "<<mydevice.Open()<<std::endl;;
        std::cout<<"\tDevice Number: "<<mydevice.GetDevNumber()<<std::endl;
        std::cout<<"\tVMEAddress: "<<mydevice.GetVMEAddress()<<std::endl;
        std::cout<<"\tSizeOfImage: "<<mydevice.GetSizeOfImage()<<std::endl;
        std::cout<<"\t Bus Error: "<<mydevice.CheckBusError()<<std::endl;
        std::cout<<"\t WriteControlRegister: "<<mydevice.WriteControlRegister()<<std::endl;
        std::cout<<"\t Enable: "<<mydevice.Enable()<<std::endl;

        std::cout<<std::endl;
    }

    std::cout<<"Successfully Completed"<<std::endl;
}
