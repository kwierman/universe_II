#include <iostream>

#include "TUVMEControlDevice.hh"


int main(){
    std::cout<<"Starting unit tests"<<std::endl;
    std::cout<<"Testing TUVMEControlDevice"<<std::endl;

    TUVMEControlDevice mydevice;
    std::cout<<"\tOpen: "<<mydevice.Open()<<std::endl;;
    std::cout<<"\tDevice Number: "<<mydevice.GetDevNumber()<<std::endl;
    std::cout<<"\tVMEAddress: "<<mydevice.GetVMEAddress()<<std::endl;
    std::cout<<"\tSizeOfImage: "<<mydevice.GetSizeOfImage()<<std::endl;
    std::cout<<"\t Bus Error: "<<mydevice.CheckBusError()<<std::endl;
    std::cout<<"\t WriteControlRegister: "<<mydevice.WriteControlRegister()<<std::endl;
    std::cout<<"\t Enable: "<<mydevice.Enable()<<std::endl;

    mydevice.SetHWByteSwap();
    mydevice.SetDSNegationSpeed();
    mydevice.SetDSHighTimeBLTs();
    std::cout<<"\t GetPCIMemorySize: "<<mydevice.GetPCIMemorySize()<<std::endl;
    std::cout<<"\t GetBoardType: "<<mydevice.GetBoardType()<<std::endl;

    std::cout<<std::endl;

    std::cout<<"Successfully Completed"<<std::endl;
}
