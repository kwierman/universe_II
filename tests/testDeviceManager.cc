#include <TUVMEDeviceManager.hh>
#include <TUVMEControlDevice.hh>
#include <iostream>

int main(){

    std::cout<<"Testing Device Manager"<<std::endl;

    TUVMEDeviceManager* man = TUVMEDeviceManager::GetDeviceManager();
    std::cout<<"Getting Control Device"<<std::endl;

    TUVMEControlDevice* con = man->GetControlDevice();
    std::cout<<"\t Control Device"<<con<<std::endl;


    std::cout<<"Done"<<std::endl;
}
