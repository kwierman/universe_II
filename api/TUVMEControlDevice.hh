#ifndef _TUVMEControlDevice_hh_
#define _TUVMEControlDevice_hh_

#ifdef __cplusplus
#include "TUVMEDevice.hh"
#include <string>
#include "universeII.h"
#include "vme_api_en.h"
#include "ConcurrentVX40x.h"

class TUVMEControlDevice: public TUVMEDevice {

  public:
    TUVMEControlDevice();
    virtual ~TUVMEControlDevice();

    enum ECycleSpeeds { kNormal  = 0,
                        kFaster  = 1,
                        kFastest = 2 };

    enum EBoardType { kUnknown = UNIVERSE_BOARD_TYPE_UNKNOWN,
	              kCCT     = UNIVERSE_BOARD_TYPE_CCT,
                      kVMIC    = UNIVERSE_BOARD_TYPE_VMIC };

    virtual std::string GetDeviceStringName() {return "vmeex/ctl";}
    virtual int Enable() {return 0;}
    virtual int Open();

    int ReadIOPortMemory( uint16_t address );
    int WriteIOPortMemory( uint16_t address, uint8_t value );
    void SetHWByteSwap(bool doByteSwap = true);
    void SetDSNegationSpeed(ECycleSpeeds speed = kNormal);
    void SetDSHighTimeBLTs(ECycleSpeeds speed = kNormal);
    size_t GetPCIMemorySize();
    EBoardType GetBoardType();

  protected:

    uint32_t fRevisionID;
};

#endif /* __cplusplus */
#endif /* TUVMEControlDevice.hh */
