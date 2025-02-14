#include "mac/mac_frame.hpp"

extern "C" uint8_t otMacFrameGetHeaderLength(otRadioFrame *aFrame)
{
    return static_cast<const ot::Mac::Frame *>(aFrame)->GetHeaderLength();
}

extern "C" uint16_t otMacFrameGetPayloadLength(otRadioFrame *aFrame)
{
    return static_cast<const ot::Mac::Frame *>(aFrame)->GetPayloadLength();
}

extern "C" uint8_t otMacFrameGetSecurityLevel(otRadioFrame *aFrame)
{
    uint8_t sec_level;
    static_cast<const ot::Mac::Frame *>(aFrame)->GetSecurityLevel(sec_level);
    return sec_level;
}
