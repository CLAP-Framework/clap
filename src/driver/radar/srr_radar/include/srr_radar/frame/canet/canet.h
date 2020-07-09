#ifndef FRAME_CANET_H_
#define FRAME_CANET_H_

// #include <arpa/inet.h>
#include <cstring>
#include <cstdint>
#include <netinet/in.h>
#include <boost/array.hpp>
#include <boost/system/error_code.hpp>
#include <boost/shared_ptr.hpp>

// #include "FastDelegate.h"
namespace drivers {
namespace canet {

enum {
    CANET_DATA32_LENGTH  = 2,
    CANET_DATA8_LENGTH  = 8,
    CANET_FRAME_LENGTH  = 13,
};

/** CanetFrame for CAN id an meta data*/
struct CanetFrame
{
    /** check if frame header is valid*/
    bool is_valid() const {
        return (dlc_ <= 8);
    }

    CanetFrame() : dlc_(0), reserved_(0), is_rtr_(0), is_extended_(0), id_(0), data64_(0) {}
    
    CanetFrame(const uint8_t* data) { std::memcpy(buf_, data, CANET_FRAME_LENGTH); }

    CanetFrame(uint32_t dlc, bool extended, bool rtr, uint32_t id)
        : dlc_(dlc), reserved_(0), is_rtr_(rtr?1:0), is_extended_(extended?1:0), id_(ntohl(id)) {}

    CanetFrame(uint32_t dlc, bool extended, bool rtr, uint32_t id, uint8_t* data)
        : dlc_(dlc), reserved_(0), is_rtr_(rtr?1:0), is_extended_(extended?1:0), id_(ntohl(id)), 
        data64_(*(reinterpret_cast<uint64_t*>(const_cast<uint8_t*>(data)))) {}

    void set_dlc_value(uint32_t len) { dlc_ = static_cast<uint8_t>(len); }
    void set_rtr_value(bool r) { is_rtr_ = r; }
    void set_extended_value(bool e) { is_extended_ = e; }
    void set_id_value(uint32_t id) { id_ = ntohl(id); }
    void set_data64_value(const uint64_t data) { data64_ = data; }
    void set_data32_value(const uint32_t* data) { data32_[0] = data[0]; data32_[1] = data[1]; }
    void set_data8_value(const uint8_t* data) { std::memcpy(data8_, data, CANET_DATA8_LENGTH); }
    void set_buf_value(const uint8_t* data) { std::memcpy(buf_, data, CANET_FRAME_LENGTH); }

    const bool      is_rtr() const {}
    const bool      is_extended() const {}
    const uint8_t   dlc() const { return dlc_; } 
    const uint32_t  id() const { return htonl(id_); } 
    const uint8_t*  data() { return reinterpret_cast<uint8_t*>(&data64_); }
    const uint8_t*  data8() { return data8_; }
    const uint32_t* data32() { return data32_; }
    const uint64_t  data64() { return data64_; }
    const uint8_t*  buf() { return buf_;}
  
    const uint8_t get_dlc_value() const { return dlc_; }
    const uint8_t get_id_value() const { return htonl(id_); }

private:
    union {
    #pragma pack(1)
        uint8_t buf_[CANET_FRAME_LENGTH];
        struct {
            uint8_t dlc_         :4;                            ///< Data length, 0 <= dlc <= 8
            uint8_t reserved_    :2;
            uint8_t is_rtr_      :1;                            ///< frame is a remote transfer request
            uint8_t is_extended_ :1;                            ///< frame uses 29 bit CAN identifier 
            uint32_t  id_;                                      ///< CAN ID (11 or 29 bits valid, depending on is_extended member
            union {
                uint64_t data64_;                               ///< CAN data buffer 
                uint32_t data32_[CANET_DATA32_LENGTH];          ///< CAN data buffer 
                uint8_t  data8_[CANET_DATA8_LENGTH];            ///< CAN data buffer 
            };
        };
    };

};      //end of struct CanetFrame


} // namespace canet
} // namespace drivers


#endif // end of FRAME_CANET_H_
