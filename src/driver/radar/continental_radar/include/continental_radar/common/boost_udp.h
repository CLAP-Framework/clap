#ifndef BOOST_UDP_H_
#define BOOST_UDP_H_

#include <string>
#include "boost/algorithm/string.hpp"
#include "boost/regex.hpp"
#include "boost/asio.hpp"
#include "boost/thread.hpp"
#include "boost/lexical_cast.hpp"

namespace common {

#define UDP_RECV_SIZE 1024

class BoostUdp
{
public:
    BoostUdp(boost::asio::io_service &udp_sock, 
        std::string local_ip, 
        int local_port, 
        std::string remote_ip, 
        int remote_port) 
            : local_ip_(local_ip), 
            remote_ip_(remote_ip),
            local_port_(local_port),
            remote_port_(remote_port),
            udp_sock_(udp_sock) {}
            
    ~BoostUdp() 
    { 
        udp_sock_.close(); 
    }

    void    start_sock();

    int     receive_data(unsigned char buf[]);

    int     send_data(const unsigned char str[], int len);

    void    close_sock() 
    { 
        udp_sock_.close();  
    }

private:
    std::string                     local_ip_;
    std::string                     remote_ip_;
    int                             local_port_;
    int                             remote_port_;
    boost::asio::ip::udp::socket    udp_sock_;
    mutable boost::mutex            mutex_;
};

} // namespace common

#endif
