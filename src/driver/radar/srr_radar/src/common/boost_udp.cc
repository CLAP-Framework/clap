
#include "srr_radar/common/boost_udp.h"

namespace common {

void BoostUdp::start_sock()
{
    // here the ip can change to 192.168.1.33
    boost::asio::ip::udp::endpoint local_add(boost::asio::ip::address_v4::from_string(local_ip_), local_port_);
    udp_sock_.open(local_add.protocol());
    udp_sock_.bind(local_add);
}

int BoostUdp::receive_data(unsigned char buf[])
{
    boost::asio::ip::udp::endpoint send_endpoint(boost::asio::ip::address_v4::from_string(local_ip_), local_port_);
    int ret = udp_sock_.receive_from(boost::asio::buffer(buf, UDP_RECV_SIZE), send_endpoint);
    return ret;
}

int BoostUdp::send_data(const unsigned char str[], int len)
{
    boost::asio::ip::udp::endpoint send_endpoint(boost::asio::ip::address_v4::from_string(remote_ip_), remote_port_); 
    int ret = udp_sock_.send_to(boost::asio::buffer(str, len), send_endpoint);
    return ret;
}

} // namespace common

