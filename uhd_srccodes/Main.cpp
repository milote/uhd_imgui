#include <iostream>
#include "USRP_receive.h"

void main()
{
	printf("Hello World\n");
	std::string rx_args = "num_recv_frames=1024"; 
	uhd::usrp::multi_usrp::sptr rx_usrp = uhd::usrp::multi_usrp::make(rx_args);
	ReceiverClass testreceiver(rx_usrp, 1e9, 1e6, 5e3, 20);

	return;
}
