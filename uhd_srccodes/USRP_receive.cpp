#include "USRP_receive.h"

void ReceiverClass::configure()
{
	// Set rxrate
	printf("Attempting to set rate to %f\n", (double)rxrate);
	rx_usrp->set_rx_rate((double)rxrate, rx_ch); 
	printf("Set rate.\n");

	// Set freq
	uhd::tune_request_t tune_request(rxfreq, lo_offset);
	rx_usrp->set_rx_freq(tune_request, rx_ch);

	// Set gain
	rx_usrp->set_rx_gain(rxgain, rx_ch);
}

bool ReceiverClass::checkConfig()
{
	if (rx_usrp->get_rx_gain(rx_ch) != rxgain) {
		printf("Actual RX Gain: %f\n", rx_usrp->get_rx_gain(rx_ch));
		throw 1;
	}

	if (round(rx_usrp->get_rx_freq(rx_ch)) != rxfreq) {
		printf("Intended RX Freq: %.4f\n", rxfreq);
		printf("Actual RX Freq: %f\n", rx_usrp->get_rx_freq(rx_ch));
		throw 2;
	}

	if (round(rx_usrp->get_rx_rate(rx_ch)) != rxrate) {
		printf("Actual RX Rate: %f Msps\n", rx_usrp->get_rx_rate(rx_ch));
		throw 3;
	}

	return true;
}

void ReceiverClass::start()
{
	// Get a streamer
	uhd::stream_args_t stream_args("sc16", "sc16");
	std::vector<size_t> channel_nums;
	channel_nums.push_back(rx_ch); 
	stream_args.channels = channel_nums;
	uhd::rx_streamer::sptr rx_stream = rx_usrp->get_rx_stream(stream_args);
	uhd::rx_metadata_t md;

	// Setup streaming
	uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
	stream_cmd.stream_now = true;

	// Start receiving
	uint8_t bufIdx = 0;
	double timeout = 0.5;
	rx_stream->issue_stream_cmd(stream_cmd);

	savethread = std::thread(&ReceiverClass::savefile, this);

	while (!Stopflag)
	{
		// Loop over usrp mini sample buffers
		for (int rIdx = 0; rIdx < rxrate; rIdx += (int)samps_per_buff) {

			size_t num_rx_samps =
				rx_stream->recv(&rxbuffs[buffidx][rIdx], samps_per_buff, md, timeout);

			if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
				std::cout << boost::format("Timeout while streaming") << std::endl;
				break;
			}
			if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) {
				std::cerr << boost::format("Got an overflow indication\n");
				break; // we want to ensure timing integrity, so if overflow let's end
			}
			if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
				std::string error = str(boost::format("Receiver error: %s") % md.strerror());
				break;
			}
		}

		buffidx = (buffidx + 1) % 2;
		buffidx2save = buffidx;
		cv.notify_one();
	}

	// Issue stop command
	stream_cmd.stream_mode = uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS;
	rx_stream->issue_stream_cmd(stream_cmd);
	Receivingflag = false;
	
	savethread.join();

}

void ReceiverClass::savefile()
{
	std::unique_lock<std::mutex> lk(mut);
	while (!Stopflag)
	{
		// wait for condition variable to be signalled
		cv.wait(lk, [&] {return buffidx2save >= 0 || Stopflag; });
		if (Stopflag)
			break;

		filename = std::to_string(timetag[buffidx2save]) + ".bin";
		std::ofstream outfile(filename, std::ios::out | std::ios::binary);
		if (buffidx2save == 0)
		{
			outfile.write(reinterpret_cast<char*>(rxbuffs[buffidx2save]), rxrate * sizeof(Ipp16sc));
			printf("Wrote from buffer 0, %zd samples\n", rxrate);
		}
		outfile.close();
		buffidx2save = -1;
	}
}