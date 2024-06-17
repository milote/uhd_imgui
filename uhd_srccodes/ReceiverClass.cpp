#include "ReceiverClass.h"

void ReceiverClass::initializeUSRP()
{
    std::string rx_args = ""; // "num_recv_frames=1024";
    rx_usrp = uhd::usrp::multi_usrp::make(rx_args);
    USRPinitializedflag = true;

    // Some USRP models do not have GPSDO feature. GPS sync functions will be disabled for them.
    if (rx_usrp->get_mboard_name().compare("B205mini") == 0)
        USRPgpsflag = -1;

}
void ReceiverClass::configure()
{
	rx_usrp->set_rx_rate((double)rxrate, rx_ch);  // Set rxrate
    samps_per_buff = static_cast<size_t>(0.1 * rx_usrp->get_rx_rate());
	
	uhd::tune_request_t tune_request(rxfreq, lo_offset); // Set freq
	rx_usrp->set_rx_freq(tune_request, rx_ch);

	rx_usrp->set_rx_gain(rxgain, rx_ch); // Set gain

    
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
	
	// Lock mboard clocks
	rx_usrp->set_clock_source("GPSDO");
	rx_usrp->set_time_source("GPSDO");
	std::cout << boost::format("Using RX Clock Source: %s") % rx_usrp->get_clock_source(0) << std::endl;
	std::cout << boost::format("Using RX Time Source: %s") % rx_usrp->get_time_source(0) << std::endl;
	std::cout << boost::format("Using RX Device: %s") % rx_usrp->get_pp_string()
		<< std::endl;

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

	thrd_savethread = std::thread(&ReceiverClass::savefile, this);

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
	
	thrd_receivethread.join();

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

void ReceiverClass::sync_to_gps()
{
    if (USRPgpsflag == -1)
    {
        std::cout << "Connected USRP does not have GPSDO!\n";
        return;
    }
        
    std::cout << boost::format("Using Device: %s\n") % rx_usrp->get_pp_string();

    try {
        size_t num_mboards = rx_usrp->get_num_mboards();
        size_t num_gps_locked = 0;
        for (size_t mboard = 0; mboard < num_mboards; mboard++) {
            std::cout << "Synchronizing mboard " << mboard << ": "
                << rx_usrp->get_mboard_name(mboard) << std::endl;

            // Set references to GPSDO
            rx_usrp->set_clock_source("gpsdo", mboard);
            rx_usrp->set_time_source("gpsdo", mboard);

            // Check for 10 MHz lock
            std::vector<std::string> sensor_names = rx_usrp->get_mboard_sensor_names(mboard);
            if (std::find(sensor_names.begin(), sensor_names.end(), "ref_locked")
                != sensor_names.end()) {
                std::cout << "Waiting for reference lock..." << std::flush;
                bool ref_locked = false;
                for (int i = 0; i < 30 and not ref_locked; i++) {
                    ref_locked = rx_usrp->get_mboard_sensor("ref_locked", mboard).to_bool();
                    if (not ref_locked) {
                        std::cout << "." << std::flush;
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                    }
                }
                if (ref_locked) {
                    std::cout << "LOCKED" << std::endl;
                }
                else {
                    std::cout << "FAILED" << std::endl;
                    std::cout << "Failed to lock to GPSDO 10 MHz Reference. Exiting." << std::endl;
                }
            }
            else {
                std::cout << boost::format(
                    "ref_locked sensor not present on this board.\n");
            }

            // Wait for GPS lock
            bool gps_locked = rx_usrp->get_mboard_sensor("gps_locked", mboard).to_bool();
            if (gps_locked) {
                num_gps_locked++;
                std::cout << boost::format("GPS Locked\n");
            }
            else {
                std::cerr
                    << "WARNING:  GPS not locked - time will not be accurate until locked"
                    << std::endl;
            }

            // Set to GPS time
            uhd::time_spec_t gps_time = uhd::time_spec_t(
                int64_t(rx_usrp->get_mboard_sensor("gps_time", mboard).to_int()));
            rx_usrp->set_time_next_pps(gps_time + 1.0, mboard);

            // Wait for it to apply
            // The wait is 2 seconds because N-Series has a known issue where
            // the time at the last PPS does not properly update at the PPS edge
            // when the time is actually set.
            std::this_thread::sleep_for(std::chrono::seconds(2));

            // Check times
            gps_time = uhd::time_spec_t(
                int64_t(rx_usrp->get_mboard_sensor("gps_time", mboard).to_int()));
            uhd::time_spec_t time_last_pps = rx_usrp->get_time_last_pps(mboard);
            std::cout << "USRP time: "
                << (boost::format("%0.9f") % time_last_pps.get_real_secs())
                << std::endl;
            std::cout << "GPSDO time: "
                << (boost::format("%0.9f") % gps_time.get_real_secs()) << std::endl;
            if (gps_time.get_real_secs() == time_last_pps.get_real_secs())
                std::cout << std::endl
                << "SUCCESS: USRP time synchronized to GPS time" << std::endl
                << std::endl;
            else
                std::cerr << std::endl
                << "ERROR: Failed to synchronize USRP time to GPS time"
                << std::endl
                << std::endl;
        }

        if (num_gps_locked == num_mboards and num_mboards > 1) {
            // Check to see if all USRP times are aligned
            // First, wait for PPS.
            uhd::time_spec_t time_last_pps = rx_usrp->get_time_last_pps();
            while (time_last_pps == rx_usrp->get_time_last_pps()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }

            // Sleep a little to make sure all devices have seen a PPS edge
            std::this_thread::sleep_for(std::chrono::milliseconds(200));

            // Compare times across all mboards
            bool all_matched = true;
            uhd::time_spec_t mboard0_time = rx_usrp->get_time_last_pps(0);
            for (size_t mboard = 1; mboard < num_mboards; mboard++) {
                uhd::time_spec_t mboard_time = rx_usrp->get_time_last_pps(mboard);
                if (mboard_time != mboard0_time) {
                    all_matched = false;
                    std::cerr << (boost::format("ERROR: Times are not aligned: USRP "
                        "0=%0.9f, USRP %d=%0.9f")
                        % mboard0_time.get_real_secs() % mboard
                        % mboard_time.get_real_secs())
                        << std::endl;
                }
            }
            if (all_matched) {
                std::cout << "SUCCESS: USRP times aligned" << std::endl << std::endl;
            }
            else {
                std::cout << "ERROR: USRP times are not aligned" << std::endl
                    << std::endl;
            }
        }
    }
    catch (std::exception& e) {
        std::cout << boost::format("\nError: %s") % e.what();
        std::cout << boost::format(
            "This could mean that you have not installed the GPSDO correctly.\n\n");
        std::cout << boost::format("Visit one of these pages if the problem persists:\n");
        std::cout << boost::format(
            " * N2X0/E1X0: http://files.ettus.com/manual/page_gpsdo.html");
        std::cout << boost::format(
            " * X3X0: http://files.ettus.com/manual/page_gpsdo_x3x0.html\n\n");
        std::cout << boost::format(
            " * E3X0: http://files.ettus.com/manual/page_usrp_e3x0.html#e3x0_hw_gps\n\n");
    }
}