#pragma once

#include <uhd/convert.hpp>
#include <uhd/exception.hpp>
#include <uhd/types/tune_request.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/safe_main.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/process.hpp>
#include <boost/program_options.hpp>
#include <vector>
#include <chrono>
#include <complex>
#include <csignal>
#include <fstream>
#include <iostream>
#include <numeric>
#include <regex>
#include <thread>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <chrono>
#include <iostream>
#include <string>
#include "ipp.h"

class ReceiverClass
{
private:
	// USRP recording metric
	uhd::usrp::multi_usrp::sptr rx_usrp;
	double rxfreq;
	int rxrate;
	double rxgain, lo_offset;
	size_t samps_per_buff = 10000; 
	size_t rx_ch = 1;

	// File saving metric
	int buffidx = 0, buffidx2save = -1;
	std::vector<UINT64> timetag = { 0,0 };
	std::string filename;

	// Signal Characteristics metric
	std::vector<double> ampVec;

	// DDC and Filters (CPU)
	Ipp32f* pTaps = nullptr;
	Ipp32fc* pTaps_c = nullptr;
	Ipp32fc* pDlySrc[2] = { nullptr, nullptr };
	IppsFIRSpec_32fc* pSpec = nullptr;
	Ipp8u* SR_pBuffer = nullptr;
	int numTaps = 2000; // refer to initFilter();
	int DlyLen;

	// FFT operation IPP variables
	IppsDFTSpec_C_32fc* pDFTSpec = nullptr;
	Ipp8u* pDFTBuffer = nullptr;
	Ipp8u* pDFTMemInit = nullptr;
	Ipp32fc* dft_in = nullptr;
	Ipp32fc* dft_out = nullptr;
	Ipp32f* magnSq = nullptr;
	Ipp32f* productpeaks = nullptr;
	Ipp32s* freqlist_inds = nullptr;
	void FFTfn(int fftlen)
	{
		freeFFTfn();

		int sizeSpec = 0, sizeInit = 0, sizeBuf = 0;
		ippsDFTGetSize_C_32fc(fftlen, IPP_FFT_NODIV_BY_ANY, ippAlgHintNone, &sizeSpec, &sizeInit, &sizeBuf);
		pDFTSpec = (IppsDFTSpec_C_32fc*)ippMalloc(sizeSpec);
		pDFTBuffer = (Ipp8u*)ippMalloc(sizeBuf);
		pDFTMemInit = (Ipp8u*)ippMalloc(sizeInit);
		ippsDFTInit_C_32fc(fftlen, IPP_FFT_NODIV_BY_ANY, ippAlgHintNone, pDFTSpec, pDFTMemInit);

		dft_in = ippsMalloc_32fc_L(fftlen);
		dft_out = ippsMalloc_32fc_L(fftlen);
		magnSq = ippsMalloc_32f_L(fftlen);
	}
	void freeFFTfn()
	{
		ippFree(pDFTSpec);
		ippFree(pDFTBuffer);
		ippFree(pDFTMemInit);

		ippsFree(dft_in);
		ippsFree(dft_out);
		ippsFree(magnSq);

		ippsFree(productpeaks);
		ippsFree(freqlist_inds);
	}

	// Thread control
	bool Receivingflag = true;
	bool Stopflag = false;
	std::thread savethread;
	std::mutex mut;
	std::condition_variable cv;

	// Arrays
	Ipp16sc* rxbuffs[2];
	Ipp32fc* rx_32fc;
	Ipp32fc* filtered;
	Ipp32fc* downsampled;
	void allocMem()
	{
		rxbuffs[0] = ippsMalloc_16sc_L(rxrate);
		rxbuffs[1] = ippsMalloc_16sc_L(rxrate);
		rx_32fc = ippsMalloc_32fc_L(rxrate*2); 
	}
	void freeMem()
	{
		ippsFree(rxbuffs[0]);
		ippsFree(rxbuffs[1]);
		ippsFree(rx_32fc);
	}

public:
	ReceiverClass(uhd::usrp::multi_usrp::sptr in_rx_usrp, double in_rxfreq,
		int in_rxrate, double in_lo_offset, double in_rxgain)
		: rx_usrp{in_rx_usrp}, rxfreq{in_rxfreq}, rxrate{in_rxrate}, 
		rxgain{in_rxgain}, lo_offset{ in_lo_offset }
	{
		configure();
		printf("Configuration Completed.\n");
		checkConfig();
		allocMem();
		printf("Receiver Class initialized.\n  Receiving at Centre freq = %f, RXrate = %f, Gain = %f\n", rx_usrp->get_rx_freq(rx_ch), 
			rx_usrp->get_rx_rate(rx_ch), rx_usrp->get_rx_gain(rx_ch));
	}
	~ReceiverClass()
	{
		freeMem();
	}
	void configure();
	bool checkConfig();
	std::vector<double>& getAmpVec() { return ampVec;}

	// Start the receiver and the process loop
	void start();
	void cancel() { Stopflag = true; }
	void savefile(); // Called as a worker thread
};