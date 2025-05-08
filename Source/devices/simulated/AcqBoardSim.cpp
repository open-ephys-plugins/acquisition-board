/*
    ------------------------------------------------------------------

    This file is part of the Open Ephys GUI
    Copyright (C) 2024 Open Ephys

    ------------------------------------------------------------------

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "AcqBoardSim.h"

const int MAX_NUM_HEADSTAGES = 8;

AcqBoardSim::AcqBoardSim () : AcquisitionBoard ()
{
    boardType = BoardType::Simulated;

    impedanceMeter = std::make_unique<ImpedanceMeterSim> ();

    for (int i = 0; i < MAX_NUM_HEADSTAGES; i++)
        headstages.add (new HeadstageSim (i));

}

AcqBoardSim::~AcqBoardSim()
{
}

bool AcqBoardSim::detectBoard()
{
    return true;
}

bool AcqBoardSim::initializeBoard()
{
    return true;
}

bool AcqBoardSim::foundInputSource() const
{
    return true;
}

Array<const Headstage*> AcqBoardSim::getHeadstages()
{

    Array<const Headstage*> connectedHeadstages;

    for (auto headstage : headstages)
    {
        if (headstage->isConnected())
        {
			connectedHeadstages.add (headstage);
		}
	}

    return connectedHeadstages;
}

Array<int> AcqBoardSim::getAvailableSampleRates()
{
    Array<int> sampleRates;
    
    sampleRates.add (1000);
    sampleRates.add (15000);
    sampleRates.add (30000);

    return sampleRates;
}

void AcqBoardSim::setSampleRate (int sampleRateHz)
{
    LOGD ("Simulated acquisition board setting sample rate to ", sampleRateHz, " Hz.");
    settings.boardSampleRate = float (sampleRateHz);
}

float AcqBoardSim::getSampleRate() const
{
    return settings.boardSampleRate;
}

void AcqBoardSim::scanPorts()
{
    for (int i = 0; i < MAX_NUM_HEADSTAGES; i++)
    {
        if (i == 0)
        {
            enableHeadstage (i, true);
		}
        else
        {
            enableHeadstage (i, false);
		}
    }
}


bool AcqBoardSim::enableHeadstage (int hsNum, bool enabled, int nStr, int strChans)
{
    LOGD ("Headstage ", hsNum, ", enabled: ", enabled, ", num streams: ", nStr, ", stream channels: ", strChans);
    LOGD ("Max num headstages: ", MAX_NUM_HEADSTAGES);

    if (enabled)
    {
        headstages[hsNum]->setFirstChannel (getNumDataOutputs (ContinuousChannel::ELECTRODE));
        headstages[hsNum]->setChannelCount (nStr * strChans);
    }
    else
    {
        headstages[hsNum]->setFirstChannel (-1);
        headstages[hsNum]->setChannelCount (0);
    }

    return true;
}

void AcqBoardSim::enableAuxChannels (bool enabled)
{
    settings.acquireAux = enabled;
}

bool AcqBoardSim::areAuxChannelsEnabled() const
{
    return settings.acquireAux;
}

void AcqBoardSim::enableAdcChannels (bool enabled)
{
    settings.acquireAdc = enabled;
}

bool AcqBoardSim::areAdcChannelsEnabled() const
{
    return settings.acquireAdc;
}

float AcqBoardSim::getBitVolts(ContinuousChannel::Type type) const
{
    switch (type)
    {
        case ContinuousChannel::ELECTRODE:
            return 0.195f;
        case ContinuousChannel::AUX:
            return 0.0000374f;
        case ContinuousChannel::ADC:
            return 0.00015258789f;
    }

    return 1.0f;
}

void AcqBoardSim::measureImpedances()
{
}

void AcqBoardSim::impedanceMeasurementFinished()
{
}

void AcqBoardSim::saveImpedances (File& file)
{
}

void AcqBoardSim::setNamingScheme (ChannelNamingScheme scheme)
{
    channelNamingScheme = scheme;

    for (auto hs : headstages)
    {
        hs->setNamingScheme (scheme);
    }
}

ChannelNamingScheme AcqBoardSim::getNamingScheme()
{
    return channelNamingScheme;
}

bool AcqBoardSim::isReady()
{
    return true;
}

bool AcqBoardSim::startAcquisition()
{
    startThread();

    return true;
}

bool AcqBoardSim::stopAcquisition()
{
    if (isThreadRunning())
    {
        signalThreadShouldExit();
    }

    buffer->clear();

    return true;
}

double AcqBoardSim::setUpperBandwidth (double upperBandwidth)
{
    settings.analogFilter.upperBandwidth = upperBandwidth;

    return upperBandwidth;
}

double AcqBoardSim::setLowerBandwidth (double lowerBandwidth)
{
    settings.analogFilter.lowerBandwidth = lowerBandwidth;

    return lowerBandwidth;
}

double AcqBoardSim::setDspCutoffFreq (double freq)
{
    settings.dsp.cutoffFreq = freq;

    return freq;
}

double AcqBoardSim::getDspCutoffFreq() const
{
    return settings.dsp.cutoffFreq;
}

void AcqBoardSim::setDspOffset (bool enabled)
{
    settings.dsp.enabled = false;
}

void AcqBoardSim::setTTLOutputMode (bool enabled)
{
    settings.ttlOutputMode = enabled;
}

void AcqBoardSim::setDAChpf (float cutoff, bool enabled)
{
    settings.desiredDAChpf = cutoff;
}

void AcqBoardSim::setFastTTLSettle (bool state, int channel)
{
    settings.fastSettleEnabled = state;
    settings.fastSettleTTLChannel = channel;
}

int AcqBoardSim::setNoiseSlicerLevel (int level)
{
    settings.noiseSlicerLevel = level;

    return level;
}

void AcqBoardSim::enableBoardLeds (bool enabled)
{
    settings.ledsEnabled = enabled;
}

int AcqBoardSim::setClockDivider (int divide_ratio)
{
    return 1;
}

void AcqBoardSim::connectHeadstageChannelToDAC (int headstageChannelIndex, int dacChannelIndex)
{
}

void AcqBoardSim::setDACTriggerThreshold (int dacChannelIndex, float threshold)
{
}

bool AcqBoardSim::isHeadstageEnabled (int hsNum) const
{
    return headstages[hsNum]->isConnected();
}

int AcqBoardSim::getActiveChannelsInHeadstage (int hsNum) const
{
    return headstages[hsNum]->getNumActiveChannels();
   
}

int AcqBoardSim::getChannelsInHeadstage (int hsNum) const
{
    if (headstages[hsNum]->isConnected())
        return 32;
    else
        return 0;
}

int AcqBoardSim::getNumDataOutputs (ContinuousChannel::Type type)
{
    if (type == ContinuousChannel::ELECTRODE)
    {
        int totalChannels = 0;

        for (auto headstage : headstages)
        {
            if (headstage->isConnected())
            {
                totalChannels += headstage->getNumActiveChannels();
            }
        }

        return totalChannels;
    }
    if (type == ContinuousChannel::AUX)
    {
        if (settings.acquireAux)
        {
            int numAuxOutputs = 0;

            for (auto headstage : headstages)
            {
                if (headstage->isConnected())
                {
                    numAuxOutputs += 3;
                }
            }
            return numAuxOutputs;
        }
        else
        {
            return 0;
        }
    }
    if (type == ContinuousChannel::ADC)
    {
        if (settings.acquireAdc)
        {
            return 8;
        }
        else
        {
            return 0;
        }
    }

    return 0;
}

void AcqBoardSim::setNumHeadstageChannels (int hsNum, int numChannels)
{
    headstages[hsNum]->setChannelCount (numChannels);
}

void AcqBoardSim::run()
{
    int64 sampleNumber = 0;
    int64 samplesPerBuffer = int64 (settings.boardSampleRate / 1000.0);
    int64 uSecPerBuffer = (samplesPerBuffer / settings.boardSampleRate) * 1e6;
    uint64 eventCode = 0;
    int skip = 30000.0f / settings.boardSampleRate;

    int64 start = Time::getHighResolutionTicks();
    int64 bufferCount = 0;
    const int numHeadstageChannels = getNumDataOutputs (ContinuousChannel::ELECTRODE);
    const int numAuxChannels = getNumDataOutputs (ContinuousChannel::AUX);
    const int numAdcChannels = getNumDataOutputs (ContinuousChannel::ADC);

    const int availableHeadstageSamples = data.spikes.size();
    const int availableAuxSamples = data.sine_wave.size();
    const int availableAdcSamples = data.adc.size();
    
    LOGD (" ");
    LOGD ("Starting acquisition.");
    LOGD ("Sample rate: ", settings.boardSampleRate);
    LOGD ("samplesPerBuffer: ", samplesPerBuffer);
    LOGD ("uSecPerBuffer: ", uSecPerBuffer);
    LOGD ("numHeadstageChannels: ", numHeadstageChannels);
    LOGD ("availableHeadstageSamples: ", availableHeadstageSamples);
    LOGD (" ");

    while (! threadShouldExit())
    {

        bufferCount++;

        for (int sample_num = 0; sample_num < samplesPerBuffer; sample_num++)
        {

            int ch = 0;

            for (int headstageChannel = 0; headstageChannel < numHeadstageChannels; headstageChannel++)
            {
                samples[(ch * samplesPerBuffer) + sample_num] = data.spikes[(sampleNumber * skip) % availableHeadstageSamples];
                ch++;
            }

            if (settings.acquireAux)
            {
                for (auto headstage : headstages)
                {
                    if (headstage->isConnected())
                    {
                        for (int aux_ch = 0; aux_ch < 3; aux_ch++)
                        {
                            samples[(ch * samplesPerBuffer) + sample_num] = data.sine_wave[(sampleNumber * skip) % availableAuxSamples] * 0.01f;
							ch++;
						}
					
                    }
                }
            }

            if (settings.acquireAdc)
            {

                for (int adc_ch = 0; adc_ch < 8; adc_ch++)
                {
                    samples[(ch * samplesPerBuffer) + sample_num] = data.adc[(sampleNumber * skip) % availableAdcSamples];
                    ch++;
                }
            }

            sampleNumbers[sample_num] = sampleNumber++;
            timestamps[sample_num] = -1.0;

            if (sampleNumber < settings.boardSampleRate * 30.0f)
            {
                if (sampleNumber % int (settings.boardSampleRate) == 0)
                {
                    if (eventCode == 0)
                        eventCode = 1;
                    else
                        eventCode = 0;
                }
            }
            else if (sampleNumber < settings.boardSampleRate * 60.0f)
            {
                if (sampleNumber % int (settings.boardSampleRate / 2) == 0)
                {
                    if (eventCode == 0)
                        eventCode = 1;
                    else
                        eventCode = 0;
                }
            }
            else if (sampleNumber < settings.boardSampleRate * 90.0f)
            {
                if (sampleNumber % int (settings.boardSampleRate / 4) == 0)
                {
                    if (eventCode == 0)
                        eventCode = 1;
                    else
                        eventCode = 0;
                }
            }
            else
            {
                if (sampleNumber % int (settings.boardSampleRate / 8) == 0)
                {
                    if (eventCode == 0)
                        eventCode = 1;
                    else
                        eventCode = 0;
                }
            }

            event_codes[sample_num] = eventCode;
        }

        buffer->addToBuffer (samples, sampleNumbers, timestamps, event_codes, samplesPerBuffer);

        int64 uSecElapsed = int64 (Time::highResolutionTicksToSeconds (Time::getHighResolutionTicks() - start) * 1e6);

        if (uSecElapsed < (uSecPerBuffer * bufferCount))
        {
            std::this_thread::sleep_for (std::chrono::microseconds ((uSecPerBuffer * bufferCount) - uSecElapsed));
        }
    }

}