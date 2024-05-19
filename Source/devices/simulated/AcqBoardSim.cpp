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

AcqBoardSim::AcqBoardSim (DataBuffer* buffer_) : AcquisitionBoard (buffer_)
{
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

    return connectedHeadstages;
}

Array<int> AcqBoardSim::getAvailableSampleRates()
{
    Array<int> sampleRates;

    sampleRates.add (30000);
    sampleRates.add (15000);
    sampleRates.add (1000);

    return sampleRates;
}

void AcqBoardSim::setSampleRate (int sampleRateHz)
{
    settings.boardSampleRate = float (sampleRateHz);
}

float AcqBoardSim::getSampleRate() const
{
    return settings.boardSampleRate;
}

void AcqBoardSim::scanPorts()
{
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
    return 0.195;
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
}

ChannelNamingScheme AcqBoardSim::getNamingScheme()
{
    return channelNamingScheme;
}

bool AcqBoardSim::startAcquisition()
{
    return true;
}

bool AcqBoardSim::stopAcquisition()
{
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
    return true;
}

int AcqBoardSim::getActiveChannelsInHeadstage (int hsNum) const
{
    return 64;
}

int AcqBoardSim::getChannelsInHeadstage (int hsNum) const
{
    return 64;
}

int AcqBoardSim::getNumDataOutputs (ContinuousChannel::Type)
{
    return 256;
}

void AcqBoardSim::setNumHeadstageChannels (int headstageIndex, int channelCount)
{
    
}

void AcqBoardSim::run()
{
}