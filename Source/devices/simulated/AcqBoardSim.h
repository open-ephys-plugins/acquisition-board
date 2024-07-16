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

#ifndef __ACQBOARDSIM_H_2C4CBD67__
#define __ACQBOARDSIM_H_2C4CBD67__

#include "../AcquisitionBoard.h"
#include "ImpedanceMeterSim.h"

/**
    Interface for a simulated Open Ephys Acquisition Board

    https://open-ephys.org/acq-board

    @see DataThread, SourceNode
*/

class AcqBoardSim : public AcquisitionBoard
{
public:
    /** Constructor */
    AcqBoardSim (DataBuffer* buffer_);

    /** Destructor */
    virtual ~AcqBoardSim();

    /** Detects whether a board is present */
    bool detectBoard();

    /** Initializes board after successful detection */
    bool initializeBoard();

     /** Returns true if the device is connected */
    bool foundInputSource() const;

    /** Returns an array of connected headstages for this board */
    Array<const Headstage*> getHeadstages();

     /** Returns available sample rates */
    Array<int> getAvailableSampleRates();

    /** Set sample rate */
    void setSampleRate (int sampleRateHz);

    /** Gets the current sample rate */
    float getSampleRate () const;

    /** Checks for connected headstages */
    void scanPorts();

    /** Enables AUX channel out */
    void enableAuxChannels (bool enabled);

    /** Checks whether AUX channels are enabled */
    bool areAuxChannelsEnabled() const;

    /** Enables ADC channel out */
    void enableAdcChannels (bool enabled);

    /** Checks whether ADC channels are enabled */
    bool areAdcChannelsEnabled() const;

    /** Returns bitVolts scaling value for each channel type */
    float getBitVolts (ContinuousChannel::Type) const;

     /** Measures impedance of each channel */
    void measureImpedances();

    /**  Called when impedance measurement is complete */
    void impedanceMeasurementFinished();

    /** Save impedance measurements to XML*/
    void saveImpedances (File& file);

    /** Sets the method for determining channel names*/
    void setNamingScheme (ChannelNamingScheme scheme);

    /** Gets the method for determining channel names*/
    ChannelNamingScheme getNamingScheme();

    /** Initializes data transfer*/
    bool startAcquisition();

    /** Stops data transfer */
    bool stopAcquisition();

    /** Sets analog filter upper limit; returns actual value */
    double setUpperBandwidth (double upperBandwidth);

    /** Sets analog filter lower limit; returns actual value */
    double setLowerBandwidth (double lowerBandwidth);

    /** Sets DSP cutoff frequency; returns actual value */
    double setDspCutoffFreq (double freq);

    /** Gets the current DSP cutoff frequency */
    double getDspCutoffFreq() const;

    /** Sets whether DSP offset is enabled */
    void setDspOffset (bool enabled);

    /** Sets whether TTL output mode is enabled */
    void setTTLOutputMode (bool enabled);

    /** Sets whether DAC highpass filter is enabled, and set the cutoff freq */
    void setDAChpf (float cutoff, bool enabled);

    /** Sets whether fast TTL settle is enabled, and set the trigger channel */
    void setFastTTLSettle (bool state, int channel);

    /** Sets level of noise slicer on DAC channels */
    int setNoiseSlicerLevel (int level);

    /** Turns LEDs on or off */
    void enableBoardLeds (bool enabled);

    /** Sets divider on clock output */
    int setClockDivider (int divide_ratio);

    /** Connects a headstage channel to a DAC */
    void connectHeadstageChannelToDAC (int headstageChannelIndex, int dacChannelIndex);

    /** Sets trigger threshold for DAC channel (if TTL output mode is enabled) */
    void setDACTriggerThreshold (int dacChannelIndex, float threshold);

     /** Returns true if a headstage is enabled */
    bool isHeadstageEnabled (int hsNum) const;

    /** Returns the active number of channels in a headstage */
    int getActiveChannelsInHeadstage (int hsNum) const;

    /** Returns the total number of channels in a headstage */
    int getChannelsInHeadstage (int hsNum) const;

    /** Returns total number of outputs per channel type */
    int getNumDataOutputs (ContinuousChannel::Type);

    /** Sets the number of channels to use in a headstage */
    void setNumHeadstageChannels (int headstageIndex, int channelCount);

private:
    /** Fills data buffer */
    void run();

    /** Impedance meter */
    std::unique_ptr<ImpedanceMeterSim> impedanceMeter;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (AcqBoardSim);
};

#endif
