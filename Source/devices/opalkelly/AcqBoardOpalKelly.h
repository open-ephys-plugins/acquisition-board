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

#ifndef __ACQBOARDOPALKELLY_H_2C4CBD67__
#define __ACQBOARDOPALKELLY_H_2C4CBD67__

#include "../AcquisitionBoard.h"
#include "ImpedanceMeterOpalKelly.h"
#include "HeadstageOpalKelly.h"

#include "rhythm-api/okFrontPanelDLL.h"
#include "rhythm-api/rhd2000datablock.h"
#include "rhythm-api/rhd2000evalboard.h"
#include "rhythm-api/rhd2000registers.h"

#define CHIP_ID_RHD2132 1
#define CHIP_ID_RHD2216 2
#define CHIP_ID_RHD2164 4
#define CHIP_ID_RHD2164_B 1000
#define REGISTER_59_MISO_A 53
#define REGISTER_59_MISO_B 58
#define RHD2132_16CH_OFFSET 8

#define MAX_NUM_CHANNELS MAX_NUM_DATA_STREAMS_USB3 * 35 + 16

/**
    Interface for Open Ephys Acquisition Board with an Opal Kelly FPGA

    https://open-ephys.org/acq-board

    @see DataThread, SourceNode
*/

class AcqBoardOpalKelly : public AcquisitionBoard
{
    friend class ImpedanceMeterOpalKelly;
    
public:
    /** Constructor */
    AcqBoardOpalKelly();

    /** Destructor */
    virtual ~AcqBoardOpalKelly();

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

    /** Get current sample rate */
    float getSampleRate() const;

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

    bool isReady() override;

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

    /** Returns the current DSP cutoff frequency */
    double getDspCutoffFreq () const;

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
    
    /** Updates registers to modify settings */
    void updateRegisters();

    /** Updates board streams after scanning ports */
    void updateBoardStreams();

    /** Returns the device ID for an Intan chip*/
    int getIntanChipId (Rhd2000DataBlock* dataBlock, int stream, int& register59Value);

    /** Sets the cable length for a particular headstage */
    void setCableLength (int hsNum, float length);

    /** Enables or disables a given headstage */
    bool enableHeadstage (int hsNum, bool enabled, int nStr = 1, int strChans = 32);

    /**Returns the global channel index for a local headstage channel */
    int getChannelFromHeadstage(int headstageIndex, int channelIndex);

    /** ??? Returns the global channel index for a local headstage channel */
    int getHeadstageChannel (int& headstageIndex, int channelIndex) const;

    /** Rhythm API classes*/
    std::unique_ptr<Rhd2000EvalBoard> evalBoard;
    Rhd2000Registers chipRegisters;
    std::unique_ptr<Rhd2000DataBlock> dataBlock;
    Array<Rhd2000EvalBoard::BoardDataSource> enabledStreams;

    /** USB blocks to read on each callback */
    int numUsbBlocksToRead;

    /** Size of the incoming data block */
    unsigned int blockSize;

    /** Holds incoming sample data */
    float thisSample[MAX_NUM_CHANNELS];
    float auxBuffer[MAX_NUM_CHANNELS];
    float auxSamples[MAX_NUM_DATA_STREAMS_USB3][3];

    /** Headstages */
    OwnedArray<HeadstageOpalKelly> headstages;

    /** Holds the number of channels per data stream */
    Array<int> numChannelsPerDataStream;

    /** Maximum number of expected headstages and data streams*/
    int MAX_NUM_HEADSTAGES;
    int MAX_NUM_DATA_STREAMS;

    /** Impedance meter */
    std::unique_ptr<ImpedanceMeterOpalKelly> impedanceMeter;

    /** Storing values for DAC channels */
    Array<int> dacChannels;
    Array<int> dacStream;
    Array<int> dacThresholds;
    Array<bool> dacChannelsToUpdate;

    /** Storing values for chip identifiers*/
    Array<int> chipId;

    /** Holds state of digital output lines */
    int TTL_OUTPUT_STATE[16];

    /** True if physical device is connected */
    bool deviceFound = false;

    /** True if acquisition is active */
    bool isTransmitting;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (AcqBoardOpalKelly);
};

#endif
