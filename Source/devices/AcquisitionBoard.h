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

#ifndef __ACQUISITIONBOARD_H_2C4CBD67__
#define __ACQUISITIONBOARD_H_2C4CBD67__

#include <DataThreadHeaders.h>

#include "Headstage.h"
#include "ImpedanceMeter.h"

/** Instructions for settings digital output */
struct DigitalOutputCommand
{
    int ttlLine;
    bool state;
};

/**
    Abstract interface for any type of Open Ephys Acquisition Board

    https://open-ephys.org/acq-board

    @see DataThread, SourceNode
*/

class AcquisitionBoard : public Thread
{
public:
    /** Constructor */
    AcquisitionBoard (DataBuffer* buffer_) : Thread ("Acquisition Board"),
                                             buffer (buffer_)
    {
    }

    /** Destructor */
    virtual ~AcquisitionBoard() {}

    /** Detects whether a board is present */
    virtual bool detectBoard() = 0;

    /** Initializes board after successful detection */
    virtual bool initializeBoard() = 0;

    /** Returns true if the device is connected */
    virtual bool foundInputSource() const = 0;

    /** Returns an array of connected headstages for this board */
    virtual Array<const Headstage*> getHeadstages() = 0;

    /** Returns available sample rates */
    virtual Array<int> getAvailableSampleRates() = 0;

    /** Set sample rate */
    virtual void setSampleRate (int sampleRateHz) = 0;

    /** Get current sample rate */
    virtual float getSampleRate() const = 0;

    /** Checks for connected headstages */
    virtual void scanPorts() = 0;

    /** Enables AUX channel out */
    virtual void enableAuxChannels (bool enabled) = 0;

    /** Checks whether AUX channels are enabled */
    virtual bool areAuxChannelsEnabled() const = 0;

    /** Enables ADC channel out */
    virtual void enableAdcChannels (bool enabled) = 0;

    /** Checks whether ADC channels are enabled */
    virtual bool areAdcChannelsEnabled() const = 0;

    /** Returns bitVolts scaling value for each channel type */
    virtual float getBitVolts (ContinuousChannel::Type) const = 0;

    /** Measures impedance of each channel */
    virtual void measureImpedances() = 0;

    /** Called when impedance measurement is complete */
    virtual void impedanceMeasurementFinished() = 0;

    /** Save impedance measurements to XML*/
    virtual void saveImpedances (File& file) = 0;

    /** Sets the method for determining channel names*/
    virtual void setNamingScheme (ChannelNamingScheme scheme) = 0;

    /** Gets the method for determining channel names*/
    virtual ChannelNamingScheme getNamingScheme() = 0;

    /** Initiates data transfer */
    virtual bool startAcquisition() = 0;

    /** Stops data transfer */
    virtual bool stopAcquisition() = 0;

    /** Sets analog filter upper limit; returns actual value */
    virtual double setUpperBandwidth (double upperBandwidth) = 0;

    /** Sets analog filter lower limit; returns actual value */
    virtual double setLowerBandwidth (double lowerBandwidth) = 0;

    /** Sets DSP cutoff frequency; returns actual value */
    virtual double setDspCutoffFreq (double freq) = 0;

    /** Gets the current DSP cutoff frequency */
    virtual double getDspCutoffFreq () const = 0;

    /** Sets whether DSP offset is enabled */
    virtual void setDspOffset (bool enabled) = 0;

    /** Sets whether TTL output mode is enabled */
    virtual void setTTLOutputMode (bool enabled) = 0;

    /** Sets whether DAC highpass filter is enabled, and set the cutoff freq */
    virtual void setDAChpf (float cutoff, bool enabled) = 0;

    /** Sets whether fast TTL settle is enabled, and set the trigger channel */
    virtual void setFastTTLSettle (bool state, int channel) = 0;

    /** Sets level of noise slicer on DAC channels */
    virtual int setNoiseSlicerLevel (int level) = 0;

    /** Turns LEDs on or off */
    virtual void enableBoardLeds (bool enabled) = 0;

    /** Sets divider on clock output; returns the actual divide ratio */
    virtual int setClockDivider (int divide_ratio) = 0;

    /** Connects a headstage channel to a DAC */
    virtual void connectHeadstageChannelToDAC (int headstageChannelIndex, int dacChannelIndex) = 0;

    /** Sets trigger threshold for DAC channel (if TTL output mode is enabled) */
    virtual void setDACTriggerThreshold (int dacChannelIndex, float threshold) = 0;

    /** Returns true if a headstage is enabled */
    virtual bool isHeadstageEnabled (int hsNum) const = 0;

    /** Sets the number of channels to use in a headstage */
    virtual void setNumHeadstageChannels (int headstageIndex, int channelCount) = 0;

    /** Returns the active number of channels in a headstage */
    virtual int getActiveChannelsInHeadstage (int hsNum) const = 0;

    /** Returns the total number of channels in a headstage */
    virtual int getChannelsInHeadstage (int hsNum) const = 0;

    /** Returns the total number of channels of a given type */
    virtual int getNumDataOutputs (ContinuousChannel::Type) = 0;

    /** Returns total number of continuous channels */
    int getNumChannels()
    {
        return getNumDataOutputs (ContinuousChannel::ELECTRODE)
               + getNumDataOutputs (ContinuousChannel::AUX)
               + getNumDataOutputs (ContinuousChannel::ADC);
    }

    /** Trigger a digital output event */
    void triggerDigitalOutput (int ttlLine, int eventDurationMs)
    {
        DigitalOutputCommand command;
        command.ttlLine = ttlLine;
        command.state = true;

        digitalOutputCommands.push (command);

        DigitalOutputTimer* timer = new DigitalOutputTimer (this, ttlLine, eventDurationMs);

        digitalOutputTimers.add (timer);
    }

    /** Creates buffers for custom streams if the acquisition board type has them */
    virtual void createCustomStreams (OwnedArray<DataBuffer>& otherBuffers) {};

    /** Create stream and channel structures is the acquisition board type has custom streams and updates the buffers */
    virtual void updateCustomStreams (OwnedArray<DataStream>& otherStreams, OwnedArray<ContinuousChannel>& otherChannels) {};

protected:
    /** Timer for triggering digtial outputs */
    class DigitalOutputTimer : public Timer
    {
    public:
        /** Constructor */
        DigitalOutputTimer (AcquisitionBoard* board_, int tllLine, int eventDurationMs) : board (board_),
                                                                                          tllOutputLine (tllLine)
        {
            startTimer (eventDurationMs);
        }

        /** Destructor*/
        ~DigitalOutputTimer() {}

        /** Sends signal to turn off event channel*/
        void timerCallback()
        {
        
            stopTimer();

            board->addDigitalOutputCommand (this, tllOutputLine, false);
        }

    private:
        AcquisitionBoard* board;

        const int tllOutputLine;
    };

    void addDigitalOutputCommand (DigitalOutputTimer* timerToDelete, int ttlLine, bool state)
    {
        DigitalOutputCommand command;
        command.ttlLine = ttlLine;
        command.state = state;

        digitalOutputCommands.push (command);

        digitalOutputTimers.removeObject (timerToDelete);
    }



    /** Sample buffer to fill */
    DataBuffer* buffer;

    /** Optimum delay settings */
    struct OptimumDelay
    {
        float portA = -1;
        float portB = -1;
        float portC = -1;
        float portD = -1;
        float portE = -1;
        float portF = -1;
        float portG = -1;
        float portH = -1;
    };

    /** Cable length settings */
    struct CableLength
    {
        float portA = 0.914f;
        float portB = 0.914f;
        float portC = 0.914f;
        float portD = 0.914f;
        float portE = 0.914f;
        float portF = 0.914f;
        float portG = 0.914f;
        float portH = 0.914f;
    };

    /** Dsp settings*/
    struct Dsp
    {
        bool enabled = true;
        double cutoffFreq = 0.5;
    };

    /** Analog filter settings */
    struct AnalogFilter
    {
        double upperBandwidth = 7500.0f;
        double lowerBandwidth = 1.0f;
    };

    /** struct containing board settings*/
    struct Settings
    {
        bool acquireAux = false;
        bool acquireAdc = false;

        bool fastSettleEnabled = false;
        bool fastTTLSettleEnabled = false;
        int fastSettleTTLChannel = -1;
        bool ttlOutputMode = false;

        Dsp dsp;
        AnalogFilter analogFilter;

        int noiseSlicerLevel;

        bool desiredDAChpfState;
        double desiredDAChpf;
        float boardSampleRate = 30000.f;

        CableLength cableLength;
        OptimumDelay optimumDelay;

        int audioOutputL = -1;
        int audioOutputR = -1;
        bool ledsEnabled = true;
        bool newScan = true;
        int numberingScheme = 1;
        uint16 clockDivideFactor;

    } settings;

    /** Impedance meter */
    std::unique_ptr<ImpedanceMeter> impedanceMeter;

    /** Impedance data*/
    Impedances impedances;

    /** Determines how channel names are created */
    ChannelNamingScheme channelNamingScheme = GLOBAL_INDEX;

    /** Queue of commands for setting digital output state */
    std::queue<DigitalOutputCommand> digitalOutputCommands;

    /** Array of timers for setting digital output state */
    OwnedArray<DigitalOutputTimer> digitalOutputTimers;

    /** True if change in settings is needed during acquisition*/
    bool updateSettingsDuringAcquisition = false;
};

#endif
