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

#ifndef __HEADSTAGEOPALKELLY_H_2C4CBD67__
#define __HEADSTAGEOPALKELLY_H_2C4CBD67__

#include "../Headstage.h"
#include "../ImpedanceMeter.h"

#include "rhythm-api/okFrontPanelDLL.h"
#include "rhythm-api/rhd2000datablock.h"
#include "rhythm-api/rhd2000evalboard.h"
#include "rhythm-api/rhd2000registers.h"

/** 
   
    Represents a headstage connected to an Open Ephys Acquisition Board
    with an Opal Kelly FPGA

*/
class HeadstageOpalKelly : public Headstage
{
public:
    /** Constructor */
    HeadstageOpalKelly (Rhd2000EvalBoard::BoardDataSource dataSource, int MAX_NUM_HEADSTAGES);

    /** Destructor*/
    ~HeadstageOpalKelly() {}

     /** Returns the number of channels this headstage sends*/
    int getNumActiveChannels() const;

    /** Returns the name of a channel at a given index*/
    String getChannelName (int ch) const;

    /** Returns true if impedances for a headstage channel have been measured */
    bool hasValidImpedance (int ch) const;

    /** Returns the impedance magnitude for a given channel */
    float getImpedanceMagnitude (int ch) const;

    /** Returns the impedance phase for a given channel */
    float getImpedancePhase (int ch) const;

    /** Generates names for each channel, depending on the scheme */
    void generateChannelNames (ChannelNamingScheme scheme);

    /** Returns the index of this headstage's data stream (offset = 0 or 1) */
    int getStreamIndex (int offset) const;

    /** Sets the index of this headstage's data stream */
    void setFirstStreamIndex (int streamIndex);

    /** Sets the number of channels per stream*/
    void setChannelsPerStream (int nchan);

    /** Returns the number of channels the headstage supports (doesn't account for half channels) */
    int getNumChannels() const;

    /** Returns the number of streams this headstage sends*/
    int getNumStreams() const;

    /** Sets the number of streams for this headstage (1 or 2)*/
    void setNumStreams (int num);

    /** Returns true if the headstage is connected*/
    bool isConnected() const override;

    /** Returns the BoardDataSource object for a given index*/
    Rhd2000EvalBoard::BoardDataSource getDataStream (int index) const;

    /** Sets the number of half-channels; mainly used for the 16-ch RHD2132 board */
    void setHalfChannels (bool half);

    /** Returns true if the headstage is in half-channels mode */
    bool getHalfChannels() const { return halfChannels; }

    /** Sets impedance values after measurement*/
    void setImpedances (Impedances& impedances);

private:
    Rhd2000EvalBoard::BoardDataSource dataSource;

    int streamIndex;

    int numStreams;
    int channelsPerStream;

    bool halfChannels;

    int MAX_NUM_HEADSTAGES;
    
    ChannelNamingScheme channelNamingScheme = GLOBAL_INDEX;

    Array<float> impedanceMagnitudes;
    Array<float> impedancePhases;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (HeadstageOpalKelly);
};

#endif // __HEADSTAGE_H_2C4CBD67__
