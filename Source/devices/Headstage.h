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

#ifndef __HEADSTAGE_H_2C4CBD67__
#define __HEADSTAGE_H_2C4CBD67__

#include <DataThreadHeaders.h>

enum ChannelNamingScheme
{
    GLOBAL_INDEX = 1,
    STREAM_INDEX = 2
};

/** 
    A headstage object represents a data source containing 
    one or more Intan chips.

    Each headstage can send 1 or 2 data streams, each with 
    up to 64 channels.
    
    A headstage can be identified in the following ways:
    - dataSource   : port (A1-D2Ddr) that the headstage is connected to
    - streamIndex  : location in the array of active streams
    - startChannel : index of the first channel acquired by this headstage,
                     out of all actively acquired neural data channels

*/
class Headstage
{
public:
    /** Constructor */
    Headstage() { }

    /** Destructor*/
    ~Headstage() { }

    /** Returns the number of channels this headstage sends*/
    virtual int getNumActiveChannels() const = 0;

    /** Returns the name of a channel at a given index*/
    virtual String getChannelName (int ch) const = 0;

    /** Returns the name of the headstage stream (used for naming AUX channels) */
    virtual String getStreamPrefix () const = 0;

    /** Returns true if impedances for a headstage channel have been measured */
    virtual bool hasValidImpedance (int ch) const = 0;

    /** Returns the impedance magnitude for a given channel */
    virtual float getImpedanceMagnitude (int ch) const = 0;

    /** Returns the impedance phase for a given channel */
    virtual float getImpedancePhase (int ch) const = 0;

    /** Generates names for each channel, depending on the scheme */
    virtual void generateChannelNames (ChannelNamingScheme scheme) = 0;

    /** Sets the channel naming scheme*/
    void setNamingScheme (ChannelNamingScheme scheme)
    {
        generateChannelNames (scheme);
    }

private:
   
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (Headstage);
};

#endif // __HEADSTAGE_H_2C4CBD67__
