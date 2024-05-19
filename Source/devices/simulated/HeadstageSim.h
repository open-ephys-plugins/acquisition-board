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

#ifndef __HEADSTAGESIM_H_2C4CBD67__
#define __HEADSTAGESIM_H_2C4CBD67__

#include "../Headstage.h"

/** 
    A headstage for a simulated Acquisition Board

*/
class HeadstageSim : public Headstage
{
public:
    /** Constructor */
    HeadstageSim();

    /** Destructor*/
    ~HeadstageSim();

    /** Returns the number of channels this headstage sends*/
    int getNumActiveChannels() const;

    /** Returns the name of a channel at a given index*/
    String getChannelName (int ch) const;

    /** Returns the name of the headstage stream (used for naming AUX channels) */
    String getStreamPrefix() const;

    /** Returns true if impedances for a headstage channel have been measured */
    bool hasValidImpedance (int ch) const;

    /** Returns the impedance magnitude for a given channel */
    float getImpedanceMagnitude (int ch) const;

    /** Returns the impedance phase for a given channel */
    float getImpedancePhase (int ch) const;

    /** Generates names for each channel, depending on the scheme */
    void generateChannelNames (ChannelNamingScheme scheme);

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (HeadstageSim);
};

#endif // __HEADSTAGESIM_H_2C4CBD67__
