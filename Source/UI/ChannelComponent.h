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

#ifndef __CHANNELCOMPONENT_H_2AD3C591__
#define __CHANNELCOMPONENT_H_2AD3C591__

#include <VisualizerEditorHeaders.h>

class ChannelList;

/** 
    Holds information about a single channel.
*/
class ChannelComponent : public Component
{
public:
    /** Constructor */
    ChannelComponent (ChannelList* cl,
                      int ch,
                      int gainIndex,
                      String name,
                      Array<float> gains,
                      ContinuousChannel::Type type);

    /** Destructor */
    ~ChannelComponent() {}

    /** Called when look and feel is updated*/
    void lookAndFeelChanged() override;

    /** Updates impedance values for this channel */
    void setImpedanceValues (float mag, float phase);

    /** Sets layout */
    void resized() override;

    /** Holds channel type */
    const ContinuousChannel::Type type;

private:
    Array<float> gains;
    ChannelList* channelList;

    std::unique_ptr<Label> staticLabel, nameLabel, impedanceLabel;

    int channel;
    String name;
    int gainIndex;
    int userDefinedData;
    bool isEnabled;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (ChannelComponent);
};

#endif // __RHD2000EDITOR_H_2AD3C591__
