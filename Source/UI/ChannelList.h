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

#ifndef __CHANNELLIST_H_2AD3C591__
#define __CHANNELLIST_H_2AD3C591__

#include <VisualizerEditorHeaders.h>

#include "../devices/AcquisitionBoard.h"

class DeviceEditor;
class ChannelComponent;

class ChannelList : public Component,
                    public Button::Listener,
                    public ComboBox::Listener
{
public:
    /** Constructor */
    ChannelList (AcquisitionBoard* board,
                 DeviceEditor* editor);

    /** Destructor */
    ~ChannelList() {}

    /** Updates label colors */
    void lookAndFeelChanged() override;

    /** Disables all channels */
    void disableAll();

    /** Enables all channels */
    void enableAll();

    /** Button callback */
    void buttonClicked (Button* btn);

    /** ComboBox callback */
    void comboBoxChanged (ComboBox* b);

    /** Updates layout of channel list */
    void update();

    /** Returns the maximum number of channels (used for setting layout) */
    int getMaxChannels() { return maxChannels; }

private:
    Array<float> gains;
    Array<ChannelInfoObject::Type> types;

    bool chainUpdate;

    AcquisitionBoard* board;
    DeviceEditor* editor;

    std::unique_ptr<UtilityButton> impedanceButton;
    std::unique_ptr<UtilityButton> saveImpedanceButton;

    std::unique_ptr<ComboBox> numberingScheme;
    std::unique_ptr<Label> numberingSchemeLabel;

    OwnedArray<Label> staticLabels;
    OwnedArray<ChannelComponent> channelComponents;

    int maxChannels;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (ChannelList);
};

#endif // __RHD2000EDITOR_H_2AD3C591__
