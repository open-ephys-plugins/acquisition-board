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

    void lookAndFeelChanged() override;

    void setNewName (int channelIndex, String newName);
    void setNewGain (int channel, float gain);
    void disableAll();
    void enableAll();
    void buttonClicked (Button* btn);
    void update();
    void updateButtons();
    int getMaxChannels() { return maxChannels; }
    void comboBoxChanged (ComboBox* b);
    void updateImpedance (Array<int> streams, Array<int> channels, Array<float> magnitude, Array<float> phase);

private:
    Array<float> gains;
    Array<ChannelInfoObject::Type> types;

    bool chainUpdate;

    AcquisitionBoard* board;
    DeviceEditor* editor;

    ScopedPointer<UtilityButton> impedanceButton;
    ScopedPointer<UtilityButton> saveImpedanceButton;

    ScopedPointer<ComboBox> numberingScheme;
    ScopedPointer<Label> numberingSchemeLabel;

    OwnedArray<Label> staticLabels;
    OwnedArray<ChannelComponent> channelComponents;

    int maxChannels;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (ChannelList);
};

#endif // __RHD2000EDITOR_H_2AD3C591__
