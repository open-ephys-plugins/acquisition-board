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

#include "ChannelComponent.h"
#include "ChannelList.h"

ChannelComponent::ChannelComponent (ChannelList* cl,
                                    int ch,
                                    int gainIndex_,
                                    String name_,
                                    Array<float> gains_,
                                    ContinuousChannel::Type type_) : type (type_),
                                                                     gains (gains_),
                                                                     channelList (cl),
                                                                     channel (ch),
                                                                     name (name_),
                                                                     gainIndex (gainIndex_)
{
    FontOptions f = FontOptions ("Inter", "Regular", 13.0f);

    staticLabel = std::make_unique<Label> ("Channel", "Channel");
    staticLabel->setFont (f);
    staticLabel->setEditable (false);
    addAndMakeVisible (staticLabel.get());

    nameLabel = std::make_unique<Label> (name, name);
    nameLabel->setFont (f);
    nameLabel->setColour (Label::backgroundColourId, findColour (ThemeColours::componentBackground).darker (0.3f));
    nameLabel->setEditable (false);
    addAndMakeVisible (nameLabel.get());

    if (type == ContinuousChannel::ELECTRODE)
    {
        impedanceLabel = std::make_unique<Label> ("Impedance", "? Ohm");
        impedanceLabel->setFont (FontOptions ("Fira Code", "Regular", 13.0f));
        impedanceLabel->setEditable (false);
        addAndMakeVisible (impedanceLabel.get());
    }
    else if (type == ContinuousChannel::ADC)
    {
        impedanceLabel = nullptr;
    }
    else
    {
        impedanceLabel = nullptr;
    }
}

void ChannelComponent::lookAndFeelChanged()
{
    nameLabel->setColour (Label::backgroundColourId,
                          findColour (ThemeColours::componentBackground).darker (0.3f));
}

void ChannelComponent::setImpedanceValues (float mag, float phase)
{
    if (impedanceLabel != nullptr)
    {
        if (mag > 10000)
            impedanceLabel->setText (String (mag / 1e6, 2) + " MOhm, " + String ((int) phase) + " deg", juce::NotificationType::dontSendNotification);
        else if (mag > 1000)
            impedanceLabel->setText (String (mag / 1e3, 0) + " kOhm, " + String ((int) phase) + " deg", juce::NotificationType::dontSendNotification);
        else
            impedanceLabel->setText (String (mag, 0) + " Ohm, " + String ((int) phase) + " deg", juce::NotificationType::dontSendNotification);
    }
    else
    {
    }
}

void ChannelComponent::resized()
{
    nameLabel->setBounds (0, 0, 90, 20);

    if (impedanceLabel != nullptr)
    {
        impedanceLabel->setBounds (100, 0, 130, 20);
    }
}
