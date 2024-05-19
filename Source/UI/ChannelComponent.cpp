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
    Font f = Font ("Small Text", 13, Font::plain);

    staticLabel = new Label ("Channel", "Channel");
    staticLabel->setFont (f);
    staticLabel->setEditable (false);
    addAndMakeVisible (staticLabel);

    editName = new Label (name, name);
    editName->setFont (f);
    editName->setEditable (false);
    editName->setColour (Label::backgroundColourId, juce::Colours::lightgrey);
    editName->addListener (this);
    addAndMakeVisible (editName);

    if (type == ContinuousChannel::ELECTRODE)
    {
        impedance = new Label ("Impedance", "? Ohm");
        impedance->setFont (Font ("Default", 13, Font::plain));
        impedance->setEditable (false);
        addAndMakeVisible (impedance);
    }
    else if (type == ContinuousChannel::ADC)
    {
        impedance = nullptr;
        rangeComboBox = new ComboBox ("ADC Ranges");
        rangeComboBox->addItem ("-5V - +5V", 1);
        rangeComboBox->addItem ("0V - +5V", 2);
        //SourceNode* proc = channelList->proc;
        // RHD2000Thread* thread = static_cast<RHD2000Thread*>(proc->getThread());
        // rangeComboBox->setSelectedId(thread->getAdcRange(proc->getDataChannel(channel)->getSourceTypeIndex()) + 1, dontSendNotification);
        rangeComboBox->addListener (this);
        addAndMakeVisible (rangeComboBox);
    }
    else
    {
        impedance = nullptr;
        rangeComboBox = nullptr;
    }
}

void ChannelComponent::setImpedanceValues (float mag, float phase)
{
    if (impedance != nullptr)
    {
        if (mag > 10000)
            impedance->setText (String (mag / 1e6, 2) + " MOhm, " + String ((int) phase) + " deg", juce::NotificationType::dontSendNotification);
        else if (mag > 1000)
            impedance->setText (String (mag / 1e3, 0) + " kOhm, " + String ((int) phase) + " deg", juce::NotificationType::dontSendNotification);
        else
            impedance->setText (String (mag, 0) + " Ohm, " + String ((int) phase) + " deg", juce::NotificationType::dontSendNotification);
    }
    else
    {
    }
}

void ChannelComponent::comboBoxChanged (ComboBox* comboBox)
{
    if (comboBox == rangeComboBox)
    {
        // SourceNode* proc = channelList->proc;
        // RHD2000Thread* thread = static_cast<RHD2000Thread*>(proc->getThread());
        // thread->setAdcRange(proc->getDataChannel(channel)->getSourceTypeIndex(), comboBox->getSelectedId() - 1);
    }
}
void ChannelComponent::labelTextChanged (Label* lbl)
{
    // channel name change
    String newName = lbl->getText();
    channelList->setNewName (channel, newName);
}

void ChannelComponent::disableEdit()
{
    editName->setEnabled (false);
}

void ChannelComponent::enableEdit()
{
    editName->setEnabled (true);
}

void ChannelComponent::buttonClicked (Button* btn)
{
}

void ChannelComponent::setUserDefinedData (int d)
{
}

int ChannelComponent::getUserDefinedData()
{
    return 0;
}

void ChannelComponent::resized()
{
    editName->setBounds (0, 0, 90, 20);
    if (rangeComboBox != nullptr)
    {
        rangeComboBox->setBounds (100, 0, 80, 20);
    }
    if (impedance != nullptr)
    {
        impedance->setBounds (100, 0, 130, 20);
    }
}
