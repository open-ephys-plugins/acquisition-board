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

#include "ChannelList.h"

#include "ChannelComponent.h"

#include "../DeviceEditor.h"
#include "../devices/Headstage.h"

ChannelList::ChannelList (AcquisitionBoard* board_, DeviceEditor* editor_) : board (board_),
                                                                         editor (editor_),
                                                                         maxChannels (0)
{
    channelComponents.clear();

    numberingSchemeLabel = new Label ("Channel Names:", "Channel Names:");
    numberingSchemeLabel->setEditable (false);
    numberingSchemeLabel->setBounds (10, 10, 150, 25);
    addAndMakeVisible (numberingSchemeLabel);

    numberingScheme = new ComboBox ("numberingScheme");
    numberingScheme->addItem ("Global", 1);
    numberingScheme->addItem ("Stream-Based", 2);
    numberingScheme->setBounds (125, 10, 140, 25);
    numberingScheme->addListener (this);
    numberingScheme->setSelectedId (1, dontSendNotification);
    addAndMakeVisible (numberingScheme);

    impedanceButton = new UtilityButton ("Measure Impedances");
    impedanceButton->setRadius (3);
    impedanceButton->setBounds (280, 10, 140, 25);
    impedanceButton->addListener (this);
    addAndMakeVisible (impedanceButton);

    saveImpedanceButton = new UtilityButton ("Save Impedances");
    saveImpedanceButton->setRadius (3);
    saveImpedanceButton->setBounds (430, 10, 150, 25);
    saveImpedanceButton->addListener (this);
    saveImpedanceButton->setEnabled (false);
    addAndMakeVisible (saveImpedanceButton);

    gains.clear();
    gains.add (0.01);
    gains.add (0.1);
    gains.add (1);
    gains.add (2);
    gains.add (5);
    gains.add (10);
    gains.add (20);
    gains.add (50);
    gains.add (100);
    gains.add (500);
    gains.add (1000);

    update();
}

void ChannelList::lookAndFeelChanged()
{
    numberingSchemeLabel->setColour (Label::textColourId, findColour (ThemeColours::defaultText));

    update();
}

void ChannelList::buttonClicked (Button* btn)
{
    if (btn == impedanceButton)
    {
        editor->measureImpedances();
        saveImpedanceButton->setEnabled (true);
    }
    else if (btn == saveImpedanceButton)
    {
        FileChooser chooseOutputFile ("Please select the location to save...",
                                      File(),
                                      "*.xml");

        if (chooseOutputFile.browseForFileToSave (true))
        {
            // Use the selected file
            File impedenceFile = chooseOutputFile.getResult();
            editor->saveImpedances (impedenceFile);
        }
    }
}

void ChannelList::update()
{
    if (! board->foundInputSource())
    {
        disableAll();
        return;
    }

    staticLabels.clear();
    channelComponents.clear();
    impedanceButton->setEnabled (true);

    const int columnWidth = 250;

    Array<const Headstage*> headstages = board->getHeadstages();

    int column = -1;

    maxChannels = 0;

    numberingScheme->setSelectedId (board->getNamingScheme(), dontSendNotification);

    for (auto hs : headstages)
    {
        column++;

        maxChannels = hs->getNumActiveChannels() > maxChannels ? hs->getNumActiveChannels() : maxChannels;

        Label* lbl = new Label (hs->getStreamPrefix(), hs->getStreamPrefix());
        lbl->setEditable (false);
        lbl->setBounds (10 + column * columnWidth, 40, columnWidth, 25);
        lbl->setJustificationType (juce::Justification::centred);
        lbl->setColour (Label::textColourId, findColour(ThemeColours::defaultText));
        staticLabels.add (lbl);
        addAndMakeVisible (lbl);

        for (int ch = 0; ch < hs->getNumActiveChannels(); ch++)
        {
            ChannelComponent* comp =
                new ChannelComponent (
                    this,
                    ch,
                    0,
                    hs->getChannelName (ch),
                    gains,
                    ContinuousChannel::ELECTRODE);

            comp->setBounds (10 + column * columnWidth, 70 + ch * 22, columnWidth, 22);

            if (hs->hasValidImpedance(ch))
            {
                comp->setImpedanceValues (
                    hs->getImpedanceMagnitude (ch),
                    hs->getImpedancePhase (ch));
            }
            //comp->setUserDefinedData(k);
            channelComponents.add (comp);
            addAndMakeVisible (comp);
        }
    }

    if (column == -1) // no headstages found
    {
        impedanceButton->setEnabled (false);
    }

    //if (board->enableAdcs())
    //{
    // create ADC channel interface
    //}
}

void ChannelList::disableAll()
{
    for (auto channelComponent : channelComponents)
    {
        channelComponent->disableEdit();
    }

    impedanceButton->setEnabled (false);
    saveImpedanceButton->setEnabled (false);
    numberingScheme->setEnabled (false);
}

void ChannelList::enableAll()
{
    for (int k = 0; k < channelComponents.size(); k++)
    {
        channelComponents[k]->enableEdit();
    }
    impedanceButton->setEnabled (true);
    saveImpedanceButton->setEnabled (true);
    numberingScheme->setEnabled (true);
}

void ChannelList::setNewGain (int channel, float gain)
{
    //RHD2000Thread* thread = (RHD2000Thread*)proc->getThread();
    //thread->modifyChannelGain(channel, gain);
    //if (chainUpdate)
    //    proc->requestChainUpdate();
}

void ChannelList::setNewName (int channel, String newName)
{
    //RHD2000Thread* thread = (RHD2000Thread*)proc->getThread();
    //thread->modifyChannelName(channel, newName);
    //if (chainUpdate)
    //    proc->requestChainUpdate();
}

void ChannelList::updateButtons()
{
}

void ChannelList::comboBoxChanged (ComboBox* b)
{
    if (b == numberingScheme)
    {
        board->setNamingScheme ((ChannelNamingScheme) b->getSelectedId());

        CoreServices::updateSignalChain (editor);
    }
}

void ChannelList::updateImpedance (Array<int> streams, Array<int> channels, Array<float> magnitude, Array<float> phase)
{
    int i = 0;
    for (int k = 0; k < streams.size(); k++)
    {
        if (i >= channelComponents.size())
            break; //little safety

        if (channelComponents[i]->type != ContinuousChannel::ELECTRODE)
        {
            k--;
        }
        else
        {
            channelComponents[i]->setImpedanceValues (magnitude[k], phase[k]);
        }
        i++;
    }
}
