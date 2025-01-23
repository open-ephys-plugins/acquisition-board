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

    numberingSchemeLabel = std::make_unique<Label> ("Channel Names:", "Channel Names:");
    numberingSchemeLabel->setFont (FontOptions ("Inter", "Semi Bold", 15.0f));
    numberingSchemeLabel->setEditable (false);
    numberingSchemeLabel->setBounds (10, 10, 150, 25);
    addAndMakeVisible (numberingSchemeLabel.get());

    numberingScheme = std::make_unique<ComboBox> ("numberingScheme");
    numberingScheme->addItem ("Global", 1);
    numberingScheme->addItem ("Stream-Based", 2);
    numberingScheme->setBounds (125, 10, 140, 25);
    numberingScheme->addListener (this);
    numberingScheme->setSelectedId (1, dontSendNotification);
    addAndMakeVisible (numberingScheme.get());

    impedanceButton = std::make_unique<UtilityButton> ("Measure Impedances");
    impedanceButton->setRadius (3);
    impedanceButton->setBounds (280, 10, 145, 25);
    impedanceButton->setFont (FontOptions (14.0f));
    impedanceButton->addListener (this);
    addAndMakeVisible (impedanceButton.get());

    saveImpedanceButton = std::make_unique<UtilityButton> ("Save Impedances");
    saveImpedanceButton->setRadius (3);
    saveImpedanceButton->setBounds (430, 10, 145, 25);
    saveImpedanceButton->setFont (FontOptions (14.0f));
    saveImpedanceButton->addListener (this);
    saveImpedanceButton->setEnabled (false);
    addAndMakeVisible (saveImpedanceButton.get());

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
    if (btn == impedanceButton.get())
    {
        editor->measureImpedances();
        saveImpedanceButton->setEnabled (true);
    }
    else if (btn == saveImpedanceButton.get())
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
        lbl->setColour (Label::textColourId, findColour (ThemeColours::defaultText));
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

            if (hs->hasValidImpedance (ch))
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
}

void ChannelList::disableAll()
{
    impedanceButton->setEnabled (false);
    saveImpedanceButton->setEnabled (false);
    numberingScheme->setEnabled (false);
}

void ChannelList::enableAll()
{
    impedanceButton->setEnabled (true);
    saveImpedanceButton->setEnabled (true);
    numberingScheme->setEnabled (true);
}

void ChannelList::comboBoxChanged (ComboBox* b)
{
    if (b == numberingScheme.get())
    {
        board->setNamingScheme ((ChannelNamingScheme) b->getSelectedId());

        CoreServices::updateSignalChain (editor);
    }
}
