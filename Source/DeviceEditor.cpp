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

#include "DeviceEditor.h"

#include "devices/AcquisitionBoard.h"

#include "UI/ChannelCanvas.h"

#include <cmath>

#ifdef WIN32
#if (_MSC_VER < 1800) //round doesn't exist on MSVC prior to 2013 version
inline double round (double x)
{
    return floor (x + 0.5);
}
#endif
#endif

DeviceEditor::DeviceEditor (GenericProcessor* parentNode,
                            AcquisitionBoard* board_)
    : VisualizerEditor (parentNode, "Acq Board", 340),
      board (board_)
{
    canvas = nullptr;
    noBoardsDetectedLabel = nullptr;

    if (board == nullptr)
    {
        noBoardsDetectedLabel = std::make_unique<Label> ("NoBoardsDetected", "No Boards Detected.");
        noBoardsDetectedLabel->setBounds (0, 15, 340, 125);
        noBoardsDetectedLabel->setAlwaysOnTop (true);
        noBoardsDetectedLabel->toFront(false);
        noBoardsDetectedLabel->setJustificationType (Justification::centred);
        noBoardsDetectedLabel->setColour(Label::textColourId, Colours::black);
        addAndMakeVisible (noBoardsDetectedLabel.get());

        return;
    }

    // add headstage-specific controls (currently just a toggle button)
    for (int i = 0; i < 4; i++)
    {
        HeadstageOptionsInterface* hsOptions = new HeadstageOptionsInterface (board, this, i);
        headstageOptionsInterfaces.add (hsOptions);
        addAndMakeVisible (hsOptions);
        hsOptions->setBounds (3, 28 + i * 20, 70, 18);
    }

    // add rescan button
    rescanButton = std::make_unique<UtilityButton> ("RESCAN");
    rescanButton->setRadius (3.0f);
    rescanButton->setBounds (6, 108, 65, 18);
    rescanButton->addListener (this);
    rescanButton->setTooltip ("Check for connected headstages");
    addAndMakeVisible (rescanButton.get());

    // add sample rate selection
    sampleRateInterface = std::make_unique<SampleRateInterface> (board, this);
    addAndMakeVisible (sampleRateInterface.get());
    sampleRateInterface->setBounds (80, 20, 80, 50);

    // add Bandwidth selection
    bandwidthInterface = std::make_unique<BandwidthInterface> (board, this);
    addAndMakeVisible (bandwidthInterface.get());
    bandwidthInterface->setBounds (80, 55, 80, 50);

    // add AUX channel enable/disable button
    auxButton = std::make_unique<UtilityButton> ("AUX");
    auxButton->setRadius (3.0f);
    auxButton->setBounds (80, 108, 32, 18);
    auxButton->addListener (this);
    auxButton->setClickingTogglesState (true);
    auxButton->setTooltip ("Toggle AUX channels (3 per headstage)");
    addAndMakeVisible (auxButton.get());

    // add ADC channel enable/disable button
    adcButton = std::make_unique<UtilityButton> ("ADC");
    adcButton->setRadius (3.0f);
    adcButton->setBounds (80 + 32 + 1, 108, 32, 18);
    adcButton->addListener (this);
    adcButton->setClickingTogglesState (true);
    adcButton->setTooltip ("Toggle 8 external HDMI ADC channels");
    addAndMakeVisible (adcButton.get());

    // add audio output config interface
    audioLabel = std::make_unique<Label> ("audio label", "Audio out");
    audioLabel->setBounds (170, 20, 75, 15);
    audioLabel->setFont (Font ("Small Text", 10, Font::plain));
    addAndMakeVisible (audioLabel.get());

    for (int i = 0; i < 2; i++)
    {
        ElectrodeButton* button = new ElectrodeButton (-1);
        electrodeButtons.add (button);

        button->setBounds (174 + i * 30, 35, 30, 15);
        button->setChannelNum (-1);
        button->setClickingTogglesState (false);
        button->setToggleState (false, dontSendNotification);

        addAndMakeVisible (button);
        button->addListener (this);

        if (i == 0)
        {
            button->setTooltip ("Audio monitor left channel");
        }
        else
        {
            button->setTooltip ("Audio monitor right channel");
        }
    }

    // add HW audio parameter selection
    audioInterface = std::make_unique<AudioInterface> (board, this);
    addAndMakeVisible (audioInterface.get());
    audioInterface->setBounds (174, 55, 70, 50);

    clockInterface = std::make_unique<ClockDivideInterface> (board, this);
    addAndMakeVisible (clockInterface.get());
    clockInterface->setBounds (174, 80, 70, 50);

    // add DSP Offset Button
    dspoffsetButton = std::make_unique<UtilityButton> ("DSP:");
    dspoffsetButton->setRadius (3.0f); // sets the radius of the button's corners
    dspoffsetButton->setBounds (174, 108, 32, 18); // sets the x position, y position, width, and height of the button
    dspoffsetButton->addListener (this);
    dspoffsetButton->setClickingTogglesState (true); // makes the button toggle its state when clicked
    dspoffsetButton->setTooltip ("Toggle DSP offset removal");
    addAndMakeVisible (dspoffsetButton.get()); // makes the button a child component of the editor and makes it visible
    dspoffsetButton->setToggleState (true, dontSendNotification);

    // add DSP Frequency Selection field
    dspInterface = std::make_unique<DSPInterface> (board, this);
    addAndMakeVisible (dspInterface.get());
    dspInterface->setBounds (174 + 32, 108, 40, 50);

    dacTTLButton = std::make_unique<UtilityButton> ("DAC TTL");
    dacTTLButton->setRadius (3.0f);
    dacTTLButton->setBounds (260, 25, 60, 18);
    dacTTLButton->addListener (this);
    dacTTLButton->setClickingTogglesState (true);
    dacTTLButton->setTooltip ("Toggle DAC Threshold TTL Output");
    addAndMakeVisible (dacTTLButton.get());

    dacHPFlabel = std::make_unique<Label> ("DAC HPF", "DAC HPF");
    dacHPFlabel->setFont (Font ("Small Text", 10, Font::plain));
    dacHPFlabel->setBounds (255, 40, 60, 20);
    addAndMakeVisible (dacHPFlabel.get());

    dacHPFcombo = std::make_unique<ComboBox> ("dacHPFCombo");
    dacHPFcombo->setBounds (260, 55, 60, 18);
    dacHPFcombo->addListener (this);
    dacHPFcombo->addItem ("OFF", 1);
    int HPFvalues[10] = { 50, 100, 200, 300, 400, 500, 600, 700, 800, 900 };
    for (int k = 0; k < 10; k++)
    {
        dacHPFcombo->addItem (String (HPFvalues[k]) + " Hz", 2 + k);
    }
    dacHPFcombo->setSelectedId (1, sendNotification);
    addAndMakeVisible (dacHPFcombo.get());

    ttlSettleLabel = std::make_unique<Label> ("TTL Settle", "TTL Settle");
    ttlSettleLabel->setFont (Font ("Small Text", 10, Font::plain));
    ttlSettleLabel->setBounds (255, 70, 70, 20);
    addAndMakeVisible (ttlSettleLabel.get());

    ttlSettleCombo = std::make_unique<ComboBox> ("FastSettleComboBox");
    ttlSettleCombo->setBounds (260, 85, 60, 18);
    ttlSettleCombo->addListener (this);
    ttlSettleCombo->addItem ("-", 1);
    for (int k = 0; k < 8; k++)
    {
        ttlSettleCombo->addItem ("TTL" + String (1 + k), 2 + k);
    }
    ttlSettleCombo->setSelectedId (1, sendNotification);
    addAndMakeVisible (ttlSettleCombo.get());

    ledButton = std::make_unique<UtilityButton> ("LED");
    ledButton->setRadius (3.0f);
    ledButton->setBounds (288, 108, 32, 18);
    ledButton->addListener (this);
    ledButton->setClickingTogglesState (true);
    ledButton->setTooltip ("Toggle board LEDs");
    ledButton->setToggleState (true, dontSendNotification);
    addAndMakeVisible (ledButton.get());
}

void DeviceEditor::measureImpedances()
{
    if (! acquisitionIsActive)
    {
        board->measureImpedances();

        CoreServices::updateSignalChain (this);
    }
}

void DeviceEditor::saveImpedances (File& file)
{
    LOGD ("Saving impedances to ", file.getFullPathName());

    board->saveImpedances (file);
}

void DeviceEditor::updateSettings()
{
    if (canvas != nullptr)
    {
        canvas->update();
    }
}

void DeviceEditor::comboBoxChanged (ComboBox* comboBox)
{
    if (comboBox == ttlSettleCombo.get())
    {
        int selectedChannel = ttlSettleCombo->getSelectedId();
        if (selectedChannel == 1)
        {
            board->setFastTTLSettle (false, 0);
        }
        else
        {
            board->setFastTTLSettle (true, selectedChannel - 2);
        }
    }
    else if (comboBox == dacHPFcombo.get())
    {
        int selection = dacHPFcombo->getSelectedId();
        if (selection == 1)
        {
            board->setDAChpf (100, false);
        }
        else
        {
            int HPFvalues[10] = { 50, 100, 200, 300, 400, 500, 600, 700, 800, 900 };
            board->setDAChpf (HPFvalues[selection - 2], true);
        }
    }
}

void DeviceEditor::channelStateChanged (Array<int> newChannels)
{
    int selectedChannel = -1;

    if (newChannels.size() > 0)
    {
        selectedChannel = newChannels[0];
    }

    board->connectHeadstageChannelToDAC (selectedChannel, int (activeAudioChannel));

    if (selectedChannel > -1)
    {
        electrodeButtons[int (activeAudioChannel)]->setToggleState (true, dontSendNotification);
        electrodeButtons[int (activeAudioChannel)]->setChannelNum (selectedChannel + 1);
    }
    else
    {
        electrodeButtons[int (activeAudioChannel)]->setChannelNum (selectedChannel);
        electrodeButtons[int (activeAudioChannel)]->setToggleState (false, dontSendNotification);
    }
}

void DeviceEditor::buttonClicked (Button* button)
{
    if (button == rescanButton.get() && ! acquisitionIsActive)
    {
        board->scanPorts();

        for (int i = 0; i < 4; i++)
        {
            headstageOptionsInterfaces[i]->checkEnabledState();
        }
        CoreServices::updateSignalChain (this);
    }
    else if (button == electrodeButtons[0] || button == electrodeButtons[1])
    {
        std::vector<bool> channelStates;

        if (button == electrodeButtons[0])
            activeAudioChannel = LEFT;
        else
            activeAudioChannel = RIGHT;

        for (int i = 0; i < board->getNumDataOutputs (ContinuousChannel::ELECTRODE); i++)
        {
            if (electrodeButtons[int (activeAudioChannel)]->getChannelNum() - 1 == i)
                channelStates.push_back (true);
            else
                channelStates.push_back (false);
        }

        auto* channelSelector = new PopupChannelSelector (this, this, channelStates);

        channelSelector->setChannelButtonColour (Colour (0, 174, 239));
        channelSelector->setMaximumSelectableChannels (1);

        CallOutBox& myBox = CallOutBox::launchAsynchronously (std::unique_ptr<Component> (channelSelector),
                                                              button->getScreenBounds(),
                                                              nullptr);
    }
    else if (button == auxButton.get() && ! acquisitionIsActive)
    {
        board->enableAuxChannels (button->getToggleState());
        LOGD ("AUX Button toggled");
        CoreServices::updateSignalChain (this);
    }
    else if (button == adcButton.get() && ! acquisitionIsActive)
    {
        board->enableAdcChannels (button->getToggleState());
        LOGD ("ADC Button toggled");
        CoreServices::updateSignalChain (this);
    }
    else if (button == dacTTLButton.get())
    {
        board->setTTLOutputMode (dacTTLButton->getToggleState());
    }
    else if (button == dspoffsetButton.get() && ! acquisitionIsActive)
    {
        LOGD ("DSP offset ", button->getToggleState());
        board->setDspOffset (button->getToggleState());
    }
    else if (button == ledButton.get())
    {
        board->enableBoardLeds (button->getToggleState());
    }
}

void DeviceEditor::startAcquisition()
{
    rescanButton->setEnabledState (false);
    auxButton->setEnabledState (false);
    adcButton->setEnabledState (false);
    dspoffsetButton->setEnabledState (false);

    for (auto headstageOptions : headstageOptionsInterfaces)
    {
		headstageOptions->setEnabled (false);
	}

    acquisitionIsActive = true;
}

void DeviceEditor::stopAcquisition()
{
    rescanButton->setEnabledState (true);
    auxButton->setEnabledState (true);
    adcButton->setEnabledState (true);
    dspoffsetButton->setEnabledState (true);

    for (auto headstageOptions : headstageOptionsInterfaces)
    {
        headstageOptions->setEnabled (true);
    }

    acquisitionIsActive = false;
}

void DeviceEditor::saveVisualizerEditorParameters (XmlElement* xml)
{
    if (board == nullptr)
    {
        return;
    }

    xml->setAttribute ("SampleRate", sampleRateInterface->getSelectedId());
    xml->setAttribute ("SampleRateString", sampleRateInterface->getText());
    xml->setAttribute ("LowCut", bandwidthInterface->getLowerBandwidth());
    xml->setAttribute ("HighCut", bandwidthInterface->getUpperBandwidth());
    xml->setAttribute ("AUXsOn", auxButton->getToggleState());
    xml->setAttribute ("ADCsOn", adcButton->getToggleState());
    xml->setAttribute ("AudioOutputL", electrodeButtons[0]->getChannelNum());
    xml->setAttribute ("AudioOutputR", electrodeButtons[1]->getChannelNum());
    xml->setAttribute ("NoiseSlicer", audioInterface->getNoiseSlicerLevel());
    xml->setAttribute ("TTLFastSettle", ttlSettleCombo->getSelectedId());
    xml->setAttribute ("DAC_TTL", dacTTLButton->getToggleState());
    xml->setAttribute ("DAC_HPF", dacHPFcombo->getSelectedId());
    xml->setAttribute ("DSPOffset", dspoffsetButton->getToggleState());
    xml->setAttribute ("DSPCutoffFreq", dspInterface->getDspCutoffFreq());
    xml->setAttribute ("LEDs", ledButton->getToggleState());
    xml->setAttribute ("ClockDivideRatio", clockInterface->getClockDivideRatio());

    // loop through all headstage options interfaces and save their parameters
    for (int i = 0; i < 4; i++)
    {
        XmlElement* hsOptions = xml->createNewChildElement ("HSOPTIONS");
        hsOptions->setAttribute ("index", i);
        hsOptions->setAttribute ("hs1_full_channels", headstageOptionsInterfaces[i]->is32Channel (0));
        hsOptions->setAttribute ("hs2_full_channels", headstageOptionsInterfaces[i]->is32Channel (1));
    }

    // save channel naming scheme
    xml->setAttribute ("Channel_Naming_Scheme", board->getNamingScheme());
}

void DeviceEditor::loadVisualizerEditorParameters (XmlElement* xml)
{
    sampleRateInterface->setSelectedId (xml->getIntAttribute ("SampleRate"));
    bandwidthInterface->setLowerBandwidth (xml->getDoubleAttribute ("LowCut"));
    bandwidthInterface->setUpperBandwidth (xml->getDoubleAttribute ("HighCut"));
    auxButton->setToggleState (xml->getBoolAttribute ("AUXsOn"), sendNotification);
    adcButton->setToggleState (xml->getBoolAttribute ("ADCsOn"), sendNotification);

    audioInterface->setNoiseSlicerLevel (xml->getIntAttribute ("NoiseSlicer"));
    ttlSettleCombo->setSelectedId (xml->getIntAttribute ("TTLFastSettle"));
    dacTTLButton->setToggleState (xml->getBoolAttribute ("DAC_TTL"), sendNotification);
    dacHPFcombo->setSelectedId (xml->getIntAttribute ("DAC_HPF"));
    dspoffsetButton->setToggleState (xml->getBoolAttribute ("DSPOffset"), sendNotification);
    dspInterface->setDspCutoffFreq (xml->getDoubleAttribute ("DSPCutoffFreq"));
    ledButton->setToggleState (xml->getBoolAttribute ("LEDs", true), sendNotification);
    clockInterface->setClockDivideRatio (xml->getIntAttribute ("ClockDivideRatio"));

    int AudioOutputL = xml->getIntAttribute ("AudioOutputL", -1);
    int AudioOutputR = xml->getIntAttribute ("AudioOutputR", -1);

    electrodeButtons[0]->setChannelNum (AudioOutputL);
    board->connectHeadstageChannelToDAC (0, AudioOutputL);
    if (AudioOutputL > -1)
        electrodeButtons[0]->setToggleState (true, dontSendNotification);

    electrodeButtons[1]->setChannelNum (AudioOutputR);
    board->connectHeadstageChannelToDAC (1, AudioOutputR);
    if (AudioOutputR > -1)
        electrodeButtons[1]->setToggleState (true, dontSendNotification);

    forEachXmlChildElementWithTagName (*xml, hsOptions, "HSOPTIONS")
    {
        int index = hsOptions->getIntAttribute ("index", -1);

        if (index > -1)
        {
            headstageOptionsInterfaces[index]->set32Channel (0, hsOptions->getBoolAttribute ("hs1_full_channels", true));
            headstageOptionsInterfaces[index]->set32Channel (1, hsOptions->getBoolAttribute ("hs2_full_channels", true));
        }
    }

    // load channel naming scheme
    board->setNamingScheme ((ChannelNamingScheme) xml->getIntAttribute ("Channel_Naming_Scheme", 0));
}

Visualizer* DeviceEditor::createNewCanvas()
{
    GenericProcessor* processor = (GenericProcessor*) getProcessor();

    if (board == nullptr)
    {
        return nullptr;
    }

    canvas = new ChannelCanvas (board, this);

    return canvas;
}

// Bandwidth Options --------------------------------------------------------------------

BandwidthInterface::BandwidthInterface (AcquisitionBoard* board_,
                                        DeviceEditor* editor_) : board (board_),
                                                                 editor (editor_)
{
    name = "Bandwidth";

    lastHighCutString = "7500";
    lastLowCutString = "1";

    actualUpperBandwidth = 7500.0f;
    actualLowerBandwidth = 1.0f;

    upperBandwidthSelection = std::make_unique<Label> ("UpperBandwidth", lastHighCutString); // this is currently set in DeviceThread, the cleaner way would be to set it here again
    upperBandwidthSelection->setEditable (true, false, false);
    upperBandwidthSelection->addListener (this);
    upperBandwidthSelection->setBounds (30, 25, 60, 20);
    addAndMakeVisible (upperBandwidthSelection.get());

    lowerBandwidthSelection = std::make_unique<Label> ("LowerBandwidth", lastLowCutString);
    lowerBandwidthSelection->setEditable (true, false, false);
    lowerBandwidthSelection->addListener (this);
    lowerBandwidthSelection->setBounds (30, 10, 60, 20);
    addAndMakeVisible (lowerBandwidthSelection.get());
}

BandwidthInterface::~BandwidthInterface()
{
}

void BandwidthInterface::labelTextChanged (Label* label)
{
    if (! (editor->acquisitionIsActive) && board->foundInputSource())
    {
        if (label == upperBandwidthSelection.get())
        {
            Value val = label->getTextValue();
            double requestedValue = double (val.getValue());

            if (requestedValue < 100.0 || requestedValue > 20000.0 || requestedValue < lastLowCutString.getFloatValue())
            {
                CoreServices::sendStatusMessage ("Value out of range.");

                label->setText (lastHighCutString, dontSendNotification);

                return;
            }

            actualUpperBandwidth = board->setUpperBandwidth (requestedValue);

            LOGD ("Setting Upper Bandwidth to ", requestedValue);
            LOGD ("Actual Upper Bandwidth:  ", actualUpperBandwidth);
            label->setText (String (round (actualUpperBandwidth * 10.f) / 10.f), dontSendNotification);
        }
        else
        {
            Value val = label->getTextValue();
            double requestedValue = double (val.getValue());

            if (requestedValue < 0.1 || requestedValue > 500.0 || requestedValue > lastHighCutString.getFloatValue())
            {
                CoreServices::sendStatusMessage ("Value out of range.");

                label->setText (lastLowCutString, dontSendNotification);

                return;
            }

            actualLowerBandwidth = board->setLowerBandwidth (requestedValue);

            LOGD ("Setting Lower Bandwidth to ", requestedValue);
            LOGD ("Actual Lower Bandwidth:  ", actualLowerBandwidth);

            label->setText (String (round (actualLowerBandwidth * 10.f) / 10.f), dontSendNotification);
        }
    }
    else if (editor->acquisitionIsActive)
    {
        CoreServices::sendStatusMessage ("Can't change bandwidth while acquisition is active!");
        if (label == upperBandwidthSelection.get())
            label->setText (lastHighCutString, dontSendNotification);
        else
            label->setText (lastLowCutString, dontSendNotification);
        return;
    }
}

void BandwidthInterface::setLowerBandwidth (double value)
{
    actualLowerBandwidth = board->setLowerBandwidth (value);
    lowerBandwidthSelection->setText (String (round (actualLowerBandwidth * 10.f) / 10.f), dontSendNotification);
}

void BandwidthInterface::setUpperBandwidth (double value)
{
    actualUpperBandwidth = board->setUpperBandwidth (value);
    upperBandwidthSelection->setText (String (round (actualUpperBandwidth * 10.f) / 10.f), dontSendNotification);
}

double BandwidthInterface::getLowerBandwidth()
{
    return actualLowerBandwidth;
}

double BandwidthInterface::getUpperBandwidth()
{
    return actualUpperBandwidth;
}

void BandwidthInterface::paint (Graphics& g)
{
    g.setColour (findColour (ThemeColours::defaultText));

    g.setFont (Font ("Small Text", 10, Font::plain));

    g.drawText (name, 0, 0, 200, 15, Justification::left, false);

    g.drawText ("Low:", 0, 11, 200, 20, Justification::left, false);

    g.drawText ("High:", 0, 26, 200, 20, Justification::left, false);
}

// Sample rate Options --------------------------------------------------------------------

SampleRateInterface::SampleRateInterface (AcquisitionBoard* board_,
                                          DeviceEditor* editor_) : board (board_),
                                                                   editor (editor_)
{
    name = "Sample Rate";

    Array<int> sampleRates = board->getAvailableSampleRates();

    rateSelection = std::make_unique<ComboBox> ("Sample Rate");
    rateSelection->addItemList (sampleRateOptions, 1);
    for (auto sampleRate : sampleRates)
    {
        int numDecimals = sampleRate < 10000 ? 2 : 1;

        rateSelection->addItem (String (float (sampleRate) / 1000.0f, numDecimals) + " kS/s", sampleRate);
    }

    rateSelection->setSelectedId (sampleRates.getLast(), dontSendNotification);
    rateSelection->addListener (this);
    rateSelection->setBounds (0, 12, 80, 20);
    addAndMakeVisible (rateSelection.get());
}

SampleRateInterface::~SampleRateInterface()
{
}

void SampleRateInterface::comboBoxChanged (ComboBox* cb)
{
    if (! (editor->acquisitionIsActive) && board->foundInputSource())
    {
        if (cb == rateSelection.get())
        {
            board->setSampleRate (cb->getSelectedId());

            LOGD ("Setting sample rate to ", cb->getSelectedId());

            CoreServices::updateSignalChain (editor);
        }
    }
}

int SampleRateInterface::getSelectedId()
{
    return rateSelection->getSelectedId();
}

void SampleRateInterface::setSelectedId (int id)
{
    rateSelection->setSelectedId (id);
}

String SampleRateInterface::getText()
{
    return rateSelection->getText();
}

void SampleRateInterface::paint (Graphics& g)
{
    g.setColour (findColour (ThemeColours::defaultText));

    g.setFont (Font ("Small Text", 10, Font::plain));

    g.drawText (name, 0, 0, 80, 15, Justification::left, false);
}

// Headstage Options --------------------------------------------------------------------

HeadstageOptionsInterface::HeadstageOptionsInterface (AcquisitionBoard* board_,
                                                      DeviceEditor* editor_,
                                                      int hsNum) : isEnabled (false),
                                                                   board (board_),
                                                                   editor (editor_)
{
    switch (hsNum)
    {
        case 0:
            name = "A";
            break;
        case 1:
            name = "B";
            break;
        case 2:
            name = "C";
            break;
        case 3:
            name = "D";
            break;
        default:
            name = "X";
    }

    hsNumber1 = hsNum * 2; // data stream 1
    hsNumber2 = hsNumber1 + 1; // data stream 2

    channelsOnHs1 = 0;
    channelsOnHs2 = 0;

    hsButton1 = std::make_unique<UtilityButton> (" ");
    hsButton1->setRadius (3.0f);
    hsButton1->setBounds (23, 1, 20, 17);
    hsButton1->setEnabledState (false);
    hsButton1->setCorners (true, false, true, false);
    hsButton1->addListener (this);
    addAndMakeVisible (hsButton1.get());

    hsButton2 = std::make_unique<UtilityButton> (" ");
    hsButton2->setRadius (3.0f);
    hsButton2->setBounds (43, 1, 20, 17);
    hsButton2->setEnabledState (false);
    hsButton2->setCorners (false, true, false, true);
    hsButton2->addListener (this);
    addAndMakeVisible (hsButton2.get());

    checkEnabledState();
}

HeadstageOptionsInterface::~HeadstageOptionsInterface()
{
}

void HeadstageOptionsInterface::setEnabled(bool state)
{
    hsButton1->setEnabledState (state);
    hsButton2->setEnabledState (state);
}

void HeadstageOptionsInterface::checkEnabledState()
{
    LOGD ("Checking enabled state of HS ", hsNumber1, " and HS ", hsNumber2);
    
    isEnabled = (board->isHeadstageEnabled (hsNumber1) || board->isHeadstageEnabled (hsNumber2));

    LOGD ("Is enabled: ", isEnabled);

    if (board->isHeadstageEnabled (hsNumber1))
    {
        channelsOnHs1 = board->getActiveChannelsInHeadstage (hsNumber1);
        hsButton1->setLabel (String (channelsOnHs1));
        hsButton1->setEnabledState (true);
        hsButton1->setToggleState (true, false);
    }
    else
    {
        channelsOnHs1 = 0;
        hsButton1->setLabel (" ");
        hsButton1->setEnabledState (false);
        hsButton1->setToggleState (false, false);
    }

    LOGD ("Channels on HS1: ", channelsOnHs1);

    if (board->isHeadstageEnabled (hsNumber2))
    {
        channelsOnHs2 = board->getActiveChannelsInHeadstage (hsNumber2);
        hsButton2->setLabel (String (channelsOnHs2));
        hsButton2->setEnabledState (true);
        hsButton2->setToggleState (true, false);
    }
    else
    {
        channelsOnHs2 = 0;
        hsButton2->setLabel (" ");
        hsButton2->setEnabledState (false);
        hsButton2->setToggleState (false, false);
    }

    LOGD ("Channels on HS2: ", channelsOnHs1);

    repaint();
}

void HeadstageOptionsInterface::buttonClicked (Button* button)
{
    if (! (editor->acquisitionIsActive) && board->foundInputSource())
    {
        if ((button == hsButton1.get()) && (board->getChannelsInHeadstage (hsNumber1) == 32))
        {
            if (channelsOnHs1 == 32)
                channelsOnHs1 = 16;
            else
                channelsOnHs1 = 32;

            hsButton1->setLabel (String (channelsOnHs1));
            board->setNumHeadstageChannels (hsNumber1, channelsOnHs1);
            CoreServices::updateSignalChain (editor);
        }
        else if ((button == hsButton2.get()) && (board->getChannelsInHeadstage (hsNumber2) == 32))
        {
            if (channelsOnHs2 == 32)
                channelsOnHs2 = 16;
            else
                channelsOnHs2 = 32;

            hsButton2->setLabel (String (channelsOnHs2));
            board->setNumHeadstageChannels (hsNumber2, channelsOnHs2);
            CoreServices::updateSignalChain (editor);
        }
   
    }
}

bool HeadstageOptionsInterface::is32Channel (int hsIndex)
{
    if (hsIndex == 0)
        return channelsOnHs1 == 32;

    else if (hsIndex == 1)
        return channelsOnHs2 == 32;
}

void HeadstageOptionsInterface::set32Channel (int hsIndex, bool is32Channel)
{
    if (hsIndex == 0 && (board->getChannelsInHeadstage (hsNumber1) == 32))
    {
        if (is32Channel)
            channelsOnHs1 = 32;
        else
            channelsOnHs1 = 16;

        hsButton1->setLabel (String (channelsOnHs1));
        board->setNumHeadstageChannels (hsNumber1, channelsOnHs1);
    }

    else if (hsIndex == 1 && (board->getChannelsInHeadstage (hsNumber2) == 32))
    {
        if (is32Channel)
            channelsOnHs2 = 32;
        else
            channelsOnHs2 = 16;

        hsButton2->setLabel (String (channelsOnHs2));
        board->setNumHeadstageChannels (hsNumber1, channelsOnHs1);
    }
}

void HeadstageOptionsInterface::paint (Graphics& g)
{
    g.setColour (findColour (ThemeColours::componentBackground).darker(0.2f));

    g.fillRoundedRectangle (5, 0, getWidth() - 10, getHeight(), 4.0f);

    g.setColour (findColour (ThemeColours::defaultText));

    g.setFont (Font ("Small Text", 15, Font::plain));

    g.drawText (name, 10, 2, 200, 15, Justification::left, false);
}

// (Direct OpalKelly) Audio Options --------------------------------------------------------------------

AudioInterface::AudioInterface (AcquisitionBoard* board_,
                                DeviceEditor* editor_) : board (board_),
                                                         editor (editor_)
{
    name = "Noise";

    lastNoiseSlicerString = "0";

    actualNoiseSlicerLevel = 0.0f;

    noiseSlicerLevelSelection = std::make_unique<Label> ("Noise Slicer", lastNoiseSlicerString); // this is currently set in DeviceThread, the cleaner would be to set it here again
    noiseSlicerLevelSelection->setEditable (true, false, false);
    noiseSlicerLevelSelection->addListener (this);
    noiseSlicerLevelSelection->setBounds (45, 6, 35, 20);
    addAndMakeVisible (noiseSlicerLevelSelection.get());
}

AudioInterface::~AudioInterface()
{
}

void AudioInterface::labelTextChanged (Label* label)
{
    if (board->foundInputSource())
    {
        if (label == noiseSlicerLevelSelection.get())
        {
            Value val = label->getTextValue();
            int requestedValue = int (val.getValue()); // Note that it might be nice to translate to actual uV levels (16*value)

            if (requestedValue < 0 || requestedValue > 127)
            {
                CoreServices::sendStatusMessage ("Value out of range.");

                label->setText (lastNoiseSlicerString, dontSendNotification);

                return;
            }

            actualNoiseSlicerLevel = board->setNoiseSlicerLevel (requestedValue);

            LOGD ("Setting Noise Slicer Level to ", requestedValue);
            label->setText (String ((roundFloatToInt) (actualNoiseSlicerLevel)), dontSendNotification);
        }
    }
    else
    {
        Value val = label->getTextValue();
        int requestedValue = int (val.getValue()); // Note that it might be nice to translate to actual uV levels (16*value)
        if (requestedValue < 0 || requestedValue > 127)
        {
            CoreServices::sendStatusMessage ("Value out of range.");
            label->setText (lastNoiseSlicerString, dontSendNotification);
            return;
        }
    }
}

void AudioInterface::setNoiseSlicerLevel (int value)
{
    actualNoiseSlicerLevel = board->setNoiseSlicerLevel (value);
    noiseSlicerLevelSelection->setText (String (roundFloatToInt (actualNoiseSlicerLevel)), dontSendNotification);
}

int AudioInterface::getNoiseSlicerLevel()
{
    return actualNoiseSlicerLevel;
}

void AudioInterface::paint (Graphics& g)
{
    g.setColour (findColour (ThemeColours::defaultText));
    g.setFont (Font ("Small Text", 10, Font::plain));
    g.drawText (name, 0, 0, 200, 15, Justification::left, false);
    g.drawText ("Slicer:", 0, 10, 200, 15, Justification::left, false);
}

// Clock Divider options
ClockDivideInterface::ClockDivideInterface (AcquisitionBoard* board_,
                                            DeviceEditor* editor_) : name ("Clock"),
                                                                     lastDivideRatioString ("1"),
                                                                     board (board_),
                                                                     editor (editor_),
                                                                     actualDivideRatio (1)

{
    divideRatioSelection = std::make_unique<Label> ("Clock Divider", lastDivideRatioString);
    divideRatioSelection->setEditable (true, false, false);
    divideRatioSelection->addListener (this);
    divideRatioSelection->setBounds (45, 6, 35, 20);
    addAndMakeVisible (divideRatioSelection.get());
}

void ClockDivideInterface::labelTextChanged (Label* label)
{
    if (board->foundInputSource())
    {
        if (label == divideRatioSelection.get())
        {
            Value val = label->getTextValue();
            int requestedValue = int (val.getValue());

            if (requestedValue < 1 || requestedValue > 65534)
            {
                CoreServices::sendStatusMessage ("Value must be between 1 and 65534.");
                label->setText (lastDivideRatioString, dontSendNotification);
                return;
            }

            actualDivideRatio = board->setClockDivider (requestedValue);
            lastDivideRatioString = String (actualDivideRatio);

            LOGD ("Setting clock divide ratio to ", actualDivideRatio);
            label->setText (lastDivideRatioString, dontSendNotification);
        }
    }
}

void ClockDivideInterface::setClockDivideRatio (int value)
{
    actualDivideRatio = board->setClockDivider (value);
    divideRatioSelection->setText (String (actualDivideRatio), dontSendNotification);
}

void ClockDivideInterface::paint (Graphics& g)
{
    g.setColour (findColour (ThemeColours::defaultText));
    g.setFont (Font ("Small Text", 10, Font::plain));
    g.drawText (name, 0, 0, 200, 15, Justification::left, false);
    g.drawText ("Divider: ", 0, 10, 200, 15, Justification::left, false);
}

// DSP Options --------------------------------------------------------------------

DSPInterface::DSPInterface (AcquisitionBoard* board_,
                            DeviceEditor* editor_) : board (board_),
                                                     editor (editor_)
{
    name = "DSP";

    dspOffsetSelection = std::make_unique<Label> ("DspOffsetSelection",
                                                  String (round (board->getDspCutoffFreq() * 10.f) / 10.f));
    dspOffsetSelection->setEditable (true, false, false);
    dspOffsetSelection->addListener (this);
    dspOffsetSelection->setBounds (0, 0, 35, 20);
    addAndMakeVisible (dspOffsetSelection.get());
}

DSPInterface::~DSPInterface()
{
}

void DSPInterface::labelTextChanged (Label* label)
{
    if (! (editor->acquisitionIsActive) && board->foundInputSource())
    {
        if (label == dspOffsetSelection.get())
        {
            Value val = label->getTextValue();
            double requestedValue = double (val.getValue());

            actualDspCutoffFreq = board->setDspCutoffFreq (requestedValue);

            LOGD ("Setting DSP Cutoff Freq to ", requestedValue);
            LOGD ("Actual DSP Cutoff Freq:  ", actualDspCutoffFreq);
            label->setText (String (round (actualDspCutoffFreq * 10.f) / 10.f), dontSendNotification);
        }
    }
    else if (editor->acquisitionIsActive)
    {
        CoreServices::sendStatusMessage ("Can't change DSP cutoff while acquisition is active!");
    }
}

void DSPInterface::setDspCutoffFreq (double value)
{
    actualDspCutoffFreq = board->setDspCutoffFreq (value);
    dspOffsetSelection->setText (String (round (actualDspCutoffFreq * 10.f) / 10.f), dontSendNotification);
}

double DSPInterface::getDspCutoffFreq()
{
    return actualDspCutoffFreq;
}

void DSPInterface::paint (Graphics& g)
{
    g.setColour (findColour (ThemeColours::defaultText));
    g.setFont (Font ("Small Text", 10, Font::plain));
}
