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

#include "DeviceThread.h"
#include "DeviceEditor.h"

#include "devices/oni/AcqBoardONI.h"
#include "devices/opalkelly/AcqBoardOpalKelly.h"
#include "devices/simulated/AcqBoardSim.h"

/** Set to true to test simulation mode with boards connected */
static const bool forceSimulationMode = false;

BoardType DeviceThread::boardType = ACQUISITION_BOARD; // initialize static member

DeviceThread::DeviceThread (SourceNode* sn, BoardType boardType_) : DataThread (sn)
{
    boardType = boardType_;

    sourceBuffers.add (new DataBuffer (2, 10000)); // start with 2 channels and automatically resize
    acquisitionBoard.reset (detectBoard()); // detect which board is connected

    if (acquisitionBoard == nullptr) // no board detected, and not running in simulation mode
    {
        deviceFound = false;
        return;
    }

    deviceFound = acquisitionBoard->initializeBoard(); // returns false if initialization fails

    if (! deviceFound)
        return;

    acquisitionBoard->scanPorts(); // check for connected headstages
}

DeviceThread::~DeviceThread()
{
}

DataThread* DeviceThread::createDataThread (SourceNode* sn)
{
    return new DeviceThread (sn, boardType);
}

std::unique_ptr<GenericEditor> DeviceThread::createEditor (SourceNode* sn)
{
    std::unique_ptr<DeviceEditor> editor = std::make_unique<DeviceEditor> (sn, acquisitionBoard.get());

    return editor;
}

AcquisitionBoard* DeviceThread::detectBoard()
{
    if (forceSimulationMode)
    {
        return new AcqBoardSim();
    }

    std::unique_ptr<AcqBoardOpalKelly> opalKellyBoard = std::make_unique<AcqBoardOpalKelly>();

    if (opalKellyBoard->detectBoard())
    {
        return opalKellyBoard.release();
    }
    else
    {
        opalKellyBoard.reset();
    }

    std::unique_ptr<AcqBoardONI> oniBoard = std::make_unique<AcqBoardONI>();

    if (oniBoard->detectBoard())
    {
        return oniBoard.release();
    }
    else
    {
        oniBoard.reset();
    }

    AlertWindow alert ("No device found",
                       "An acquisition board could not be found. Make sure your device is connected properly and powered on. "
                       "If the problem persists, refer to the troubleshooting guide linked below.",
                       MessageBoxIconType::WarningIcon);

    auto hyperlink = std::make_unique<HyperlinkButton> ("Troubleshooting Guide", URL ("https://open-ephys.github.io/acq-board-docs/User-Manual/Troubleshooting.html"));
    hyperlink->setName ("");
    hyperlink->setFont (FontOptions ("Inter", "Medium", 18.0f).withUnderline (true), false);
    hyperlink->setTooltip ("https://open-ephys.github.io/acq-board-docs/User-Manual/Troubleshooting.html");
    hyperlink->setJustificationType (Justification::centred);
    hyperlink->changeWidthToFitText();
    hyperlink->setSize (hyperlink->getWidth() + 20, 20);
    hyperlink->setColour (HyperlinkButton::ColourIds::textColourId, Colours::deepskyblue);

    alert.addCustomComponent (hyperlink.get());

    alert.addTextBlock ("\t\tDo you want to run this plugin in simulation mode?");

    alert.addButton ("Yes", 0, KeyPress (KeyPress::returnKey, 0, 0));
    alert.addButton ("No", 1, KeyPress (KeyPress::escapeKey, 0, 0));
    int response = alert.runModalLoop();

    if (response == 0)
    {
        return new AcqBoardSim();
    }

    // if we reach this point, we have no device connected
    return nullptr;
}

bool DeviceThread::foundInputSource()
{
    return deviceFound;
}

bool DeviceThread::isReady()
{
    return acquisitionBoard->isReady();
}

bool DeviceThread::startAcquisition()
{
    return acquisitionBoard->startAcquisition();
}

bool DeviceThread::stopAcquisition()
{
    return acquisitionBoard->stopAcquisition();
}

void DeviceThread::updateSettings (OwnedArray<ContinuousChannel>* continuousChannels,
                                   OwnedArray<EventChannel>* eventChannels,
                                   OwnedArray<SpikeChannel>* spikeChannels,
                                   OwnedArray<DataStream>* sourceStreams,
                                   OwnedArray<DeviceInfo>* devices,
                                   OwnedArray<ConfigurationObject>* configurationObjects)
{
    if (! deviceFound)
        return;

    continuousChannels->clear();
    eventChannels->clear();
    spikeChannels->clear();
    sourceStreams->clear();
    devices->clear();
    configurationObjects->clear();
    sourceBuffers.clear();

    if (acquisitionBoard->getNumChannels() > 0)
    {
        sourceBuffers.add (acquisitionBoard->getBuffer());

        bool generatesTimestamps = acquisitionBoard->getBoardType() == AcquisitionBoard::BoardType::ONI;

        DataStream::Settings dataStreamSettings {
            "acquisition_board",
            "Continuous and event data from an Open Ephys Acquisition Board",
            "acq-board.rhythm",
            static_cast<float> (acquisitionBoard->getSampleRate()),
            generatesTimestamps
        };

        DataStream* stream = new DataStream (dataStreamSettings);

        sourceStreams->add (stream);

        for (auto headstage : acquisitionBoard->getHeadstages())
        {
            for (int ch = 0; ch < headstage->getNumActiveChannels(); ch++)
            {
                ContinuousChannel::Settings channelSettings {
                    ContinuousChannel::ELECTRODE,
                    headstage->getChannelName (ch),
                    "Headstage channel from an Open Ephys Acquisition Board",
                    "acq-board.rhythm.continuous.ephys",

                    acquisitionBoard->getBitVolts (ContinuousChannel::Type::ELECTRODE),

                    stream
                };

                continuousChannels->add (new ContinuousChannel (channelSettings));
                continuousChannels->getLast()->setUnits ("uV");

                if (headstage->hasValidImpedance (ch))
                {
                    continuousChannels->getLast()->impedance.magnitude = headstage->getImpedanceMagnitude (ch);
                    continuousChannels->getLast()->impedance.phase = headstage->getImpedancePhase (ch);
                }
            }
        }

        if (acquisitionBoard->areAuxChannelsEnabled())
        {
            const ContinuousChannel::InputRange accelerationRange { -10.0f, 10.0f };

            for (auto headstage : acquisitionBoard->getHeadstages())
            {
                for (int ch = 0; ch < 3; ch++)
                {
                    ContinuousChannel::Settings channelSettings {
                        ContinuousChannel::AUX,
                        headstage->getStreamPrefix() + "_AUX" + String (ch + 1),
                        "Aux input channel from an Open Ephys Acquisition Board",
                        "acq-board.rhythm.continuous.aux",

                        acquisitionBoard->getBitVolts (ContinuousChannel::Type::AUX),

                        stream
                    };

                    continuousChannels->add (new ContinuousChannel (channelSettings));
                    continuousChannels->getLast()->setUnits ("mV");
                    continuousChannels->getLast()->inputRange = accelerationRange;
                }
            }
        }

        if (acquisitionBoard->areAdcChannelsEnabled())
        {
            for (int ch = 0; ch < 8; ch++)
            {
                String name = "ADC" + String (ch + 1);

                ContinuousChannel::Settings channelSettings {
                    ContinuousChannel::ADC,
                    name,
                    "ADC input channel from an Open Ephys Acquisition Board",
                    "acq-board.rhythm.continuous.adc",

                    acquisitionBoard->getBitVolts (ContinuousChannel::Type::ADC),

                    stream
                };

                continuousChannels->add (new ContinuousChannel (channelSettings));
                continuousChannels->getLast()->setUnits ("V");
            }
        }

        EventChannel::Settings settings {
            EventChannel::Type::TTL,
            "Acquisition Board TTL Input",
            "Events on digital input lines of an Open Ephys Acquisition Board",
            "acq-board.rhythm.events",
            stream,
            8
        };

        eventChannels->add (new EventChannel (settings));
    }

    OwnedArray<DataStream> otherStreams;
    OwnedArray<ContinuousChannel> otherChannels;
    OwnedArray<DataBuffer> otherBuffers;
    acquisitionBoard->createCustomStreams (otherBuffers);
    sourceBuffers.addArray (otherBuffers);
    otherBuffers.clearQuick (false);
    acquisitionBoard->updateCustomStreams (otherStreams, otherChannels);
    sourceStreams->addArray (otherStreams);
    otherStreams.clearQuick (false);
    continuousChannels->addArray (otherChannels);
    otherChannels.clearQuick (false);
}

void DeviceThread::handleBroadcastMessage (const String& msg, const int64 messageTimeMilliseconds)
{
    StringArray parts = StringArray::fromTokens (msg, " ", "");

    if (parts[0].equalsIgnoreCase ("ACQBOARD"))
    {
        if (parts.size() > 1)
        {
            String command = parts[1];

            if (command.equalsIgnoreCase ("TRIGGER"))
            {
                if (parts.size() == 4)
                {
                    int ttlLine = parts[2].getIntValue() - 1;

                    if (ttlLine < 0 || ttlLine > 7)
                        return;

                    int eventDurationMs = parts[3].getIntValue();

                    if (eventDurationMs < 10 || eventDurationMs > 5000)
                        return;

                    acquisitionBoard->triggerDigitalOutput (ttlLine, eventDurationMs);
                }
            }
        }
    }
}
