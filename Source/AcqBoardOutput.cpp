/*
    ------------------------------------------------------------------

    This file is part of the Open Ephys GUI
    Copyright (C) 2016 Open Ephys

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

#include "AcqBoardOutput.h"
#include "AcqBoardOutputEditor.h"

#include <stdio.h>

AcqBoardOutput::AcqBoardOutput()
    : GenericProcessor ("Acq Board Output"),
      gateIsOpen (true)
{
}

void AcqBoardOutput::registerParameters()
{
    addTtlLineParameter (Parameter::STREAM_SCOPE, "ttl_out", "TTL Out", "The digital output to trigger");

    addTtlLineParameter (Parameter::STREAM_SCOPE, "trigger_line", "Trigger Line", "The TTL bit for triggering output");

    addTtlLineParameter (Parameter::STREAM_SCOPE, "gate_line", "Gate Line", "The TTL bit for gating the output", 8, false, true);
    dataStreamParameters.getLast()->currentValue = -1;

    addNotificationParameter (Parameter::PROCESSOR_SCOPE, "trigger", "Trigger", "Manually triggers output", false);

    addFloatParameter (
        Parameter::PROCESSOR_SCOPE,
        "event_duration",
        "Duration",
        "The amount of time (in ms) the output stays high",
        "ms",
        500,
        100,
        2000,
        1.0);
}

AudioProcessorEditor* AcqBoardOutput::createEditor()
{
    editor = std::make_unique<AcqBoardOutputEditor> (this);
    return editor.get();
}

void AcqBoardOutput::triggerOutput()
{
    getParameter ("trigger")->setNextValue (true);
}

void AcqBoardOutput::parameterValueChanged (Parameter* param)
{
    if (param->getName().equalsIgnoreCase ("trigger"))
    {
        DataStream* stream = nullptr;
        if (getEditor() != nullptr)
        {
            stream = getDataStream (getEditor()->getCurrentStream());
        }
        else if (dataStreams.size() > 0)
        {
            stream = dataStreams.getFirst();
        }

        if (stream != nullptr)
        {
            auto ttlOut = ((TtlLineParameter*) stream->getParameter ("ttl_out"))->getSelectedLine() + 1;
            broadcastMessage ("ACQBOARD TRIGGER "
                              + String (ttlOut)
                              + " "
                              + getParameter ("event_duration")->getValue().toString());
        }
    }
    else if (param->getName().equalsIgnoreCase ("gate_line"))
    {
        TtlLineParameter* p = (TtlLineParameter*) param;
        if (p->getSelectedLine() >= 0)
            gateIsOpen = false;
        else
            gateIsOpen = true;
    }
}

void AcqBoardOutput::handleTTLEvent (TTLEventPtr event)
{
    const int eventBit = event->getLine();
    DataStream* stream = getDataStream (event->getStreamId());

    //std::cout << "Event on line " << eventBit << " for stream " << stream->getStreamId() << std::endl;

    if (eventBit == int ((*stream)["gate_line"]))
    {
        gateIsOpen = event->getState();
    }

    if (gateIsOpen)
    {
        if (eventBit == int ((*stream)["trigger_line"]))
        {
            if (event->getState())
            {
                auto ttlOut = ((TtlLineParameter*) stream->getParameter ("ttl_out"))->getSelectedLine() + 1;
                String msg = "ACQBOARD TRIGGER "
                             + String (ttlOut)
                             + " "
                             + getParameter ("event_duration")->getValue().toString();
                broadcastMessage (msg);

                //std::cout << "Sending message " << msg << std::endl;
            }
        }
    }
}

void AcqBoardOutput::process (AudioBuffer<float>& buffer)
{
    checkForEvents();
}
