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
    addIntParameter (Parameter::STREAM_SCOPE, "ttl_out", "TTL Out", "The digital output to trigger", 1, 1, 8);
    addIntParameter (Parameter::STREAM_SCOPE, "trigger_line", "Trigger Line", "The TTL bit for triggering output", 1, 1, 16);
    addIntParameter (Parameter::STREAM_SCOPE, "gate_line", "Gate Line", "The TTL bit for gating the output", 0, 0, 16);
    addNotificationParameter (Parameter::GLOBAL_SCOPE, "trigger", "Trigger", "Manually triggers output", false);

    addFloatParameter (
        Parameter::GLOBAL_SCOPE,
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

void AcqBoardOutput::triggerOutput (uint16 streamId)
{
    DataStream* stream = getDataStream (streamId);

    stream->getParameter ("trigger")->setNextValue (true);
}

void AcqBoardOutput::parameterValueChanged (Parameter* param)
{
    if (param->getName().equalsIgnoreCase ("trigger"))
    {
        DataStream* stream = getDataStream (param->getStreamId());

        broadcastMessage ("ACQBOARD TRIGGER "
                          + (*stream)["ttl_out"].toString()
                          + " "
                          + (*stream)["event_duration"].toString());
    }
    else if (param->getName().equalsIgnoreCase ("gate_line"))
    {
        IntParameter* p = (IntParameter*) param;
        if (p->getIntValue() > 0)
            gateIsOpen = false;
        else
            gateIsOpen = true;
    }
}

void AcqBoardOutput::handleTTLEvent (TTLEventPtr event)
{
    const int eventBit = event->getLine() + 1;
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
                String msg = "ACQBOARD TRIGGER "
                             + (*stream)["ttl_out"].toString()
                             + " "
                             + (*stream)["event_duration"].toString();
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
