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

#include "AcqBoardOutputEditor.h"
#include "AcqBoardOutput.h"
#include <stdio.h>

AcqBoardOutputEditor::AcqBoardOutputEditor (GenericProcessor* parentNode)
    : GenericEditor (parentNode)

{
    desiredWidth = 250;

    board = (AcqBoardOutput*) parentNode;

    addComboBoxParameterEditor (Parameter::STREAM_SCOPE, "ttl_out", 10, 30);
    addComboBoxParameterEditor (Parameter::STREAM_SCOPE, "trigger_line", 10, 76);
    addComboBoxParameterEditor (Parameter::STREAM_SCOPE, "gate_line", 100, 76);
    addTextBoxParameterEditor (Parameter::GLOBAL_SCOPE, "event_duration", 100, 30);

    triggerButton = std::make_unique<UtilityButton> ("Trigger");
    triggerButton->setBounds (190, 95, 55, 25);
    triggerButton->addListener (this);
    addAndMakeVisible (triggerButton.get());
}

void AcqBoardOutputEditor::buttonClicked (Button* button)
{
    if (button == triggerButton.get())
    {
        AcqBoardOutput* processor = (AcqBoardOutput*) getProcessor();
        processor->triggerOutput (getCurrentStream());
    }
}
