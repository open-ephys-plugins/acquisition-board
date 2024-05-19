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

#ifndef __ACQBOARDOUTPUTEDITOR_H_28EB4CC9__
#define __ACQBOARDOUTPUTEDITOR_H_28EB4CC9__

#include <EditorHeaders.h>

/**

    User interface for the AcqBoardOutput processor.

    @see AcqBoardOutput

*/

class AcqBoardOutput;

class AcqBoardOutputEditor : public GenericEditor,
                             public Button::Listener

{
public:
    /** Constructor*/
    AcqBoardOutputEditor (GenericProcessor* parentNode);

    /** Destructor*/
    ~AcqBoardOutputEditor() {}

    /** Responds to trigger button press*/
    void buttonClicked (Button* button);

private:
    std::unique_ptr<UtilityButton> triggerButton;

    AcqBoardOutput* board;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (AcqBoardOutputEditor);
};

#endif // __AcqBoardOutputEDITOR_H_28EB4CC9__
