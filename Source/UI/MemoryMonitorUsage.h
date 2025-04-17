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

#pragma once

#include <VisualizerEditorHeaders.h>

/*
	Tracks the MemoryMonitor usage while data acquisition is running
*/
class MemoryMonitorUsage : public LevelMonitor
{
public:
    MemoryMonitorUsage (GenericProcessor*);

    void timerCallback() override;

    void startAcquisition();

    void stopAcquisition();

    void setPercentMemoryUsed (float memoryUsed);

private:
    std::atomic<float> percentMemoryUsed;

    // NB: Calculate the maximum logarithmic value to convert from linear scale (x: 0-100) to logarithmic scale (y: 0-1)
    //	   using the following equation: y = log_e(x + 1) / log_e(x_max + 1);
    const float maxLogarithmicValue = std::log (101);

    const int TimerFrequencyHz = 10;

    JUCE_LEAK_DETECTOR (MemoryMonitorUsage);
};
