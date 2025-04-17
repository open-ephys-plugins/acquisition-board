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

#include "MemoryMonitorUsage.h"

MemoryMonitorUsage::MemoryMonitorUsage (GenericProcessor* p)
    : LevelMonitor (p)
{
    percentMemoryUsed = 0.0f;
}

void MemoryMonitorUsage::timerCallback()
{
    setFillPercentage (std::log (percentMemoryUsed + 1) / maxLogarithmicValue);
    repaint();
}

void MemoryMonitorUsage::setPercentMemoryUsed (float memoryUsed)
{
    percentMemoryUsed = memoryUsed;
}

void MemoryMonitorUsage::startAcquisition()
{
    startTimerHz (TimerFrequencyHz);
}

void MemoryMonitorUsage::stopAcquisition()
{
    stopTimer();
    setFillPercentage (0.0f);
    repaint();
}
