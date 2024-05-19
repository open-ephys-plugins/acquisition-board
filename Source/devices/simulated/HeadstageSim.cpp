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

#include "HeadstageSim.h"

HeadstageSim::HeadstageSim()
{

}


HeadstageSim::~HeadstageSim()
{

}


int HeadstageSim::getNumActiveChannels() const
{
    return 64;
}


String HeadstageSim::getChannelName (int ch) const
{
    return "CH" + ch;
}


String HeadstageSim::getStreamPrefix() const
{
    return "A1";
}


bool HeadstageSim::hasValidImpedance (int ch) const
{
    return true;
}


float HeadstageSim::getImpedanceMagnitude (int ch) const
{
    return 1.0f;
}


float HeadstageSim::getImpedancePhase (int ch) const
{
    return 1.0f;
}


void HeadstageSim::generateChannelNames (ChannelNamingScheme scheme)
{

}
