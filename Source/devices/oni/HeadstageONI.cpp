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

#include "HeadstageONI.h"

HeadstageONI::HeadstageONI (Rhd2000ONIBoard::BoardDataSource dataSource_,
                            int MAX_H) : Headstage (int (dataSource_)),
                                         dataSource (dataSource_),
                                         MAX_NUM_HEADSTAGES (MAX_H),
                                         numStreams (0),
                                         channelsPerStream (32),
                                         halfChannels (false),
                                         streamIndex (-1),
                                         m_hasBno (false)
{
}

int HeadstageONI::getNumStreams() const
{
    return numStreams;
}

void HeadstageONI::setNumStreams (int num)
{
    LOGD ("Headstage ", prefix, " setting num streams to ", num);

    if (num == 2)
        halfChannels = false;

    if (numStreams != num)
    {
        numStreams = num;

        generateChannelNames (channelNamingScheme);
    }
}

void HeadstageONI::setHasBno (bool hasBno)
{
    m_hasBno = hasBno;
}

void HeadstageONI::setChannelsPerStream (int nchan)
{
    LOGD ("Headstage ", prefix, " setting channels per stream to ", nchan);

    if (channelsPerStream != nchan)
    {
        channelsPerStream = nchan;

        generateChannelNames (channelNamingScheme);
    }
}

void HeadstageONI::setFirstStreamIndex (int streamIndex_)
{
    streamIndex = streamIndex_;
}

int HeadstageONI::getStreamIndex (int offset) const
{
    return streamIndex + offset;
}

int HeadstageONI::getNumChannels() const
{
    return channelsPerStream * numStreams;
}

void HeadstageONI::setHalfChannels (bool half)
{
    if (getNumChannels() == 64)
        return;

    if (halfChannels != half)
    {
        halfChannels = half;

        generateChannelNames (channelNamingScheme);
    }
}

int HeadstageONI::getNumActiveChannels() const
{
    return (int) (getNumChannels() / (halfChannels ? 2 : 1));
}

Rhd2000ONIBoard::BoardDataSource HeadstageONI::getDataStream (int index) const
{
    if (index < 0 || index > 1)
        index = 0;
    return static_cast<Rhd2000ONIBoard::BoardDataSource> (dataSource + MAX_NUM_HEADSTAGES * index);
}

bool HeadstageONI::isConnected() const
{
    return numStreams > 0 || m_hasBno;
}

String HeadstageONI::getChannelName (int ch) const
{
    String name;

    if (ch > -1 && ch < channelNames.size())
        name = channelNames[ch];
    else
        name = " ";

    if (ch == 0)
        LOGD ("Headstage ", prefix, " channel ", ch, " name: ", name);

    return name;
}

void HeadstageONI::generateChannelNames (ChannelNamingScheme scheme)
{
    channelNamingScheme = scheme;

    channelNames.clear();

    switch (scheme)
    {
        case GLOBAL_INDEX:
            for (int i = 0; i < getNumActiveChannels(); i++)
            {
                channelNames.add ("CH" + String (firstChannelIndex + i + 1));
            }
            break;
        case STREAM_INDEX:
            for (int i = 0; i < getNumActiveChannels(); i++)
            {
                channelNames.add (prefix + "_CH" + String (i + 1));
            }
    }
}

bool HeadstageONI::hasValidImpedance (int ch) const
{
    if (ch < impedanceMagnitudes.size())
        return true;
    else
        return false;
}

void HeadstageONI::setImpedances (Impedances& impedances)
{
    impedanceMagnitudes.clear();
    impedancePhases.clear();

    for (int i = 0; i < impedances.streams.size(); i++)
    {
        if (impedances.streams[i] == streamIndex)
        {
            impedanceMagnitudes.add (impedances.magnitudes[i]);
            impedancePhases.add (impedances.phases[i]);
        }

        if (numStreams == 2 && impedances.streams[i] == streamIndex + 1)
        {
            impedanceMagnitudes.add (impedances.magnitudes[i]);
            impedancePhases.add (impedances.phases[i]);
        }
    }
}

float HeadstageONI::getImpedanceMagnitude (int channel) const
{
    if (channel < impedanceMagnitudes.size())
        return impedanceMagnitudes[channel];

    return 0.0f;
}

float HeadstageONI::getImpedancePhase (int channel) const
{
    if (channel < impedancePhases.size())
        return impedancePhases[channel];

    return 0.0f;
}