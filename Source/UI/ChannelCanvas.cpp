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

#include "ChannelCanvas.h"

#include "ChannelList.h"

/**********************************************/

ChannelCanvas::ChannelCanvas (AcquisitionBoard* board_,
                              DeviceEditor* editor_) : board (board_),
                                                       editor (editor_)
{
    channelViewport = std::make_unique<Viewport>();

    channelList = std::make_unique<ChannelList> (board, editor);

    channelViewport->setViewedComponent (channelList.get(), false);
    channelViewport->setScrollBarsShown (true, true);
    channelViewport->setScrollBarThickness (10);
    addAndMakeVisible (channelViewport.get());

    update();

    resized();
}

void ChannelCanvas::paint (Graphics& g)
{
    g.fillAll (findColour (ThemeColours::componentBackground));
}

void ChannelCanvas::refresh()
{
    repaint();
}

void ChannelCanvas::refreshState()
{
    resized();
}

void ChannelCanvas::updateSettings()
{
    channelList->update();
    resized();
}

void ChannelCanvas::updateAsync()
{
    Timer::callAfterDelay (5, [this]
                           { update(); });
}

void ChannelCanvas::beginAnimation()
{
    channelList->disableAll();
}

void ChannelCanvas::endAnimation()
{
    channelList->enableAll();
}

void ChannelCanvas::resized()
{
    int scrollBarThickness = channelViewport->getScrollBarThickness();

    channelViewport->setBounds (0, 0, getWidth(), getHeight());

    channelList->setBounds (0, 0, getWidth() - scrollBarThickness, 200 + 22 * channelList->getMaxChannels());
}
