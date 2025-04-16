/*
    ------------------------------------------------------------------

    This file is part of the Open Ephys GUI
    Copyright (C) 2020 Open Ephys

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

#ifndef __DEVICETHREAD_H_2C4CBD67__
#define __DEVICETHREAD_H_2C4CBD67__

#include <DataThreadHeaders.h>

#include "devices/AcquisitionBoard.h"
#include "devices/Headstage.h"

enum BoardType
{
    ACQUISITION_BOARD
};

/**
    Communicates with any type of acquisition board

    @see DataThread, SourceNode
*/
class DeviceThread : public DataThread
{
public:
    /** Currently only ACQUISITION_BOARD is used */
    static BoardType boardType;

    /** Constructor; must specify the type of board used */
    DeviceThread (SourceNode* sn, BoardType boardType = ACQUISITION_BOARD);

    /** Destructor */
    ~DeviceThread();

    /** Creates the DataThread object */
    static DataThread* createDataThread (SourceNode* sn);

    /** Creates the UI for this plugin */
    std::unique_ptr<GenericEditor> createEditor (SourceNode* sn);

    /** Detects the type of board to use */
    AcquisitionBoard* detectBoard();

    /** Fills the DataBuffer with incoming data -- not used 
        Instead, the AcquisitionBoard object fills the buffer
    */
    bool updateBuffer() override { return true; }

    /** Informs Source Node whether a device is available */
    bool foundInputSource() override;

    bool isReady() override;

    /** Initializes data transfer*/
    bool startAcquisition() override;

    /** Stops data transfer */
    bool stopAcquisition() override;

    /* Passes the processor's info objects to DataThread, to allow them to be configured */
    void updateSettings (OwnedArray<ContinuousChannel>* continuousChannels,
                         OwnedArray<EventChannel>* eventChannels,
                         OwnedArray<SpikeChannel>* spikeChannels,
                         OwnedArray<DataStream>* sourceStreams,
                         OwnedArray<DeviceInfo>* devices,
                         OwnedArray<ConfigurationObject>* configurationObjects) override;

    /** Allow the thread to respond to messages sent by other plugins */
    void handleBroadcastMessage (const String& msg, const int64 messageTimeMilliseconds) override;

private:
    /** Pointer to device */
    std::unique_ptr<AcquisitionBoard> acquisitionBoard;

    /** True if device is available*/
    bool deviceFound = true;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (DeviceThread);
};

#endif // __DEVICETHREAD_H_2C4CBD67__
