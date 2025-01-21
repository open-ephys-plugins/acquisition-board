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

#include "AcqBoardOpalKelly.h"

#if defined(_WIN32)
#define okLIB_NAME "okFrontPanel.dll"
#define okLIB_EXTENSION "*.dll"
#elif defined(__APPLE__)
#define okLIB_NAME "libokFrontPanel.dylib"
#define okLIB_EXTENSION "*.dylib"
#elif defined(__linux__)
#define okLIB_NAME "./libokFrontPanel.so"
#define okLIB_EXTENSION "*.so"
#endif

#define INIT_STEP (evalBoard->isUSB3() ? 256 : 60)

AcqBoardOpalKelly::AcqBoardOpalKelly (DataBuffer* buffer_) : AcquisitionBoard (buffer_),
                                                             chipRegisters (30000.0f)
{
    impedanceMeter = std::make_unique<ImpedanceMeterOpalKelly> (this);
    
    evalBoard = std::make_unique<Rhd2000EvalBoard>();

    memset (auxBuffer, 0, sizeof (auxBuffer));
    memset (auxSamples, 0, sizeof (auxSamples));

    MAX_NUM_DATA_STREAMS = evalBoard->MAX_NUM_DATA_STREAMS;
    MAX_NUM_HEADSTAGES = MAX_NUM_DATA_STREAMS / 2;

    int maxNumHeadstages = 8;

    for (int i = 0; i < maxNumHeadstages; i++)
        headstages.add (new HeadstageOpalKelly (static_cast<Rhd2000EvalBoard::BoardDataSource> (i), maxNumHeadstages));

    for (int k = 0; k < 8; k++)
    {
        dacChannelsToUpdate.add (true);
        dacStream.add (0);
        setDACTriggerThreshold (k, 65534);
        dacChannels.add (0);
        dacThresholds.set (k, 0);
    }
}

AcqBoardOpalKelly::~AcqBoardOpalKelly()
{
    LOGD ("RHD2000 interface destroyed.");

    if (deviceFound)
    {
        int ledArray[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
        evalBoard->setLedDisplay (ledArray);
        evalBoard->resetFpga();
    }
}

bool AcqBoardOpalKelly::detectBoard()
{
    LOGC ("Searching for Opal Kelly Acquisition Board...");

#if defined(__APPLE__)
    File appBundle = File::getSpecialLocation (File::currentApplicationFile);
    const String executableDirectory = appBundle.getChildFile ("Contents/Resources").getFullPathName();
#else
    File executable = File::getSpecialLocation (File::currentExecutableFile);
    const String executableDirectory = executable.getParentDirectory().getFullPathName();
#endif

    String libraryFilePath = executableDirectory;
    libraryFilePath += File::getSeparatorString();
    libraryFilePath += okLIB_NAME;

    int return_code = evalBoard->open (libraryFilePath.getCharPointer());

    if (return_code == 1) // device found
    {
        LOGC ("Board opened successfully.");

            // Get some general information about the XEM.
        LOGC("FPGA system clock: ", evalBoard->getSystemClockFreq(), " MHz"); // Should indicate 100 MHz
        LOGC ("Opal Kelly device firmware version: ", evalBoard->dev->GetDeviceMajorVersion(), ".", evalBoard->dev->GetDeviceMinorVersion());
        LOGC ("Opal Kelly device serial number: ", evalBoard->dev->GetSerialNumber().c_str());
        LOGC ("Opal Kelly device ID std::string: ", evalBoard->dev->GetDeviceID().c_str());

        deviceFound = true;
        return true;
    }
    else if (return_code == -1) // Opal Kelly library not found
    {
        LOGC ("No Opal Kelly DLL found.");
        return false;
    }
    else if (return_code == -2)
    {
        LOGC ("No Opal Kelly Acquisition Board found.");
        return false;
    }
}

bool AcqBoardOpalKelly::initializeBoard()
{
    LOGC ("Initializing Opal Kelly Acquisition Board...");

    dataBlock = std::make_unique<Rhd2000DataBlock> (1, evalBoard->isUSB3());

    String bitfilename;

#if defined(__APPLE__)
    File appBundle = File::getSpecialLocation (File::currentApplicationFile);
    const String executableDirectory = appBundle.getChildFile ("Contents/Resources").getFullPathName();
#else
    File executable = File::getSpecialLocation (File::currentExecutableFile);
    const String executableDirectory = executable.getParentDirectory().getFullPathName();
#endif

    bitfilename = executableDirectory;
    bitfilename += File::getSeparatorString();
    bitfilename += "shared";
    bitfilename += File::getSeparatorString();
    bitfilename += evalBoard->isUSB3() ? "rhd2000_usb3.bit" : "rhd2000.bit";

    if (! evalBoard->uploadFpgaBitfile (bitfilename.toStdString()))
    {
        LOGC ("Could not upload FPGA bitfile from ", bitfilename);
        deviceFound = false;
        return false;
    }

    LOGC ("Successfully uploaded bitfile, initializing board...");

    evalBoard->initialize();
    // This applies the following settings:
    //  - sample rate to 30 kHz
    //  - aux command banks to zero
    //  - aux command lengths to zero
    //  - continuous run mode to 'true'
    //  - maxTimeStep to 2^32 - 1
    //  - all cable lengths to 3 feet
    //  - dspSettle to 'false'
    //  - data source mapping as 0->PortA1, 1->PortB1, 2->PortC1, 3->PortD1, etc.
    //  - enables all data streams
    //  - clears the ttlOut
    //  - disables all DACs and sets gain to 0

    setSampleRate (30000);

    evalBoard->setCableLengthMeters (Rhd2000EvalBoard::PortA, settings.cableLength.portA);
    evalBoard->setCableLengthMeters (Rhd2000EvalBoard::PortB, settings.cableLength.portB);
    evalBoard->setCableLengthMeters (Rhd2000EvalBoard::PortC, settings.cableLength.portC);
    evalBoard->setCableLengthMeters (Rhd2000EvalBoard::PortD, settings.cableLength.portD);

    // Select RAM Bank 0 for AuxCmd3 initially, so the ADC is calibrated.
    evalBoard->selectAuxCommandBank (Rhd2000EvalBoard::PortA, Rhd2000EvalBoard::AuxCmd3, 0);
    evalBoard->selectAuxCommandBank (Rhd2000EvalBoard::PortB, Rhd2000EvalBoard::AuxCmd3, 0);
    evalBoard->selectAuxCommandBank (Rhd2000EvalBoard::PortC, Rhd2000EvalBoard::AuxCmd3, 0);
    evalBoard->selectAuxCommandBank (Rhd2000EvalBoard::PortD, Rhd2000EvalBoard::AuxCmd3, 0);

    // Since our longest command sequence is 60 commands, run the SPI interface for
    // 60 samples (64 for usb3 power-of two needs)
    evalBoard->setMaxTimeStep (INIT_STEP);
    evalBoard->setContinuousRunMode (false);

    // Start SPI interface
    evalBoard->run();

    // Wait for the 60-sample run to complete
    while (evalBoard->isRunning())
    {
        ;
    }

    // Read the resulting single data block from the USB interface. We don't
    // need to do anything with this, since it was only used for ADC calibration
    std::unique_ptr<Rhd2000DataBlock> dataBlock = std::make_unique<Rhd2000DataBlock> (evalBoard->getNumEnabledDataStreams(), evalBoard->isUSB3());

    evalBoard->readDataBlock (dataBlock.get(), INIT_STEP);
    // Now that ADC calibration has been performed, we switch to the command sequence
    // that does not execute ADC calibration.
    evalBoard->selectAuxCommandBank (Rhd2000EvalBoard::PortA, Rhd2000EvalBoard::AuxCmd3, settings.fastSettleEnabled ? 2 : 1);
    evalBoard->selectAuxCommandBank (Rhd2000EvalBoard::PortB, Rhd2000EvalBoard::AuxCmd3, settings.fastSettleEnabled ? 2 : 1);
    evalBoard->selectAuxCommandBank (Rhd2000EvalBoard::PortC, Rhd2000EvalBoard::AuxCmd3, settings.fastSettleEnabled ? 2 : 1);
    evalBoard->selectAuxCommandBank (Rhd2000EvalBoard::PortD, Rhd2000EvalBoard::AuxCmd3, settings.fastSettleEnabled ? 2 : 1);

    if (evalBoard->isUSB3())
        LOGD ("USB3 board mode enabled");

    // Turn one LED on to indicate that the board is now connected
    int ledArray[8] = { 1, 0, 0, 0, 0, 0, 0, 0 };
    evalBoard->setLedDisplay (ledArray);

    return true;
}

bool AcqBoardOpalKelly::foundInputSource() const
{
    return deviceFound;
}

Array<const Headstage*> AcqBoardOpalKelly::getHeadstages()
{
    Array<const Headstage*> connectedHeadstages;

    for(auto headstage : headstages)
    {
        if (headstage->isConnected())
            connectedHeadstages.add (headstage);
    }

    return connectedHeadstages;
}

Array<int> AcqBoardOpalKelly::getAvailableSampleRates()
{
    Array<int> sampleRates;

    sampleRates.add (1000);
    sampleRates.add (1250);
    sampleRates.add (1500);
    sampleRates.add (2000);
    sampleRates.add (2500);
    sampleRates.add (3000);
    sampleRates.add (3333);
    sampleRates.add (4000);
    sampleRates.add (5000);
    sampleRates.add (6250);
    sampleRates.add (8000);
    sampleRates.add (10000);
    sampleRates.add (12500);
    sampleRates.add (15000);
    sampleRates.add (20000);
    sampleRates.add (25000);
    sampleRates.add (30000);

    return sampleRates;
}

void AcqBoardOpalKelly::setSampleRate (int desiredSampleRate)
{

    Rhd2000EvalBoard::AmplifierSampleRate sampleRate;

    switch (desiredSampleRate)
    {
        case 1000:
            sampleRate = Rhd2000EvalBoard::SampleRate1000Hz;
            numUsbBlocksToRead = 1;
            settings.boardSampleRate = 1000.0f;
            break;
        case 1250:
            sampleRate = Rhd2000EvalBoard::SampleRate1250Hz;
            numUsbBlocksToRead = 1;
            settings.boardSampleRate = 1250.0f;
            break;
        case 1500:
            sampleRate = Rhd2000EvalBoard::SampleRate1500Hz;
            numUsbBlocksToRead = 1;
            settings.boardSampleRate = 1500.0f;
            break;
        case 2000:
            sampleRate = Rhd2000EvalBoard::SampleRate2000Hz;
            numUsbBlocksToRead = 1;
            settings.boardSampleRate = 2000.0f;
            break;
        case 2500:
            sampleRate = Rhd2000EvalBoard::SampleRate2500Hz;
            numUsbBlocksToRead = 1;
            settings.boardSampleRate = 2500.0f;
            break;
        case 3000:
            sampleRate = Rhd2000EvalBoard::SampleRate3000Hz;
            numUsbBlocksToRead = 2;
            settings.boardSampleRate = 3000.0f;
            break;
        case 3333:
            sampleRate = Rhd2000EvalBoard::SampleRate3333Hz;
            numUsbBlocksToRead = 2;
            settings.boardSampleRate = 3333.0f;
            break;
        case 4000:
            sampleRate = Rhd2000EvalBoard::SampleRate4000Hz;
            numUsbBlocksToRead = 2;
            settings.boardSampleRate = 4000.0f;
            break;
        case 5000:
            sampleRate = Rhd2000EvalBoard::SampleRate5000Hz;
            numUsbBlocksToRead = 3;
            settings.boardSampleRate = 5000.0f;
            break;
        case 6250:
            sampleRate = Rhd2000EvalBoard::SampleRate6250Hz;
            numUsbBlocksToRead = 3;
            settings.boardSampleRate = 6250.0f;
            break;
        case 8000:
            sampleRate = Rhd2000EvalBoard::SampleRate8000Hz;
            numUsbBlocksToRead = 4;
            settings.boardSampleRate = 8000.0f;
            break;
        case 10000:
            sampleRate = Rhd2000EvalBoard::SampleRate10000Hz;
            numUsbBlocksToRead = 6;
            settings.boardSampleRate = 10000.0f;
            break;
        case 12500:
            sampleRate = Rhd2000EvalBoard::SampleRate12500Hz;
            numUsbBlocksToRead = 7;
            settings.boardSampleRate = 12500.0f;
            break;
        case 15000:
            sampleRate = Rhd2000EvalBoard::SampleRate15000Hz;
            numUsbBlocksToRead = 8;
            settings.boardSampleRate = 15000.0f;
            break;
        case 20000:
            sampleRate = Rhd2000EvalBoard::SampleRate20000Hz;
            numUsbBlocksToRead = 12;
            settings.boardSampleRate = 20000.0f;
            break;
        case 25000:
            sampleRate = Rhd2000EvalBoard::SampleRate25000Hz;
            numUsbBlocksToRead = 14;
            settings.boardSampleRate = 25000.0f;
            break;
        case 30000:
            sampleRate = Rhd2000EvalBoard::SampleRate30000Hz;
            numUsbBlocksToRead = 16;
            settings.boardSampleRate = 30000.0f;
            break;
        default:
            LOGC ("Invalid sample rate.");
            return;
    }

    // Select per-channel amplifier sampling rate.
    evalBoard->setSampleRate (sampleRate);

    LOGC ("Sample rate set to ", evalBoard->getSampleRate());

    // Now that we have set our sampling rate, we can set the MISO sampling delay
    // which is dependent on the sample rate.
    evalBoard->setCableLengthMeters (Rhd2000EvalBoard::PortA, settings.cableLength.portA);
    evalBoard->setCableLengthMeters (Rhd2000EvalBoard::PortB, settings.cableLength.portB);
    evalBoard->setCableLengthMeters (Rhd2000EvalBoard::PortC, settings.cableLength.portC);
    evalBoard->setCableLengthMeters (Rhd2000EvalBoard::PortD, settings.cableLength.portD);

    updateRegisters();
}

float AcqBoardOpalKelly::getSampleRate() const
{
    return settings.boardSampleRate;
}

void AcqBoardOpalKelly::updateRegisters()
{
    // Set up an RHD2000 register object using this sample rate to
    // optimize MUX-related register settings.
    chipRegisters.defineSampleRate (settings.boardSampleRate);

    int commandSequenceLength;
    std::vector<int> commandList;

    // Create a command list for the AuxCmd1 slot.  This command sequence will continuously
    // update Register 3, which controls the auxiliary digital output pin on each RHD2000 chip.
    // In concert with the v1.4 Rhythm FPGA code, this permits real-time control of the digital
    // output pin on chips on each SPI port.
    chipRegisters.setDigOutLow(); // Take auxiliary output out of HiZ mode.
    commandSequenceLength = chipRegisters.createCommandListUpdateDigOut (commandList);
    evalBoard->uploadCommandList (commandList, Rhd2000EvalBoard::AuxCmd1, 0);
    evalBoard->selectAuxCommandLength (Rhd2000EvalBoard::AuxCmd1, 0, commandSequenceLength - 1);
    evalBoard->selectAuxCommandBank (Rhd2000EvalBoard::PortA, Rhd2000EvalBoard::AuxCmd1, 0);
    evalBoard->selectAuxCommandBank (Rhd2000EvalBoard::PortB, Rhd2000EvalBoard::AuxCmd1, 0);
    evalBoard->selectAuxCommandBank (Rhd2000EvalBoard::PortC, Rhd2000EvalBoard::AuxCmd1, 0);
    evalBoard->selectAuxCommandBank (Rhd2000EvalBoard::PortD, Rhd2000EvalBoard::AuxCmd1, 0);

    // Next, we'll create a command list for the AuxCmd2 slot.  This command sequence
    // will sample the temperature sensor and other auxiliary ADC inputs.
    commandSequenceLength = chipRegisters.createCommandListTempSensor (commandList);
    evalBoard->uploadCommandList (commandList, Rhd2000EvalBoard::AuxCmd2, 0);
    evalBoard->selectAuxCommandLength (Rhd2000EvalBoard::AuxCmd2, 0, commandSequenceLength - 1);
    evalBoard->selectAuxCommandBank (Rhd2000EvalBoard::PortA, Rhd2000EvalBoard::AuxCmd2, 0);
    evalBoard->selectAuxCommandBank (Rhd2000EvalBoard::PortB, Rhd2000EvalBoard::AuxCmd2, 0);
    evalBoard->selectAuxCommandBank (Rhd2000EvalBoard::PortC, Rhd2000EvalBoard::AuxCmd2, 0);
    evalBoard->selectAuxCommandBank (Rhd2000EvalBoard::PortD, Rhd2000EvalBoard::AuxCmd2, 0);

    // Before generating register configuration command sequences, set amplifier
    // bandwidth paramters.
    settings.dsp.cutoffFreq = chipRegisters.setDspCutoffFreq (settings.dsp.cutoffFreq);
    settings.analogFilter.lowerBandwidth = chipRegisters.setLowerBandwidth (settings.analogFilter.lowerBandwidth);
    settings.analogFilter.upperBandwidth = chipRegisters.setUpperBandwidth (settings.analogFilter.upperBandwidth);
    chipRegisters.enableDsp (settings.dsp.enabled);

    // enable/disable aux inputs:
    chipRegisters.enableAux1 (settings.acquireAux);
    chipRegisters.enableAux2 (settings.acquireAux);
    chipRegisters.enableAux3 (settings.acquireAux);

    chipRegisters.createCommandListRegisterConfig (commandList, true);
    // Upload version with ADC calibration to AuxCmd3 RAM Bank 0.
    evalBoard->uploadCommandList (commandList, Rhd2000EvalBoard::AuxCmd3, 0);
    evalBoard->selectAuxCommandLength (Rhd2000EvalBoard::AuxCmd3, 0, commandSequenceLength - 1);

    commandSequenceLength = chipRegisters.createCommandListRegisterConfig (commandList, false);
    // Upload version with no ADC calibration to AuxCmd3 RAM Bank 1.
    evalBoard->uploadCommandList (commandList, Rhd2000EvalBoard::AuxCmd3, 1);
    evalBoard->selectAuxCommandLength (Rhd2000EvalBoard::AuxCmd3, 0, commandSequenceLength - 1);

    chipRegisters.setFastSettle (true);

    commandSequenceLength = chipRegisters.createCommandListRegisterConfig (commandList, false);
    // Upload version with fast settle enabled to AuxCmd3 RAM Bank 2.
    evalBoard->uploadCommandList (commandList, Rhd2000EvalBoard::AuxCmd3, 2);
    evalBoard->selectAuxCommandLength (Rhd2000EvalBoard::AuxCmd3, 0, commandSequenceLength - 1);

    chipRegisters.setFastSettle (false);
    evalBoard->selectAuxCommandBank (Rhd2000EvalBoard::PortA, Rhd2000EvalBoard::AuxCmd3, settings.fastSettleEnabled ? 2 : 1);
    evalBoard->selectAuxCommandBank (Rhd2000EvalBoard::PortB, Rhd2000EvalBoard::AuxCmd3, settings.fastSettleEnabled ? 2 : 1);
    evalBoard->selectAuxCommandBank (Rhd2000EvalBoard::PortC, Rhd2000EvalBoard::AuxCmd3, settings.fastSettleEnabled ? 2 : 1);
    evalBoard->selectAuxCommandBank (Rhd2000EvalBoard::PortD, Rhd2000EvalBoard::AuxCmd3, settings.fastSettleEnabled ? 2 : 1);
}

int AcqBoardOpalKelly::getIntanChipId (Rhd2000DataBlock* dataBlock, int stream, int& register59Value)
{
    bool intanChipPresent;

    // First, check ROM registers 32-36 to verify that they hold 'INTAN', and
    // the initial chip name ROM registers 24-26 that hold 'RHD'.
    // This is just used to verify that we are getting good data over the SPI
    // communication channel.
    intanChipPresent = ((char) dataBlock->auxiliaryData[stream][2][32] == 'I' && (char) dataBlock->auxiliaryData[stream][2][33] == 'N' && (char) dataBlock->auxiliaryData[stream][2][34] == 'T' && (char) dataBlock->auxiliaryData[stream][2][35] == 'A' && (char) dataBlock->auxiliaryData[stream][2][36] == 'N' && (char) dataBlock->auxiliaryData[stream][2][24] == 'R' && (char) dataBlock->auxiliaryData[stream][2][25] == 'H' && (char) dataBlock->auxiliaryData[stream][2][26] == 'D');

    // If the SPI communication is bad, return -1.  Otherwise, return the Intan
    // chip ID number stored in ROM regstier 63.
    if (! intanChipPresent)
    {
        register59Value = -1;
        return -1;
    }
    else
    {
        register59Value = dataBlock->auxiliaryData[stream][2][23]; // Register 59
        return dataBlock->auxiliaryData[stream][2][19]; // chip ID (Register 63)
    }
}

void AcqBoardOpalKelly::scanPorts()
{

    //Clear previous known streams
    enabledStreams.clear();

    // Scan SPI ports
    int delay, hs, id;
    int register59Value;

    for (auto headstage : headstages)
    {
        headstage->setNumStreams (0); // reset stream count
    }

    Rhd2000EvalBoard::BoardDataSource initStreamPorts[8] = {
        Rhd2000EvalBoard::PortA1,
        Rhd2000EvalBoard::PortA2,
        Rhd2000EvalBoard::PortB1,
        Rhd2000EvalBoard::PortB2,
        Rhd2000EvalBoard::PortC1,
        Rhd2000EvalBoard::PortC2,
        Rhd2000EvalBoard::PortD1,
        Rhd2000EvalBoard::PortD2
    };

    chipId.insertMultiple (0, -1, 8);
    Array<int> tmpChipId (chipId);

    float previousSampleRate = settings.boardSampleRate;

    setSampleRate (30000); // set to 30 kHz temporarily

    // Enable all data streams, and set sources to cover one or two chips
    // on Ports A-D.

    // THIS IS DIFFERENT FOR RECORDING CONTROLLER:
    for (int i = 0; i < 8; i++)
        evalBoard->setDataSource (i, initStreamPorts[i]);

    for (int i = 0; i < 8; i++)
        evalBoard->enableDataStream (i, true);

    LOGD ("Number of enabled data streams: ", evalBoard->getNumEnabledDataStreams());

    evalBoard->selectAuxCommandBank (Rhd2000EvalBoard::PortA,
                                     Rhd2000EvalBoard::AuxCmd3,
                                     0);
    evalBoard->selectAuxCommandBank (Rhd2000EvalBoard::PortB,
                                     Rhd2000EvalBoard::AuxCmd3,
                                     0);
    evalBoard->selectAuxCommandBank (Rhd2000EvalBoard::PortC,
                                     Rhd2000EvalBoard::AuxCmd3,
                                     0);
    evalBoard->selectAuxCommandBank (Rhd2000EvalBoard::PortD,
                                     Rhd2000EvalBoard::AuxCmd3,
                                     0);

    // Since our longest command sequence is 60 commands, we run the SPI
    // interface for 60 samples. (64 for usb3 power-of two needs)
    evalBoard->setMaxTimeStep (INIT_STEP);
    evalBoard->setContinuousRunMode (false);

    std::unique_ptr<Rhd2000DataBlock> dataBlock =
        std::make_unique<Rhd2000DataBlock> (evalBoard->getNumEnabledDataStreams(), evalBoard->isUSB3());

    Array<int> sumGoodDelays;
    sumGoodDelays.insertMultiple (0, 0, 8);

    Array<int> indexFirstGoodDelay;
    indexFirstGoodDelay.insertMultiple (0, -1, 8);

    Array<int> indexSecondGoodDelay;
    indexSecondGoodDelay.insertMultiple (0, -1, 8);

    // Run SPI command sequence at all 16 possible FPGA MISO delay settings
    // to find optimum delay for each SPI interface cable.

    LOGD ("Checking for connected amplifier chips...");

    for (delay = 0; delay < 16; delay++)
    {
        evalBoard->setCableDelay (Rhd2000EvalBoard::PortA, delay);
        evalBoard->setCableDelay (Rhd2000EvalBoard::PortB, delay);
        evalBoard->setCableDelay (Rhd2000EvalBoard::PortC, delay);
        evalBoard->setCableDelay (Rhd2000EvalBoard::PortD, delay);

        // Start SPI interface.
        evalBoard->run();

        // Wait for the 60-sample run to complete.
        while (evalBoard->isRunning())
        {
            ;
        }
        // Read the resulting single data block from the USB interface.
        evalBoard->readDataBlock (dataBlock.get(), INIT_STEP);

        // Read the Intan chip ID number from each RHD2000 chip found.
        // Record delay settings that yield good communication with the chip.
        for (hs = 0; hs < headstages.size(); ++hs)
        {
            id = getIntanChipId (dataBlock.get(), hs, register59Value);

            if (id == CHIP_ID_RHD2132 || id == CHIP_ID_RHD2216 || (id == CHIP_ID_RHD2164 && register59Value == REGISTER_59_MISO_A))
            {
                LOGD ("Device ID found: ", id);

                sumGoodDelays.set (hs, sumGoodDelays[hs] + 1);

                if (indexFirstGoodDelay[hs] == -1)
                {
                    indexFirstGoodDelay.set (hs, delay);
                    tmpChipId.set (hs, id);
                }
                else if (indexSecondGoodDelay[hs] == -1)
                {
                    indexSecondGoodDelay.set (hs, delay);
                    tmpChipId.set (hs, id);
                }
            }
        }
    }

#if DEBUG_EMULATE_HEADSTAGES > 0
    if (tmpChipId[0] > 0)
    {
        int chipIdx = 0;
        for (int hs = 0; hs < DEBUG_EMULATE_HEADSTAGES && hs < headstages.size(); ++hs)
        {
            if (enabledStreams.size() < MAX_NUM_DATA_STREAMS (evalBoard->isUSB3()))
            {
#ifdef DEBUG_EMULATE_64CH
                chipId.set (chipIdx++, CHIP_ID_RHD2164);
                chipId.set (chipIdx++, CHIP_ID_RHD2164_B);
                enableHeadstage (hs, true, 2, 32);
#else
                chipId.set (chipIdx++, CHIP_ID_RHD2132);
                enableHeadstage (hs, true, 1, 32);
#endif
            }
        }
        for (int i = 0; i < enabledStreams.size(); i++)
        {
            enabledStreams.set (i, Rhd2000EvalBoard::PortA1);
        }
    }

#else
    // Now, disable data streams where we did not find chips present.
    int chipIdx = 0;

    for (int hs = 0; hs < headstages.size(); ++hs)
    {
        if ((tmpChipId[hs] > 0) && (enabledStreams.size() < MAX_NUM_DATA_STREAMS))
        {
            chipId.set (chipIdx++, tmpChipId[hs]);

            LOGD ("Enabling headstage ", hs);

            if (tmpChipId[hs] == CHIP_ID_RHD2164) //RHD2164
            {
                if (enabledStreams.size() < MAX_NUM_DATA_STREAMS - 1)
                {
                    enableHeadstage (hs, true, 2, 32);
                    chipId.set (chipIdx++, CHIP_ID_RHD2164_B);
                }
                else //just one stream left
                {
                    enableHeadstage (hs, true, 1, 32);
                }
            }
            else
            {
                enableHeadstage (hs, true, 1, tmpChipId[hs] == 1 ? 32 : 16);
            }
        }
        else
        {
            enableHeadstage (hs, false);
        }
    }
#endif
    updateBoardStreams();

    LOGD ("Number of enabled data streams: ", evalBoard->getNumEnabledDataStreams());

    // Set cable delay settings that yield good communication with each
    // RHD2000 chip.
    Array<int> optimumDelay;

    optimumDelay.insertMultiple (0, 0, headstages.size());

    for (hs = 0; hs < headstages.size(); ++hs)
    {
        if (sumGoodDelays[hs] == 1 || sumGoodDelays[hs] == 2)
        {
            optimumDelay.set (hs, indexFirstGoodDelay[hs]);
        }
        else if (sumGoodDelays[hs] > 2)
        {
            optimumDelay.set (hs, indexSecondGoodDelay[hs]);
        }
    }

    evalBoard->setCableDelay (Rhd2000EvalBoard::PortA,
                              jmax (optimumDelay[0], optimumDelay[1]));
    evalBoard->setCableDelay (Rhd2000EvalBoard::PortB,
                              jmax (optimumDelay[2], optimumDelay[3]));
    evalBoard->setCableDelay (Rhd2000EvalBoard::PortC,
                              jmax (optimumDelay[4], optimumDelay[5]));
    evalBoard->setCableDelay (Rhd2000EvalBoard::PortD,
                              jmax (optimumDelay[6], optimumDelay[7]));

    settings.cableLength.portA =
        evalBoard->estimateCableLengthMeters (jmax (optimumDelay[0], optimumDelay[1]));
    settings.cableLength.portB =
        evalBoard->estimateCableLengthMeters (jmax (optimumDelay[2], optimumDelay[3]));
    settings.cableLength.portC =
        evalBoard->estimateCableLengthMeters (jmax (optimumDelay[4], optimumDelay[5]));
    settings.cableLength.portD =
        evalBoard->estimateCableLengthMeters (jmax (optimumDelay[6], optimumDelay[7]));

    setSampleRate (previousSampleRate); // restore saved sample rate
}

void AcqBoardOpalKelly::setCableLength (int hsNum, float length)
{
    // Set the MISO sampling delay, which is dependent on the sample rate.

    switch (hsNum)
    {
        case 0:
            evalBoard->setCableLengthFeet (Rhd2000EvalBoard::PortA, length);
            break;
        case 1:
            evalBoard->setCableLengthFeet (Rhd2000EvalBoard::PortB, length);
            break;
        case 2:
            evalBoard->setCableLengthFeet (Rhd2000EvalBoard::PortC, length);
            break;
        case 3:
            evalBoard->setCableLengthFeet (Rhd2000EvalBoard::PortD, length);
            break;
        case 4:
            evalBoard->setCableLengthFeet (Rhd2000EvalBoard::PortE, length);
            break;
        case 5:
            evalBoard->setCableLengthFeet (Rhd2000EvalBoard::PortF, length);
            break;
        case 6:
            evalBoard->setCableLengthFeet (Rhd2000EvalBoard::PortG, length);
            break;
        case 7:
            evalBoard->setCableLengthFeet (Rhd2000EvalBoard::PortH, length);
            break;
        default:
            break;
    }
}


bool AcqBoardOpalKelly::enableHeadstage (int hsNum, bool enabled, int nStr, int strChans)
{
    LOGD ("Headstage ", hsNum, ", enabled: ", enabled, ", num streams: ", nStr, ", stream channels: ", strChans);
    LOGD ("Max num headstages: ", MAX_NUM_HEADSTAGES);

    if (enabled)
    {
        headstages[hsNum]->setFirstChannel (getNumDataOutputs (ContinuousChannel::ELECTRODE));
        headstages[hsNum]->setNumStreams (nStr);
        headstages[hsNum]->setChannelsPerStream (strChans);
        headstages[hsNum]->setFirstStreamIndex (enabledStreams.size());
        enabledStreams.add (headstages[hsNum]->getDataStream (0));
        numChannelsPerDataStream.add (strChans);

        if (nStr > 1)
        {
            enabledStreams.add (headstages[hsNum]->getDataStream (1));
            numChannelsPerDataStream.add (strChans);
        }
    }
    else
    {
        int idx = enabledStreams.indexOf (headstages[hsNum]->getDataStream (0));

        if (idx >= 0)
        {
            enabledStreams.remove (idx);
            numChannelsPerDataStream.remove (idx);
        }

        if (headstages[hsNum]->getNumStreams() > 1)
        {
            idx = enabledStreams.indexOf (headstages[hsNum]->getDataStream (1));
            if (idx >= 0)
            {
                enabledStreams.remove (idx);
                numChannelsPerDataStream.remove (idx);
            }
        }

        headstages[hsNum]->setNumStreams (0);
    }

    buffer->resize (getNumChannels(), 10000);

    return true;
}

void AcqBoardOpalKelly::updateBoardStreams()
{
    for (int i = 0; i < MAX_NUM_DATA_STREAMS; i++)
    {
        if (i < enabledStreams.size())
        {
            //std::cout << "Enabling stream " << i << " with source " << enabledStreams[i] << std::endl;
            evalBoard->enableDataStream (i, true);
            evalBoard->setDataSource (i, enabledStreams[i]);
        }
        else
        {
            evalBoard->enableDataStream (i, false);
        }
    }
}

bool AcqBoardOpalKelly::isHeadstageEnabled (int hsNum) const
{
    return headstages[hsNum]->isConnected();
}

int AcqBoardOpalKelly::getActiveChannelsInHeadstage (int hsNum) const
{
    return headstages[hsNum]->getNumActiveChannels();
}

int AcqBoardOpalKelly::getChannelsInHeadstage (int hsNum) const
{
    return headstages[hsNum]->getNumChannels();
}

float AcqBoardOpalKelly::getBitVolts (ContinuousChannel::Type channelType) const
{
    switch (channelType)
    {
        case ContinuousChannel::ELECTRODE:
            return 0.195f;
        case ContinuousChannel::AUX:
            return 0.0000374;
        case ContinuousChannel::ADC:
            return 0.00015258789;
    }
}

void AcqBoardOpalKelly::measureImpedances()
{
    impedanceMeter->stopThreadSafely();

    impedanceMeter->runThread();
}

void AcqBoardOpalKelly::impedanceMeasurementFinished()
{
    
    if (impedances.valid)
    {
        LOGD ("Updating headstage impedance values");

        for (auto hs : headstages)
        {
            if (hs->isConnected())
            {
                hs->setImpedances (impedances);
            }
        }

        editor->impedanceMeasurementFinished();
    }
 }

void AcqBoardOpalKelly::saveImpedances (File& file)
{
    if (impedances.valid)
    {
        std::unique_ptr<XmlElement> xml = std::unique_ptr<XmlElement> (new XmlElement ("IMPEDANCES"));

        int globalChannelNumber = -1;

        for (auto hs : headstages)
        {
            XmlElement* headstageXml = new XmlElement ("HEADSTAGE");
            headstageXml->setAttribute ("name", hs->getStreamPrefix());

            for (int ch = 0; ch < hs->getNumActiveChannels(); ch++)
            {
                globalChannelNumber++;

                XmlElement* channelXml = new XmlElement ("CHANNEL");
                channelXml->setAttribute ("name", hs->getChannelName (ch));
                channelXml->setAttribute ("number", globalChannelNumber);
                channelXml->setAttribute ("magnitude", hs->getImpedanceMagnitude (ch));
                channelXml->setAttribute ("phase", hs->getImpedancePhase (ch));
                headstageXml->addChildElement (channelXml);
            }

            xml->addChildElement (headstageXml);
        }

       xml->writeTo(file, XmlElement::TextFormat());
    }
}

void AcqBoardOpalKelly::setNamingScheme (ChannelNamingScheme scheme)
{
    channelNamingScheme = scheme;

    for (auto hs : headstages)
    {
        hs->setNamingScheme (scheme);
    }
}

ChannelNamingScheme AcqBoardOpalKelly::getNamingScheme()
{
    return channelNamingScheme;
}

void AcqBoardOpalKelly::enableAuxChannels (bool enabled)
{
    settings.acquireAux = enabled;
    buffer->resize (getNumChannels(), 10000);
    updateRegisters();
}

bool AcqBoardOpalKelly::areAuxChannelsEnabled() const
{
    return settings.acquireAux;
}

void AcqBoardOpalKelly::enableAdcChannels (bool enabled)
{
    settings.acquireAdc = enabled;
    buffer->resize (getNumChannels(), 10000);
}

bool AcqBoardOpalKelly::areAdcChannelsEnabled() const
{
    return settings.acquireAdc;
}

double AcqBoardOpalKelly::setUpperBandwidth (double upper)
{

    settings.analogFilter.upperBandwidth = upper;

    updateRegisters();

    return settings.analogFilter.upperBandwidth;
}

double AcqBoardOpalKelly::setLowerBandwidth (double lower)
{

    settings.analogFilter.lowerBandwidth = lower;

    updateRegisters();

    return settings.analogFilter.lowerBandwidth;
}

double AcqBoardOpalKelly::setDspCutoffFreq (double freq)
{

    settings.dsp.cutoffFreq = freq;

    updateRegisters();

    return settings.dsp.cutoffFreq;
}

double AcqBoardOpalKelly::getDspCutoffFreq() const
{
    return settings.dsp.cutoffFreq;
}

void AcqBoardOpalKelly::setDspOffset (bool state)
{

    settings.dsp.enabled = state;

    updateRegisters();
}

void AcqBoardOpalKelly::setTTLOutputMode (bool state)
{
    settings.ttlOutputMode = state;

    updateSettingsDuringAcquisition = true;
}

void AcqBoardOpalKelly::setDAChpf (float cutoff, bool enabled)
{
    settings.desiredDAChpf = cutoff;

    settings.desiredDAChpfState = enabled;

    updateSettingsDuringAcquisition = true;
}

void AcqBoardOpalKelly::setFastTTLSettle (bool state, int channel)
{
    settings.fastTTLSettleEnabled = state;

    settings.fastSettleTTLChannel = channel;

    updateSettingsDuringAcquisition = true;
}

int AcqBoardOpalKelly::setNoiseSlicerLevel (int level)
{
    settings.noiseSlicerLevel = level;

    if (deviceFound)
        evalBoard->setAudioNoiseSuppress (settings.noiseSlicerLevel);

    // Level has been checked once before this and then is checked again in setAudioNoiseSuppress.
    // This may be overkill - maybe API should change so that the final function returns the value?

    return settings.noiseSlicerLevel;
}

void AcqBoardOpalKelly::enableBoardLeds (bool enable)
{
    settings.ledsEnabled = enable;

    if (isTransmitting)
        updateSettingsDuringAcquisition = true;
    else
        evalBoard->enableBoardLeds (enable);
}

int AcqBoardOpalKelly::setClockDivider (int divide_ratio)
{
    if (! deviceFound)
        return 1;

    // Divide ratio should be 1 or an even number
    if (divide_ratio != 1 && divide_ratio % 2)
        divide_ratio--;

    // Format the divide ratio from its true value to the
    // format required by the firmware
    // Ratio    N
    // 1        0
    // >=2      Ratio/2
    if (divide_ratio == 1)
        settings.clockDivideFactor = 0;
    else
        settings.clockDivideFactor = static_cast<uint16> (divide_ratio / 2);

    if (isTransmitting)
        updateSettingsDuringAcquisition = true;
    else
        evalBoard->setClockDivider (settings.clockDivideFactor);

    return divide_ratio;
}


void AcqBoardOpalKelly::setDACTriggerThreshold (int dacChannelIndex, float threshold)
{
    dacThresholds.set (dacChannelIndex, threshold);
    dacChannelsToUpdate.set (dacChannelIndex, true);
    updateSettingsDuringAcquisition = true;

    //evalBoard->setDacThresholdVoltage(dacOutput,threshold);
}

void AcqBoardOpalKelly::connectHeadstageChannelToDAC (int headstageChannelIndex, int dacChannelIndex)
{

    if (dacChannelIndex == -1)
        return;

    if (headstageChannelIndex < getNumDataOutputs (ContinuousChannel::ELECTRODE))
    {
        int channelCount = 0;
        for (int i = 0; i < enabledStreams.size(); i++)
        {
            if (headstageChannelIndex < channelCount + numChannelsPerDataStream[i])
            {
                dacChannels.set (dacChannelIndex, headstageChannelIndex - channelCount);
                dacStream.set (dacChannelIndex, i);
                break;
            }
            else
            {
                channelCount += numChannelsPerDataStream[i];
            }
        }
        dacChannelsToUpdate.set (dacChannelIndex, true);
        updateSettingsDuringAcquisition = true;
    }
}

bool AcqBoardOpalKelly::startAcquisition()
{
    impedanceMeter->waitSafely();
    dataBlock.reset( new Rhd2000DataBlock(evalBoard->getNumEnabledDataStreams(), evalBoard->isUSB3()));

    LOGD ("Expecting ", getNumChannels(), " channels.");

    int ledArray[8] = { 1, 1, 0, 0, 0, 0, 0, 0 };
    evalBoard->setLedDisplay (ledArray);

    // reset TTL output state
    for (int k = 0; k < 16; k++)
    {
        TTL_OUTPUT_STATE[k] = 0;
    }

    evalBoard->flush();
    evalBoard->setContinuousRunMode (true);
    evalBoard->run();

    blockSize = dataBlock->calculateDataBlockSizeInWords (evalBoard->getNumEnabledDataStreams(), evalBoard->isUSB3());
    //LOGD("Expecting blocksize of ", blockSize, " for ", evalBoard->getNumEnabledDataStreams(), " streams");

    isTransmitting = true;

    startThread();

    return true;
}

bool AcqBoardOpalKelly::stopAcquisition()
{
    //LOGD("RHD2000 data thread stopping acquisition.");

    if (isThreadRunning())
    {
        signalThreadShouldExit();
    }

    if (waitForThreadToExit (500))
    {
        //LOGD("RHD2000 data thread exited.");
    }
    else
    {
        //LOGD("RHD2000 data thread failed to exit, continuing anyway...");
    }

    if (deviceFound)
    {
        evalBoard->setContinuousRunMode (false);
        evalBoard->setMaxTimeStep (0);
        LOGD ("Flushing FIFO.");
        evalBoard->flush();
    }

    buffer->clear();

    if (deviceFound)
    {
        LOGD ("Number of 16-bit words in FIFO: ", evalBoard->numWordsInFifo());

        int ledArray[8] = { 1, 0, 0, 0, 0, 0, 0, 0 };
        evalBoard->setLedDisplay (ledArray);
    }

    isTransmitting = false;
    updateSettingsDuringAcquisition = false;

    // remove timers
    digitalOutputTimers.clear();

    // remove commands
    while (! digitalOutputCommands.empty())
        digitalOutputCommands.pop();

    return true;
}

void AcqBoardOpalKelly::run()
{
    while (! threadShouldExit())
    {
        unsigned char* bufferPtr;
        double ts;

        if (evalBoard->isUSB3() || evalBoard->numWordsInFifo() >= blockSize)
        {
            bool return_code;

            return_code = evalBoard->readRawDataBlock (&bufferPtr);
            // see Rhd2000DataBlock::fillFromUsbBuffer() for documentation of buffer structure

            int index = 0;
            int auxIndex, chanIndex;
            int numStreams = enabledStreams.size();
            int nSamps = Rhd2000DataBlock::getSamplesPerDataBlock (evalBoard->isUSB3());

            //evalBoard->printFIFOmetrics();
            for (int samp = 0; samp < nSamps; samp++)
            {
                int channel = -1;

                if (! Rhd2000DataBlock::checkUsbHeader (bufferPtr, index))
                {
                    LOGE ("Error in Rhd2000EvalBoard::readDataBlock: Incorrect header.");
                    break;
                }

                index += 8; // magic number header width (bytes)
                int64 timestamp = Rhd2000DataBlock::convertUsbTimeStamp (bufferPtr, index);
                index += 4; // timestamp width
                auxIndex = index; // aux chans start at this offset
                index += 6 * numStreams; // width of the 3 aux chans

                for (int dataStream = 0; dataStream < numStreams; dataStream++)
                {
                    int nChans = numChannelsPerDataStream[dataStream];

                    chanIndex = index + 2 * dataStream;

                    if ((chipId[dataStream] == CHIP_ID_RHD2132) && (nChans == 16)) //RHD2132 16ch. headstage
                    {
                        chanIndex += 2 * RHD2132_16CH_OFFSET * numStreams;
                    }

                    for (int chan = 0; chan < nChans; chan++)
                    {
                        channel++;
                        thisSample[channel] = float (*(uint16*) (bufferPtr + chanIndex) - 32768) * 0.195f;
                        chanIndex += 2 * numStreams; // single chan width (2 bytes)
                    }
                }
                index += 64 * numStreams; // neural data width
                auxIndex += 2 * numStreams; // skip AuxCmd1 slots (see updateRegisters())
                // copy the 3 aux channels
                if (settings.acquireAux)
                {
                    for (int dataStream = 0; dataStream < numStreams; dataStream++)
                    {
                        if (chipId[dataStream] != CHIP_ID_RHD2164_B)
                        {
                            int auxNum = (samp + 3) % 4;
                            if (auxNum < 3)
                            {
                                auxSamples[dataStream][auxNum] = float (*(uint16*) (bufferPtr + auxIndex) - 32768) * 0.0000374;
                            }
                            for (int chan = 0; chan < 3; chan++)
                            {
                                channel++;
                                if (auxNum == 3)
                                {
                                    auxBuffer[channel] = auxSamples[dataStream][chan];
                                }
                                thisSample[channel] = auxBuffer[channel];
                            }
                        }
                        auxIndex += 2; // single chan width (2 bytes)
                    }
                }
                index += 2 * numStreams; // skip over filler word at the end of each data stream
                // copy the 8 ADC channels
                if (settings.acquireAdc)
                {
                    for (int adcChan = 0; adcChan < 8; ++adcChan)
                    {
                        channel++;
                        // ADC waveform units = volts


                        thisSample[channel] = getBitVolts (ContinuousChannel::ADC) * float (*(uint16*) (bufferPtr + index)) - 5 - 0.4096;
                        
                        index += 2; // single chan width (2 bytes)
                    }
                }
                else
                {
                    index += 16; // skip ADC chans (8 * 2 bytes)
                }

                uint64 ttlEventWord = *(uint64*) (bufferPtr + index) & 65535;

                index += 4;

                buffer->addToBuffer (thisSample,
                                               &timestamp,
                                               &ts,
                                               &ttlEventWord,
                                               1);
            }
        }

        if (updateSettingsDuringAcquisition)
        {
            LOGD ("DAC");
            for (int k = 0; k < 8; k++)
            {
                if (dacChannelsToUpdate[k])
                {
                    dacChannelsToUpdate.set(k, false);
                    if (dacChannels[k] >= 0)
                    {
                        evalBoard->enableDac (k, true);
                        evalBoard->selectDacDataStream (k, dacStream[k]);
                        evalBoard->selectDacDataChannel (k, dacChannels[k]);
                        evalBoard->setDacThreshold (k, (int) abs ((dacThresholds[k] / 0.195) + 32768), dacThresholds[k] >= 0);
                        // evalBoard->setDacThresholdVoltage(k, (int) dacThresholds[k]);
                    }
                    else
                    {
                        evalBoard->enableDac (k, false);
                    }
                }
            }

            evalBoard->setTtlMode (settings.ttlOutputMode ? 1 : 0);
            evalBoard->enableExternalFastSettle (settings.fastTTLSettleEnabled);
            evalBoard->setExternalFastSettleChannel (settings.fastSettleTTLChannel);
            evalBoard->setDacHighpassFilter (settings.desiredDAChpf);
            evalBoard->enableDacHighpassFilter (settings.desiredDAChpfState);
            evalBoard->enableBoardLeds (settings.ledsEnabled);
            evalBoard->setClockDivider (settings.clockDivideFactor);

            updateSettingsDuringAcquisition = false;
        }

        if (! digitalOutputCommands.empty())
        {
            while (! digitalOutputCommands.empty())
            {
                DigitalOutputCommand command = digitalOutputCommands.front();
                TTL_OUTPUT_STATE[command.ttlLine] = command.state;
                digitalOutputCommands.pop();
            }

            evalBoard->setTtlOut (TTL_OUTPUT_STATE);

            LOGB ("TTL OUTPUT STATE: ",
                  TTL_OUTPUT_STATE[0],
                  TTL_OUTPUT_STATE[1],
                  TTL_OUTPUT_STATE[2],
                  TTL_OUTPUT_STATE[3],
                  TTL_OUTPUT_STATE[4],
                  TTL_OUTPUT_STATE[5],
                  TTL_OUTPUT_STATE[6],
                  TTL_OUTPUT_STATE[7]);
        }
    }
}



void AcqBoardOpalKelly::setNumHeadstageChannels (int hsNum, int numChannels)
{
    if (headstages[hsNum]->getNumChannels() == 32)
    {
        if (numChannels < headstages[hsNum]->getNumChannels())
            headstages[hsNum]->setHalfChannels (true);
        else
            headstages[hsNum]->setHalfChannels (false);

        numChannelsPerDataStream.set (headstages[hsNum]->getStreamIndex (0), numChannels);
    }

    int channelIndex = 0;

    for (auto hs : headstages)
    {
        if (hs->isConnected())
        {
            hs->setFirstChannel (channelIndex);

            channelIndex += hs->getNumActiveChannels();
        }
    }
}

int AcqBoardOpalKelly::getNumDataOutputs (ContinuousChannel::Type type)
{
    if (type == ContinuousChannel::ELECTRODE)
    {
        int totalChannels = 0;

        for (auto headstage : headstages)
        {
            if (headstage->isConnected())
            {
                totalChannels += headstage->getNumActiveChannels();
            }
        }

        return totalChannels;
    }
    if (type == ContinuousChannel::AUX)
    {
        if (settings.acquireAux)
        {
            int numAuxOutputs = 0;

            for (auto headstage : headstages)
            {
                if (headstage->isConnected())
                {
                    numAuxOutputs += 3;
                }
            }
            return numAuxOutputs;
        }
        else
        {
            return 0;
        }
    }
    if (type == ContinuousChannel::ADC)
    {
        if (settings.acquireAdc)
        {
            return 8;
        }
        else
        {
            return 0;
        }
    }

    return 0;
}


int AcqBoardOpalKelly::getChannelFromHeadstage (int hs, int ch)
{
    int channelCount = 0;
    int hsCount = 0;
    if (hs < 0 || hs >= headstages.size() + 1)
        return -1;
    if (hs == headstages.size()) //let's consider this the ADC channels
    {
        int adcOutputs = getNumDataOutputs (ContinuousChannel::ADC);

        if (adcOutputs > 0)
        {
            return getNumDataOutputs (ContinuousChannel::ELECTRODE) + getNumDataOutputs (ContinuousChannel::AUX) + ch;
        }
        else
            return -1;
    }
    if (headstages[hs]->isConnected())
    {
        if (ch < 0)
            return -1;
        if (ch < headstages[hs]->getNumActiveChannels())
        {
            for (int i = 0; i < hs; i++)
            {
                channelCount += headstages[i]->getNumActiveChannels();
            }
            return channelCount + ch;
        }
        else if (ch < headstages[hs]->getNumActiveChannels() + 3)
        {
            for (int i = 0; i < headstages.size(); i++)
            {
                if (headstages[i]->isConnected())
                {
                    channelCount += headstages[i]->getNumActiveChannels();
                    if (i < hs)
                        hsCount++;
                }
            }
            return channelCount + hsCount * 3 + ch - headstages[hs]->getNumActiveChannels();
        }
        else
        {
            return -1;
        }
    }
    else
    {
        return -1;
    }
}

int AcqBoardOpalKelly::getHeadstageChannel (int& hs, int ch) const
{
    int channelCount = 0;
    int hsCount = 0;

    if (ch < 0)
        return -1;

    for (int i = 0; i < headstages.size(); i++)
    {
        if (headstages[i]->isConnected())
        {
            int chans = headstages[i]->getNumActiveChannels();

            if (ch >= channelCount && ch < channelCount + chans)
            {
                hs = i;
                return ch - channelCount;
            }
            channelCount += chans;
            hsCount++;
        }
    }
    if (ch < (channelCount + hsCount * 3)) //AUX
    {
        hsCount = (ch - channelCount) / 3;

        for (int i = 0; i < headstages.size(); i++)
        {
            if (headstages[i]->isConnected())
            {
                if (hsCount == 0)
                {
                    hs = i;
                    return ch - channelCount;
                }
                hsCount--;
                channelCount++;
            }
        }
    }
    return -1;
}
