#pragma once

#include "rhd2000ONIdatablock.h"
#include <oni.h>
#include <queue>
#include <vector>

#define MAX_NUM_DATA_STREAMS_USB3 16

class Rhd2000ONIBoard
{
public:
    Rhd2000ONIBoard();
    ~Rhd2000ONIBoard();

    int MAX_NUM_DATA_STREAMS;
    enum AuxCmdSlot
    {
        AuxCmd1,
        AuxCmd2,
        AuxCmd3
    };

    enum BoardPort
    {
        PortA = 0,
        PortB,
        PortC,
        PortD
    };

    enum BoardDataSource
    {
        PortA1 = 0,
        PortA2 = 1,
        PortB1 = 2,
        PortB2 = 3,
        PortC1 = 4,
        PortC2 = 5,
        PortD1 = 6,
        PortD2 = 7,
        PortA1Ddr = 8,
        PortA2Ddr = 9,
        PortB1Ddr = 10,
        PortB2Ddr = 11,
        PortC1Ddr = 12,
        PortC2Ddr = 13,
        PortD1Ddr = 14,
        PortD2Ddr = 15
    };

    enum AmplifierSampleRate
    {
        SampleRate1000Hz,
        SampleRate1250Hz,
        SampleRate1500Hz,
        SampleRate2000Hz,
        SampleRate2500Hz,
        SampleRate3000Hz,
        SampleRate3333Hz,
        SampleRate5000Hz,
        SampleRate6250Hz,
        SampleRate10000Hz,
        SampleRate12500Hz,
        SampleRate15000Hz,
        SampleRate20000Hz,
        SampleRate25000Hz,
        SampleRate30000Hz
    };

    bool isUSB3();
    int open (const oni_driver_info_t** driverInfo = nullptr);
    void initialize();

    void setCableDelay (BoardPort port, int delay);
    void setCableLengthMeters (BoardPort port, double lengthInMeters);
    void setCableLengthFeet (BoardPort port, double lengthInFeet);
    void uploadCommandList (const std::vector<int>& commandList, AuxCmdSlot auxCommandSlot, int bank);
    double estimateCableLengthMeters (int delay) const;

    void printCommandList (const std::vector<int>& commandList) const;
    void selectAuxCommandBank (BoardPort port, AuxCmdSlot auxCommandSlot, int bank);
    void selectAuxCommandLength (AuxCmdSlot auxCommandSlot, int loopIndex, int endIndex);

    void resetBoard();
    void setContinuousRunMode (bool continuousMode);
    void setMaxTimeStep (unsigned int maxTimeStep);
    void run();
    void stop();

    bool isRunning() const;
    int getNumEnabledDataStreams() const;

    void setDataSource (int stream, BoardDataSource dataSource);
    void enableDataStream (int stream, bool enabled);

    //This must be called after the number of streams have been changed, i.e.: after a set of calls to enableDataStream,
    //to update the Rhythm device block size.
    void updateStreamBlockSize();

    bool setSampleRate (AmplifierSampleRate newSampleRate);
    double getSampleRate() const;

    void setDspSettle (bool enabled);

    void setAudioNoiseSuppress (int noiseSuppress);
    void enableExternalFastSettle (bool enable);
    void setExternalFastSettleChannel (int channel);
    void enableExternalDigOut (BoardPort port, bool enable);
    void setExternalDigOutChannel (BoardPort port, int channel);
    void selectDacDataStream (int dacChannel, int stream);
    void selectDacDataChannel (int dacChannel, int dataChannel);
    void enableDacHighpassFilter (bool enable);
    void setDacHighpassFilter (double cutoff);
    void setDacThreshold (int dacChannel, int threshold, bool trigPolarity);
    void setTtlMode (int mode);

    bool isStreamEnabled (int streamIndex);
    void enableBoardLeds (bool enable);
    void setClockDivider (int divide_factor);

    bool readDataBlock (Rhd2000ONIDataBlock* dataBlock, int nSamples = -1);
    bool readDataBlocks (int numBlocks, std::queue<Rhd2000ONIDataBlock>& dataqueue);

    int readFrame (oni_frame_t** frame);

    void setTtlOut (int ttlOutArray[16]);
    void clearTtlOut();

    void enableDac (int dacChannel, bool enabled);
    void setDacManual (int value);
    void setDacGain (int gain);

    bool getFirmwareVersion (int* major, int* minor) const;

    void getONIVersion (int* major, int* minor, int* patch);
    void getONIDriverInfo (const oni_driver_info_t** driverInfo);
    bool getFTDriverInfo (int* major, int* minor, int* patch);
    bool getFTLibInfo (int* major, int* minor, int* patch);

    enum BoardMemState
    {
        BOARDMEM_INIT = 0,
        BOARDMEM_OK = 1,
        BOARDMEM_INVALID = 2, //this should not happen
        BOARDMEM_ERR = 3
    };

    BoardMemState getBoardMemState() const;

private:
    const oni_size_t usbReadBlockSize = 24 * 1024;

    static int oni_write_reg_mask (const oni_ctx ctx, oni_dev_idx_t dev_idx, oni_reg_addr_t addr, oni_reg_val_t value, unsigned int mask);

    enum Rhythm_Registers
    {
        ENABLE = 0,
        MODE,
        MAX_TIMESTEP,
        CABLE_DELAY,
        AUXCMD_BANK_1,
        AUXCMD_BANK_2,
        AUXCMD_BANK_3,
        MAX_AUXCMD_INDEX_1,
        MAX_AUXCMD_INDEX_2,
        MAX_AUXCMD_INDEX_3,
        LOOP_AUXCMD_INDEX_1,
        LOOP_AUXCMD_INDEX_2,
        LOOP_AUXCMD_INDEX_3,
        DATA_STREAM_1_8_SEL,
        DATA_STREAM_9_16_SEL,
        DATA_STREAM_EN,
        EXTERNAL_FAST_SETTLE,
        EXTERNAL_DIGOUT_A,
        EXTERNAL_DIGOUT_B,
        EXTERNAL_DIGOUT_C,
        EXTERNAL_DIGOUT_D,
        SYNC_CLKOUT_DIVIDE,
        DAC_CTL,
        DAC_SEL_1,
        DAC_SEL_2,
        DAC_SEL_3,
        DAC_SEL_4,
        DAC_SEL_5,
        DAC_SEL_6,
        DAC_SEL_7,
        DAC_SEL_8,
        DAC_THRESH_1,
        DAC_THRESH_2,
        DAC_THRESH_3,
        DAC_THRESH_4,
        DAC_THRESH_5,
        DAC_THRESH_6,
        DAC_THRESH_7,
        DAC_THRESH_8,
        HPF,
        SPI_RUNNING
    };

    enum Rhythm_Mode
    {
        SPI_RUN_CONTINUOUS = 1,
        DSP_SETTLE = 2,
        TTL_OUT_MODE = 3,
        LED_ENABLE = 4
    };

    /** ONI device indices*/
    const oni_dev_idx_t DEVICE_RHYTHM = 0x0101;
    const oni_dev_idx_t DEVICE_TTL = 0x0102;
    const oni_dev_idx_t DEVICE_DAC = 0x0103;

    const oni_dev_idx_t RHYTHM_HUB_MANAGER = 0x01FE;
    const oni_reg_addr_t HUB_CLOCK_SEL = 0x2000;
    const oni_reg_addr_t HUB_CLOCK_BUSY = 0x2001;

    /** The ONI context object */
    oni_ctx ctx;

    AmplifierSampleRate sampleRate;
    int numDataStreams; // total number of data streams currently enabled
    int dataStreamEnabled[MAX_NUM_DATA_STREAMS_USB3]; // 0 (disabled) or 1 (enabled), set for maximum stream number
    std::vector<int> cableDelay;
};
