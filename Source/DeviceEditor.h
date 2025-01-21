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

#ifndef __DEVICEEDITOR_H_2AD3C591__
#define __DEVICEEDITOR_H_2AD3C591__

#include <VisualizerEditorHeaders.h>

class HeadstageOptionsInterface;
class SampleRateInterface;
class BandwidthInterface;
class DSPInterface;
class AudioInterface;
class ClockDivideInterface;
class DeviceThread;
class ChannelCanvas;

struct ImpedanceData;

class DeviceEditor : public VisualizerEditor,
                     public ComboBox::Listener,
                     public Button::Listener,
                     public PopupChannelSelector::Listener

{
public:
    /** Constructor */
    DeviceEditor (GenericProcessor* parentNode, class AcquisitionBoard* board);

    /** Destructor*/
    ~DeviceEditor() {}

    /** Respond to combo box changes (e.g. sample rate)*/
    void comboBoxChanged (ComboBox* comboBox);

    /** Respond to button clicks*/
    void buttonClicked (Button* button);

    /** Disable UI during acquisition*/
    void startAcquisition();

    /** Enable UI after acquisition is finished*/
    void stopAcquisition();

    /** Runs impedance test*/
    void measureImpedances();

    /** Callback when impedance measurement is finished */
    void impedanceMeasurementFinished();

    /** Saves impedance data to a file */
    void saveImpedances (File& file);

    /** Updates channel canvas*/
    void updateSettings();

    /** Saves custom parameters */
    void saveVisualizerEditorParameters (XmlElement* xml) override;

    /** Loads custom parameters*/
    void loadVisualizerEditorParameters (XmlElement* xml) override;

    /** Creates an interface with additional channel settings*/
    Visualizer* createNewCanvas (void);

    /** Called by PopupChannelSelector */
    void channelStateChanged (Array<int> newChannels) override;

    virtual Array<int> getSelectedChannels() { return Array<int>(); }

private:
    /** Pointer to acquisition board device */
    class AcquisitionBoard* board;

    /** Pointer to visualizer canvas */
    ChannelCanvas* canvas;

    /** XmlElement to hold previously saved parameters if no device is found */
    std::unique_ptr<XmlElement> previousSettings;

    OwnedArray<HeadstageOptionsInterface> headstageOptionsInterfaces;
    OwnedArray<ElectrodeButton> electrodeButtons;

    std::unique_ptr<SampleRateInterface> sampleRateInterface;
    std::unique_ptr<BandwidthInterface> bandwidthInterface;
    std::unique_ptr<DSPInterface> dspInterface;

    std::unique_ptr<AudioInterface> audioInterface;
    std::unique_ptr<ClockDivideInterface> clockInterface;

    std::unique_ptr<UtilityButton> rescanButton, dacTTLButton;
    std::unique_ptr<UtilityButton> auxButton;
    std::unique_ptr<UtilityButton> adcButton;
    std::unique_ptr<UtilityButton> ledButton;

    std::unique_ptr<UtilityButton> dspoffsetButton;
    std::unique_ptr<ComboBox> ttlSettleCombo, dacHPFcombo;
    std::unique_ptr<Label> audioLabel, ttlSettleLabel, dacHPFlabel;
    std::unique_ptr<Label> noBoardsDetectedLabel;

    enum AudioChannel
    {
        LEFT = 0,
        RIGHT = 1
    };

    AudioChannel activeAudioChannel;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (DeviceEditor);
};

/** 
    Holds buttons for headstages on one port.

    If a 32-channel headstages is detected, it 
    allows the user to toggle between 16 and 32-channel mode

*/
class HeadstageOptionsInterface : public Component,
                                  public Button::Listener
{
public:
    /** Constructor*/
    HeadstageOptionsInterface (class AcquisitionBoard*, DeviceEditor*, int hsNum);

    /** Destructor */
    ~HeadstageOptionsInterface();

    /** Draw the options interface background */
    void paint (Graphics& g);

    /** Toggle between 16 and 32 ch */
    void buttonClicked (Button* button);

    /** Refresh button state*/
    void checkEnabledState();

    /** Set enabled (e.g. during acquisition) */
    void setEnabled (bool state);

    /** Checks whether headstage is in 32- or 16-channel mode*/
    bool is32Channel (int hsIndex);

    /** Sets HS in 32- or 16-ch mode */
    void set32Channel (int hsIndex, bool is32Channel);

private:
    int hsNumber1, hsNumber2;
    int channelsOnHs1, channelsOnHs2;
    String name;

    bool isEnabled;

    class AcquisitionBoard* board;
    DeviceEditor* editor;

    std::unique_ptr<UtilityButton> hsButton1;
    std::unique_ptr<UtilityButton> hsButton2;
};

/** 
    Holds settings for RHD chip analog filters

*/
class BandwidthInterface : public Component,
                           public Label::Listener
{
public:
    /** Constructor */
    BandwidthInterface (class AcquisitionBoard*, DeviceEditor*);

    /** Destructor */
    ~BandwidthInterface();

    /** Draw interface labels */
    void paint (Graphics& g);

    /** Called when settings are changed */
    void labelTextChanged (Label* te);

    /** Sets lower bandwidth value */
    void setLowerBandwidth (double value);

    /** Sets upper bandwidth value */
    void setUpperBandwidth (double value);

    /** Returns actual lower bandwidth value */
    double getLowerBandwidth();

    /** Returns actual upper bandwidth value */
    double getUpperBandwidth();

private:
    String name;

    String lastLowCutString, lastHighCutString;

    class AcquisitionBoard* board;
    DeviceEditor* editor;

    std::unique_ptr<Label> upperBandwidthSelection;
    std::unique_ptr<Label> lowerBandwidthSelection;

    double actualUpperBandwidth;
    double actualLowerBandwidth;
};

/** 
    Holds settings for digital on-chip filters
    
*/
class DSPInterface : public Component,
                     public Label::Listener
{
public:
    /** Constructor */
    DSPInterface (class AcquisitionBoard*, DeviceEditor*);

    /** Destructor */
    ~DSPInterface();

    /** Draw interface labels */
    void paint (Graphics& g);

    /** Called when settings are changed */
    void labelTextChanged (Label* te);

    /** Sets DSP cutoff frequency */
    void setDspCutoffFreq (double value);

    /** Returns actual DSP cutoff frequency */
    double getDspCutoffFreq();

private:
    String name;

    class AcquisitionBoard* board;
    DeviceEditor* editor;

    std::unique_ptr<Label> dspOffsetSelection;

    double actualDspCutoffFreq = 0.5;
};

/**

   Holds sample rate settings

*/
class SampleRateInterface : public Component,
                            public ComboBox::Listener
{
public:
    /** Constructor */
    SampleRateInterface (class AcquisitionBoard*, DeviceEditor*);

    /** Destructor */
    ~SampleRateInterface();

    /** Returns index of selected sample rate */
    int getSelectedId();

    /** Sets sample rate by index */
    void setSelectedId (int);

    /** Returns sample rate string */
    String getText();

    /** Draw interface labels */
    void paint (Graphics& g);

    /** Called when settings are changed */
    void comboBoxChanged (ComboBox* cb);

private:
    int sampleRate;
    String name;

    class AcquisitionBoard* board;
    DeviceEditor* editor;

    std::unique_ptr<ComboBox> rateSelection;
    StringArray sampleRateOptions;
};

/** 
    Holds settings for audio output
    
*/
class AudioInterface : public Component,
                       public Label::Listener
{
public:
    /** Constructor */
    AudioInterface (class AcquisitionBoard*, DeviceEditor*);

    /** Destructor */
    ~AudioInterface();

    /** Draw interface labels */
    void paint (Graphics& g);

    /** Called when settings are changed */
    void labelTextChanged (Label* te);

    /** Sets noise slicer level (used to reduce background noise) */
    void setNoiseSlicerLevel (int value);

    /** Returns actual noise slicer level */
    int getNoiseSlicerLevel();

private:
    String name;

    String lastNoiseSlicerString;
    String lastGainString;

    class AcquisitionBoard* board;
    DeviceEditor* editor;

    std::unique_ptr<Label> noiseSlicerLevelSelection;

    int actualNoiseSlicerLevel;
};

/** 
    Holds settings for clock divider 

    The clock divider set the ratio of the sample rate
    at which the digital output on the sync BNC is updated

    For example, if the sample rate is 30 kHz and the clock
    divider is set to 10, the sync output will be updated at 3 kHz

*/
class ClockDivideInterface : public Component,
                             public Label::Listener
{
public:
    /** Constructor */
    ClockDivideInterface (class AcquisitionBoard*, DeviceEditor*);

    /** Draws the interface labels */
    void paint (Graphics& g);

    /** Called when settings are changed */
    void labelTextChanged (Label* te);

    /** Sets clock divide ratio */
    void setClockDivideRatio (int value);

    /** Returns actual clock divide ratio */
    int getClockDivideRatio() const { return actualDivideRatio; };

private:
    String name;
    String lastDivideRatioString;

    class AcquisitionBoard* board;
    DeviceEditor* editor;

    std::unique_ptr<Label> divideRatioSelection;
    int actualDivideRatio;
};

#endif // __DEVICEEDITOR_H_2AD3C591__
