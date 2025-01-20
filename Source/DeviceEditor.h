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

class BandwidthInterface : public Component,
                           public Label::Listener
{
public:
    BandwidthInterface (class AcquisitionBoard*, DeviceEditor*);
    ~BandwidthInterface();

    void paint (Graphics& g);
    void labelTextChanged (Label* te);

    void setLowerBandwidth (double value);
    void setUpperBandwidth (double value);
    double getLowerBandwidth();
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

class DSPInterface : public Component,
                     public Label::Listener
{
public:
    DSPInterface (class AcquisitionBoard*, DeviceEditor*);
    ~DSPInterface();

    void paint (Graphics& g);
    void labelTextChanged (Label* te);

    void setDspCutoffFreq (double value);
    double getDspCutoffFreq();

private:
    String name;

    class AcquisitionBoard* board;
    DeviceEditor* editor;

    std::unique_ptr<Label> dspOffsetSelection;

    double actualDspCutoffFreq;
};

class SampleRateInterface : public Component,
                            public ComboBox::Listener
{
public:
    SampleRateInterface (class AcquisitionBoard*, DeviceEditor*);
    ~SampleRateInterface();

    int getSelectedId();
    void setSelectedId (int);

    String getText();

    void paint (Graphics& g);
    void comboBoxChanged (ComboBox* cb);

private:
    int sampleRate;
    String name;

    class AcquisitionBoard* board;
    DeviceEditor* editor;

    std::unique_ptr<ComboBox> rateSelection;
    StringArray sampleRateOptions;
};

class AudioInterface : public Component,
                       public Label::Listener
{
public:
    AudioInterface (class AcquisitionBoard*, DeviceEditor*);
    ~AudioInterface();

    void paint (Graphics& g);
    void labelTextChanged (Label* te);

    void setNoiseSlicerLevel (int value);
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

class ClockDivideInterface : public Component,
                             public Label::Listener
{
public:
    ClockDivideInterface (class AcquisitionBoard*, DeviceEditor*);

    void paint (Graphics& g);
    void labelTextChanged (Label* te);

    void setClockDivideRatio (int value);
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
