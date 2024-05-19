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

#ifndef __IMPEDANCEMETER_H_2C4CBD67__
#define __IMPEDANCEMETER_H_2C4CBD67__

#include <DataThreadHeaders.h>

struct Impedances
{
    Array<int> streams;
    Array<int> channels;
    Array<float> magnitudes;
    Array<float> phases;
    bool valid = false;
};

class ImpedanceMeter : public ThreadWithProgressWindow
{
public:
    /** Constructor*/
    ImpedanceMeter() : ThreadWithProgressWindow (
        "Impedance Measurement",
        true,
        true)
    {
    }

    /** Destructor*/
    virtual ~ImpedanceMeter()
    {
        stopThreadSafely();
    }

    /** Calculates impedance values for all channels*/
    virtual void runImpedanceMeasurement (Impedances& impedances) = 0;

    /** Interrupt impedance measurement thread*/
    virtual void stopThreadSafely()
    {
        if (isThreadRunning())
        {
            CoreServices::sendStatusMessage ("Impedance measurement in progress. Stopping it.");

            if (! stopThread (3000)) //wait three seconds max for it to exit gracefully
            {
                std::cerr << "ERROR: Impedance measurement did not exit." << std::endl;
            }
        }
    }

    /** Wait for thread to finish*/
    virtual void waitSafely()
    {
        if (! waitForThreadToExit (120000)) //two minutes should be enough for completing a scan
        {
            CoreServices::sendStatusMessage ("Impedance measurement took too much time. Aborting.");

            if (! stopThread (3000)) //wait three seconds max for it to exit gracefully
            {
                std::cerr << "ERROR: Impedance measurement thread did not exit." << std::endl;
            }
        }
    }

    // Allocates memory for a 3-D array of doubles (helper function)
    void allocateDoubleArray3D (std::vector<std::vector<std::vector<double>>>& array3D,
                                int xSize,
                                int ySize,
                                int zSize)
    {
        int i, j;

        if (xSize == 0)
            return;
        array3D.resize (xSize);
        for (i = 0; i < xSize; ++i)
        {
            array3D[i].resize (ySize);
            for (j = 0; j < ySize; ++j)
            {
                array3D[i][j].resize (zSize);
            }
        }
    }

private:
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (ImpedanceMeter);
};



#endif // __RHD2000THREAD_H_2C4CBD67__
