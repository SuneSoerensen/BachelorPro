# BachelorPro
## How to prepare this project for use on your (ubuntu) machine:
1. Make sure you have RobWork, OpenCV, SDH-library and PEAKCAN-driver installed and working.
2. Create a project folder, named whatever you like.
3. Add the following to your ~/.bashrc:

<pre><code>#BacherlorPro# 
export BACH_ROOT=MYPROJECTFO/  
export URCON_ROOT=MYPROJECTFO/URControl/src/  
export VISION_ROOT=MYPROJECTFO/Vision/src/  
export SDHCON_ROOT=MYPROJECTFO/SDHControl/src/  
export TIAFC_ROOT=MYPROJECTFO/TIAFC/src/
export COORDS_ROOT=MYPROJECTFO/Coords/src/
export ANALYT_ROOT=MYPROJECTFO/AnalytGrasp/src/</code></pre>

Replace 'MYPROJECTFO' with the path to your project folder.

## How to use this project to create new tests and applications:
In your CmakeLists.txt, besides your other includes etc. (such as RobWork) add the project directories as follows:

<pre><code>INCLUDE_DIRECTORIES($ENV{BACH_ROOT})
INCLUDE_DIRECTORIES($ENV{URCON_ROOT})
...and so on for all directories...</code></pre>

You shold always include `$ENV{BACH_ROOT}` as a minimum, and then your desired libraries. You can now add your desired libraries like this:

<pre><code>add_library(URControl $ENV{URCON_ROOT}/URControl.cpp)</code></pre>
