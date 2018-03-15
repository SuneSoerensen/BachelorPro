# BachelorPro
## How to prepare this project for use on your (ubuntu) machine:
1. Make sure you have RobWork, OpenCV, SDH-library and PEAKCAN-driver installed and working.
2. Create a project folder, named whatever you like.
3. Add the following to your ~/.bashrc:

<pre><code>#BacherlorPro# 
export BACH_ROOT=MYPROJECTFO/  
export URCON_ROOT=MYPROJECTFO/URControl/src/  
export CONTPROC_ROOT=MYPROJECTFO/ContProc/src/  
export SDHCON_ROOT=MYPROJECTFO/SDHControl/src/  
export TIAFC_ROOT=MYPROJECTFO/TIAFC/src/
export COORDS_ROOT=MYPROJECTFO/Coords/src/</code></pre>

## How to use this project to create new tests and applications:
In your CmakeLists.txt, besides your other includes etc. (such as RobWork) add the project directories as follows:

<pre><code>INCLUDE_DIRECTORIES($ENV{BACH_ROOT})
INCLUDE_DIRECTORIES($ENV{URCON_ROOT})</code></pre>

You shold always include `$ENV{BACH_ROOT}` as a minimun, and then your desired libraries. You can then add the libraries:

<pre><code>add_library(URControl $ENV{URCON_ROOT}/URControl.cpp)</code></pre>
