#include <rw/kinematics/FixedFrame.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/models/TreeDevice.hpp>
#include <vector>

using rw::common::Log;
using namespace rw::kinematics;
using rw::loaders::WorkCellLoader;
using namespace rw::math;
using namespace rw::models;

int main(int argc, char** argv) {

    WorkCell::Ptr wc = WorkCellLoader::Factory::load("XMLtest/Workcell.xml");
    if (wc.isNull())
    	RW_THROW("Could not load workcell!");

    Log::infoLog() << "Name of workcell: " << wc->getName() << std::endl;
    // get the default state
    State state = wc->getDefaultState();
    Frame* worldFrame = wc->getWorldFrame();
    // find a frame by name, remember NULL is a valid return
    Frame* urframe = wc->findFrame("UR-6-85-5-A");
    Frame* sdhframe = wc->findFrame("handBase");
    // find a frame by name, but with a specific frame type
    //FixedFrame* fframe = wc->findFrame<FixedFrame>("UR-6-85-5-A");
    std::vector<Frame*> test;
    //test = wc->findFrames<EndEffector>();
    for (int i = 0; i < test.size(); i++) {
      std::cout << "frame name " << test[i]->getName() << '\n';
    }

    MovableFrame* mframe = wc->findFrame<MovableFrame>("UR-6-85-5-A.BaseMov");
    if(mframe == NULL)
    {
      std::cout << "mframe is null" << '\n';
    }
    // find a device by name
    TreeDevice::Ptr tdevice = wc->findDevice<TreeDevice>("SchunkHand");
    SerialDevice::Ptr sdevice = wc->findDevice<SerialDevice>("UR-6-85-5-A");




     //calculate the transform from one frame to another
    Transform3D<> fTmf = Kinematics::frameTframe(urframe, mframe, state);
    // calculate the transform from world to frame
    Transform3D<> wTmf = Kinematics::worldTframe( mframe, state );
    // we can find the world to frame transform by a little jogling
    Transform3D<> wTf = wTmf * inverse(fTmf);
    // test if frame is a dynamic attachable frame
    if( Kinematics::isDAF( mframe ) ){
        // attach mframe to end of serial device
        Kinematics::gripFrame(mframe, sdevice->getEnd(), state);
    }


     //get device base frame
    Frame *base = sdevice->getBase();
    // get device end effector
    Frame *end = sdevice->getEnd();
    // calculate base to end transform
    Transform3D<> bTe = sdevice->baseTend(state);
    // or just base to any frame
    Transform3D<> bTmf = sdevice->baseTframe(mframe, state);
    // get device name
    std::string sdevicename = sdevice->getName();
    // the degrees of freedom of this device
    int dof = sdevice->getDOF();
    // set the configuration of the device to zero
    sdevice->setQ( Q::zero(dof) , state );

    return 0;
}
