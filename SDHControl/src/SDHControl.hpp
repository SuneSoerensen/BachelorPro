//
//  SDHControl.hpp
//  SDHControl
//
//  Created by <author> on 08/03/2018.
//
//

#ifndef SDHControl_hpp
#define SDHControl_hpp

#define deg2rad 0.0174532925

#include <stdio.h>
#include <rwhw/sdh/SDHDriver.hpp>
#include "Coords.hpp"

#include <rw/math/Q.hpp>
#include "sdh/sdh.h"
#include "sdh/util.h"
#include "sdh/sdhlibrary_settings.h"
#include "sdh/basisdef.h"
#include "sdhoptions.h"

USING_NAMESPACE_SDH

using namespace rwhw;
using namespace rw::common;
using namespace std;
using namespace rw::math;

class SDHControl
{
public:
    SDHControl();
    SDHControl(string canDev);

    void grasp(double distA, double distB, double distB);
    void goToInit();

    double getGraspHeight(Coords a, Coords b, Coords c);


    ~SDHControl();


    SDHDriver *sdh;
    Q initQ(7, -80*deg2rad, 80*deg2rad, 45*deg2rad, -80*deg2rad, 80*deg2rad, -80*deg2rad, 80*deg2rad);

};


#endif /* SDHControl_hpp */
