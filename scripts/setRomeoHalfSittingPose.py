import sys
import time
from naoqi import ALProxy
import romeo_common


def main(robotIP):
    PORT = 9559

    try:
        motionProxy = ALProxy("ALMotion", robotIP, PORT)
    except Exception,e:
        print "Could not create proxy to ALMotion"
        print "Error was: ",e
        sys.exit(1)
    

    # Set robot to pose HalfSitting used in multi-contact

    halfSitting = romeo_common.halfSitting
    angleLists = []
    timeLists = []

    # Get the names of all the joints in the body.
    bodyNames = motionProxy.getBodyNames("Body")
    print "Body:"
    print str(bodyNames)
    print ""

    print halfSitting

    for name in bodyNames:
    	if (halfSitting.has_key(name)):
            if name == 'RAnkleRoll':
                angleLists.append(0.0260776)
                timeLists.append(2.0)
            elif (name == 'LAnkleRoll'):
                angleLists.append(-0.087436)
                timeLists.append(2.0)
            else:
                angleLists.append(halfSitting.get(name))
                timeLists.append(2.0)
    	else:
    		print "Error:  %d is not a valid joint name." % name
    		sys.exit(1)

    print "Values:"
    print  angleLists

    motionProxy.setStiffnesses(bodyNames, 1.0)
    isAbsolute = True
    motionProxy.angleInterpolation(bodyNames, angleLists, timeLists, isAbsolute)


if __name__ == "__main__":
    robotIp = "198.18.0.1"

    if len(sys.argv) <= 1:
        print "Usage python almotion_changeangles.py robotIP (optional default: 127.0.0.1)"
    else:
        robotIp = sys.argv[1]

    main(robotIp)