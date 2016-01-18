#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <algorithm>

// The only file that needs to be included to use the Myo C++ SDK is myo.hpp.
#include <myo/myo.hpp>

// Classes that inherit from myo::DeviceListener can be used to receive events from Myo devices. DeviceListener
// provides several virtual functions for handling different kinds of events. If you do not override an event, the
// default behavior is to do nothing.

using namespace std;

class DataCollector : public myo::DeviceListener {
public:
    std::string text = "";
    std::string conc = "";
    
    DataCollector()
    : onArm(false), isUnlocked(false), roll_w(0), pitch_w(0), yaw_w(0), currentPose()
    {
    }
    
    // onUnpair() is called whenever the Myo is disconnected from Myo Connect by the user.
    void onUnpair(myo::Myo* myo, uint64_t timestamp)
    {
        // We've lost a Myo.
        // Let's clean up some leftover state.
        roll_w = 0;
        pitch_w = 0;
        yaw_w = 0;
        onArm = false;
        isUnlocked = false;
    }
    
    // onOrientationData() is called whenever the Myo device provides its current orientation, which is represented
    // as a unit quaternion.
    void onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& quat)
    {
        using std::atan2;
        using std::asin;
        using std::sqrt;
        using std::max;
        using std::min;
        
        // Calculate Euler angles (roll, pitch, and yaw) from the unit quaternion.
        // Use this feature to set different areas to represent a different group of letters
        float roll = atan2(2.0f * (quat.w() * quat.x() + quat.y() * quat.z()),
                           1.0f - 2.0f * (quat.x() * quat.x() + quat.y() * quat.y()));
        float pitch = asin(max(-1.0f, min(1.0f, 2.0f * (quat.w() * quat.y() - quat.z() * quat.x()))));
        float yaw = atan2(2.0f * (quat.w() * quat.z() + quat.x() * quat.y()),
                          1.0f - 2.0f * (quat.y() * quat.y() + quat.z() * quat.z()));
        
        // Convert the floating point angles in radians to a scale from 0 to 18.
        /*roll_w = static_cast<int>((roll + (float)M_PI)/(M_PI * 2.0f) * 18);
         pitch_w = static_cast<int>((pitch + (float)M_PI/2.0f)/M_PI * 18);
         yaw_w = static_cast<int>((yaw + (float)M_PI)/(M_PI * 2.0f) * 18);
         */
        
        roll_w = roll;
        pitch_w = pitch;
        yaw_w = yaw;
        
        yaw_f = yaw_w - yaw_ref;
        roll_f = roll_w - roll_ref;
        pitch_f = pitch_w - pitch_ref;
        
    }
    
    // onPose() is called whenever the Myo detects that the person wearing it has changed their pose, for example,
    // making a fist, or not making a fist anymore.
    // Use function to force reset roll, pitch, yaw after every movement
    void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose)
    {
        currentPose = pose;
        
        if (pose == myo::Pose::fist)
        {
            yaw_ref = yaw_w;
            roll_ref = roll_w;
            pitch_ref = pitch_w;
            //yaw_f = 0, roll_f = 0, pitch_f = 0;
        }
    }
    
    // onArmSync() is called whenever Myo has recognized a Sync Gesture after someone has put it on their
    // arm. This lets Myo know which arm it's on and which way it's facing.
    void onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection, float rotation,
                   myo::WarmupState warmupState)
    {
        onArm = true;
        whichArm = arm;
    }
    
    // onArmUnsync() is called whenever Myo has detected that it was moved from a stable position on a person's arm after
    // it recognized the arm. Typically this happens when someone takes Myo off of their arm, but it can also happen
    // when Myo is moved around on the arm.
    void onArmUnsync(myo::Myo* myo, uint64_t timestamp)
    {
        onArm = false;
    }
    
    // onUnlock() is called whenever Myo has become unlocked, and will start delivering pose events.
    void onUnlock(myo::Myo* myo, uint64_t timestamp)
    {
        isUnlocked = true;
    }
    
    // onLock() is called whenever Myo has become locked. No pose events will be sent until the Myo is unlocked again.
    void onLock(myo::Myo* myo, uint64_t timestamp)
    {
        isUnlocked = false;
    }
    
    /*  void onGyroscopeData(myo::Myo *myo, uint64_t timestamp, const myo::Vector3< float > &gyro) {
     printVector(gyroFile, timestamp, gyro);
     
     }
     */
    
    // There are other virtual functions in DeviceListener that we could override here, like onAccelerometerData().
    // For this example, the functions overridden above are sufficient.
    
    // We define this function to print the current values that were updated by the on...() functions above.
    void print()
    {
        // Clear the current line
        std::cout << '\r';
        
        // Print out the orientation. Orientation data is always available, even if no arm is currently recognized.
        /*std::cout << '[' << std::string(roll_w, '*') << std::string(18 - roll_w, ' ') << ']'
         << '[' << std::string(pitch_w, '*') << std::string(18 - pitch_w, ' ') << ']'
         << '[' << std::string(yaw_w, '*') << std::string(18 - yaw_w, ' ') << ']'; */
        // std::cout << roll_w << endl;
        // std::cout << pitch_w << endl;
        std::cout << "yaw_f: " <<yaw_f <<"    roll_f: "<< roll_f << "    pitch_f: " <<pitch_f << std::endl;
        
        //conditions for different letters
        if (yaw_f > 0.5 && yaw_f < 2.0){
            if (currentPose == myo::Pose::fingersSpread) {
                cout << "O" << endl;
            }
            else if (currentPose == myo::Pose::waveIn) {
                cout << "M" << endl;
            }
            else if (currentPose == myo::Pose::waveOut) {
                cout << "N" << endl;
            }
        }
        else if (yaw_f < -0.8){
            if (currentPose == myo::Pose::fingersSpread) {
                cout << "G" << endl;
            }
            else if (currentPose == myo::Pose::waveIn) {
                cout << "I" << endl;
            }
            else if (currentPose == myo::Pose::waveOut) {
                cout << "H" << endl;
            }
            else if (currentPose == myo::Pose::doubleTap) {
                cout << endl;
            }
        }
        else if (pitch_f > 0.8){
            if (currentPose == myo::Pose::fingersSpread) {
                cout << "C" << endl;
            }
            else if (currentPose == myo::Pose::waveIn) {
                cout << "A" << endl;
            }
            else if (currentPose == myo::Pose::waveOut) {
                cout << "B" << endl;
            }
            else if (currentPose == myo::Pose::doubleTap) {
                cout << " " << endl;
            }
        }
        else if (pitch_f < -0.8){
            if (currentPose == myo::Pose::fingersSpread) {
                cout << "V" << endl;
            }
            else if (currentPose == myo::Pose::waveIn) {
                cout << "T" << endl;
            }
            else if (currentPose == myo::Pose::waveOut) {
                cout << "U" << endl;
            }
        }
        else if (roll_f > 2){
            if (currentPose == myo::Pose::fingersSpread) {
                cout << "F" << endl;
            }
            else if (currentPose == myo::Pose::waveIn) {
                cout << "D" << endl;
            }
            else if (currentPose == myo::Pose::waveOut) {
                cout << "E" << endl;
            }
        }
        else if (roll_f < -0.35){
            if (currentPose == myo::Pose::fingersSpread) {
                cout << "P" << endl;
            }
            else if (currentPose == myo::Pose::waveIn) {
                cout << "Q" << endl;
            }
            else if (currentPose == myo::Pose::waveOut) {
                cout << "R" << endl;
            }
            else if (currentPose == myo::Pose::doubleTap){
                cout << "S" << endl;
            }
        }
        //diagonals
        else if (yaw_f < 0.5 && yaw_f > 0){
            if (currentPose == myo::Pose::fingersSpread) {
                cout << "W" << endl;
            }
            else if (currentPose == myo::Pose::waveIn) {
                cout << "X" << endl;
            }
            else if (currentPose == myo::Pose::waveOut) {
                cout << "Y" << endl;
            }
        }
        else if (yaw_f < 0 && yaw_f > -0.8) {
            if (currentPose == myo::Pose::fingersSpread) {
                cout << "Z" << endl;
            }
            else if (currentPose == myo::Pose::waveIn) {
                cout << "J" << endl;
            }
            else if (currentPose == myo::Pose::waveOut) {
                cout << "K" << endl;
            }
            else if (currentPose == myo::Pose::doubleTap && roll_f > -0.35) {
                cout << "L" << endl;
            }
        }
        
        if (onArm) {
            // Print out the lock state, the currently recognized pose, and which arm Myo is being worn on.
            
            // Pose::toString() provides the human-readable name of a pose. We can also output a Pose directly to an
            // output stream (e.g. std::cout << currentPose;). In this case we want to get the pose name's length so
            // that we can fill the rest of the field with spaces below, so we obtain it as a string using toString().
            std::string poseString = currentPose.toString();
            
            std::cout //<< '[' << (isUnlocked ? "unlocked" : "locked  ") << ']'
            << '[' << (whichArm == myo::armLeft ? "L" : "R") << ']'
            << '[' << poseString << std::string(14 - poseString.size(), ' ') << ']';
        }
        else {
            // Print out a placeholder for the arm and pose when Myo doesn't currently know which arm it's on.
            std::cout << '[' << std::string(8, ' ') << ']' << "[?]" << '[' << std::string(14, ' ') << ']';
        }
        
        std::cout << std::flush;
    }
    
    // These values are set by onArmSync() and onArmUnsync() above.
    bool onArm;
    myo::Arm whichArm;
    
    // This is set by onUnlocked() and onLocked() above.
    bool isUnlocked;
    
    // These values are set by onOrientationData() and onPose() above.
    float roll_w, pitch_w, yaw_w;
    float roll_ref, pitch_ref, yaw_ref;
    float roll_f, pitch_f, yaw_f;
    myo::Pose currentPose;
};

int main(int argc, char** argv)
{
    // We catch any exceptions that might occur below -- see the catch statement for more details.
    try {
        
        // First, we create a Hub with our application identifier. Be sure not to use the com.example namespace when
        // publishing your application. The Hub provides access to one or more Myos.
        myo::Hub hub("com.example.hello-myo");
        
        std::cout << "Attempting to find a Myo..." << std::endl;
        
        // Next, we attempt to find a Myo to use. If a Myo is already paired in Myo Connect, this will return that Myo
        // immediately.
        // waitForMyo() takes a timeout value in milliseconds. In this case we will try to find a Myo for 10 seconds, and
        // if that fails, the function will return a null pointer.
        myo::Myo* myo = hub.waitForMyo(10000);
        
        // If waitForMyo() returned a null pointer, we failed to find a Myo, so exit with an error message.
        if (!myo) {
            throw std::runtime_error("Unable to find a Myo!");
        }
        
        // We've found a Myo.
        std::cout << "Connected to a Myo armband!" << std::endl << std::endl;
        
        // set locking policy to none
        hub.setLockingPolicy(myo::Hub::lockingPolicyNone);
        
        
        // Next we construct an instance of our DeviceListener, so that we can register it with the Hub.
        DataCollector collector;
        
        // Hub::addListener() takes the address of any object whose class inherits from DeviceListener, and will cause
        // Hub::run() to send events to all registered device listeners.
        hub.addListener(&collector);
        
        // Finally we enter our main loop.
        while (1) {
            // In each iteration of our main loop, we run the Myo event loop for a set number of milliseconds.
            // In this case, we wish to update our display 20 times a second, so we run for 1000/20 milliseconds.
            hub.run(250);
            // After processing events, we call the print() member function we defined above to print out the values we've
            // obtained from any events that have occurred.
            collector.print();
        }
        
        // If a standard exception occurred, we print out its message and exit.
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        std::cerr << "Press enter to continue.";
        std::cin.ignore();
        return 1;
    }
}