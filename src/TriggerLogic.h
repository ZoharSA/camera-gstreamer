#include <mutex>
#include <unordered_map>
#include <memory>
#include <vector>
#include <assert.h>

#include <camera-interface.h>
#include <CameraGstreamer.h>


class TriggerLogic
{

public:
    TriggerLogic(
        std::shared_ptr< std::vector<CameraGstreamer*> > cameras,
        std::shared_ptr<std::mutex> triggerCameraStateMutex
    );

    ~TriggerLogic();

    void adjustTrigger(CameraId id, bool active);
   
private:
    CameraId _currentTrigger;
    CameraId _definedTriggerId;
    CameraId _invalidCameraId;
    std::shared_ptr< std::vector<CameraGstreamer*> > _cameras;
    std::shared_ptr<std::mutex>  _triggerCameraStateMutex;

    bool allCamerasStopped();
    void changeTrigger(CameraId id, bool trigger);
    CameraId findTopPriorityActiveCamera();

    TriggerLogic(const TriggerLogic & rhs) = delete;
    TriggerLogic & operator=(const TriggerLogic & rhs) = delete;
};
