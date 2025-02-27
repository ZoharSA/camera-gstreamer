#include <TriggerLogic.h>

#include <iostream>
#include <string>
#include <assert.h>
#include <string.h>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

TriggerLogic::TriggerLogic(
    std::shared_ptr< std::vector<CameraGstreamer*> > cameras,
    std::shared_ptr<std::mutex> triggerCameraStateMutex
):
    _cameras(cameras),
    _triggerCameraStateMutex(triggerCameraStateMutex),
    _currentTrigger(0),
    _definedTriggerId(0)
{
    CameraId id = 0;
    for(auto camera: *_cameras) {
        if (camera->getIsTrigger()) {
            _currentTrigger = id;
            _definedTriggerId = id;
            std::cout << "Initial trigger camera id: " << id << "." << std::endl;
            return;
        }
        ++id;
    }
}

TriggerLogic::~TriggerLogic(){}

void TriggerLogic::adjustTrigger(CameraId id, bool active) {
    if (active) {
        if (id == _currentTrigger) {
            return;
        }
        if (id == findTopPriorityActiveCamera()) {
            assert(("After starting a camera found that all cameras are not running.", id != INVALID_CAMERA_ID));
            changeTrigger(id, true);
        }
    }
    else {
        if (id == _currentTrigger) {
            changeTrigger(id, false);
        }
    }
}

void TriggerLogic::changeTrigger(CameraId id, bool trigger) {
    std::lock_guard<std::mutex> lock(*_triggerCameraStateMutex);
    if (trigger) {
        if (!allCamerasStopped()) {
            std::cout << "Starting camera with id: " << _currentTrigger << " as trigger." << std::endl;
            (*_cameras)[_currentTrigger]->setTrigger(false);
        }
        else {
            std::cout << "Changing trigger camera from id: " << _currentTrigger << " to id: " << id << "." << std::endl;
        }
        (*_cameras)[id]->setTrigger(true);
        _currentTrigger = id;
    }
    else {
        _currentTrigger = findTopPriorityActiveCamera(id);
        if (!allCamerasStopped()) {
            std::cout << "Trigger camera with id: " << id << " stopped. Changing trigger camera to camera with id: " << _currentTrigger << "." << std::endl;
            (*_cameras)[id]->setTrigger(false);
            (*_cameras)[_currentTrigger]->setTrigger(true);
        }
    }
}

CameraId TriggerLogic::findTopPriorityActiveCamera(CameraId excludeId) const {
    if ((_definedTriggerId != excludeId) && (*_cameras)[_definedTriggerId]->getIsRunning()) {
        return _definedTriggerId;
    }
    CameraId id = 0;
    for(auto camera: *_cameras) 
    {
        if (id != excludeId && camera->getIsRunning()) {
            return id;
        }
        ++id;
    }
    return INVALID_CAMERA_ID;
}

bool TriggerLogic::allCamerasStopped() const{
    return _currentTrigger == INVALID_CAMERA_ID;
}
