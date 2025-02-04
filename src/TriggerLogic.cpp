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
     _triggerCameraStateMutex(triggerCameraStateMutex)
{
    CameraId id = 0;
    for(auto camera: *_cameras) {
        if (camera->getIsTrigger()) {
            _currentTrigger = id;
            _definedTriggerId = id;
            std::cout << "Initial trigger camera id: " << id << std::endl;
        }
        ++id;
    }
    _invalidCameraId = id;
}

TriggerLogic::~TriggerLogic(){}

void TriggerLogic::adjustTrigger(CameraId id, bool active) {
    if (active) {
        if (id == _currentTrigger) {
            return;
        }
        if (id == findTopPriorityActiveCamera()) {
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
     _triggerCameraStateMutex->lock();
    if (trigger) {
        std::cout << "Changing trigger camera from id: " << _currentTrigger << " to id: " << id << std::endl;
        (*_cameras)[_currentTrigger]->setTrigger(false);
        (*_cameras)[id]->setTrigger(true);
        _currentTrigger = id;
    }
    else {
        CameraId _currentTrigger = findTopPriorityActiveCamera();
        if (!allCamerasStopped()) {
            std::cout << "Changing trigger camera from id: " << id << " to id: " << _currentTrigger << std::endl;
            (*_cameras)[id]->setTrigger(false);
            (*_cameras)[_currentTrigger]->setTrigger(true);
        }
    }
     _triggerCameraStateMutex->unlock();
}

CameraId TriggerLogic::findTopPriorityActiveCamera() {
    if ((*_cameras)[_definedTriggerId]->getIsRunning()) {
        return _definedTriggerId;
    }
    CameraId id = 0;
    for(auto camera: *_cameras) {
        if (camera->getIsRunning()) {
            return id;
        }
        ++id;
    }
    assert(id==_invalidCameraId);
    return id;
}

bool TriggerLogic::allCamerasStopped() {
    return _currentTrigger == _invalidCameraId;
}
